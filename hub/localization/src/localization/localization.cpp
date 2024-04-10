#include "localization.h"

Localization::Localization()
{

}

Localization::~Localization()
{
  
}

void Localization::setComunication(std::shared_ptr<AgentCommunication> agentCommunication)
{
    this->agentCommunication=agentCommunication;
}

void Localization::setRingBuffer(std::shared_ptr<ringBuffer> buffer)
{
    this->buffer=buffer;    
}

void Localization::initRingBuffer()
{
    
}

int Localization::init(int argc,char **argv)
{

    cv::Mat R, T, R1, R2, P1, P2, Q;
    cv::Mat map1x, map1y, map2x, map2y;
    // Stereo matching
    cv::Mat rectifiedImage1, rectifiedImage2, disparity;

    
    this->twoCameras=false;
     //init aruco code----------
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return -1;
    }

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }
    int dictionaryId = parser.get<int>("d");

    this->marker_length_m = parser.get<float>("l");

    if (this->marker_length_m<= 0) {
        std::cerr << "marker length must be a positive value in meter" 
                  << std::endl;
        return -1;
    }
    this->baseArenaLength = parser.get<float>("b");
    if (baseArenaLength<= 0) {
        std::cerr << "base arena length must be a positive value in meter" 
                  << std::endl;
        return -1;
    }
    this->heightArenaLength = parser.get<float>("h");
    if (heightArenaLength<= 0) {
        std::cerr << "height arena length must be a positive value in meter" 
                  << std::endl;
        return -1;
    }

    if (parser.has("v")) {
        this->videoInput = parser.get<cv::String>("v");
        if (this->videoInput.empty()) {
            parser.printMessage();
            return -1;
        }
     char* end = nullptr;
        int source = static_cast<int>(std::strtol(this->videoInput.c_str(), &end, \
            10));
        if (!end || end == this->videoInput.c_str()) {
             /*  in_video.open(videoInput); // url
            in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
			in_video.set(cv::CAP_PROP_FPS,20);*/
            
        } else {
            this->in_video.open(source); // id
            
        }
    } else {
        return -1;
    }
    if(parser.has("vv"))
    {
        this->videoInput2 = parser.get<cv::String>("vv");
        if (this->videoInput.empty()) {
            parser.printMessage();
            return -1;
        }
        this->twoCameras=true;
         char* end = nullptr;
        int source = static_cast<int>(std::strtol(this->videoInput2.c_str(), &end, \
            10));
        if (!end || end == this->videoInput2.c_str()) {
             /*  in_video.open(videoInput); // url
            in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
			in_video.set(cv::CAP_PROP_FPS,20);*/
            
        } else {
            this->in_video2.open(source); // id
            
        } 
    }
    this->in_video.set(cv::CAP_PROP_FPS,30);
    




    if (!parser.check()) {
        parser.printErrors();
        return -1;
    }
    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return -1;
    }

     this->dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    
    cv::FileStorage fs("/home/alex/workspace/robotarium-hub/hub/localization/calibration_params.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> this->camera_matrix;
    fs["distortion_coefficients"] >> this->dist_coeffs;

    this-> frame_width = in_video.get(cv::CAP_PROP_FRAME_WIDTH);
	this-> frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT);
    if(this->twoCameras)
    {
    

    }
    return 1;
}


bool Localization::FindArena()
{
    bool ArenaFound=false;
    std::vector<float> reprojectionError;
    cv::Scalar color(0,0,255);
    while (this->in_video.grab()&& ArenaFound==false)
    {
        this->filteredContours.clear();
        this->rvecs.clear();
        this->tvecs.clear();
        this->in_video.retrieve(this->image);
        cv::Mat bothImages;
        cvtColor(this->image,this->grayMat,cv::COLOR_BGR2GRAY);
        this->image.copyTo(this->image_copy);
        //Finding the contours of the arena
        //cv::threshold(this->grayMat,this->binary_image,110,255,cv::CHAIN_APPROX_NONE);
        // Aplicar un desenfoque para reducir el ruido
        GaussianBlur(this->grayMat, this->grayMat, cv::Size(5, 5), 0);
        // Detectar bordes usando el algoritmo de Canny
        cv::Canny(this->grayMat, this->grayMat, 100, 250);
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(this->grayMat,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        
        
        double minContourArea = 2000;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > minContourArea) {
               
            
             std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.04 * cv::arcLength(contour, true), true);

            if (approx.size() == 4 && std::fabs(cv::contourArea(approx)) > minContourArea * 0.9) {  // Check if the contour has 4 vertices (rectangle)
                this->filteredContours.push_back(approx);

                // Optionally, you can draw the rectangles on the image
                cv::polylines(this->image, approx, true, color, 2);
                //save the lines

            }
        }
        }
        std::vector<cv::Point2f> approxCurve;
        std::cout<<"filtered"<<std::endl;
        std::cout<<"filtered size: "<<this->filteredContours.size()<<std::endl;
        if (this->filteredContours.size()>0){
            for( long unsigned int i=0;i<this->filteredContours.size();i++){
                cv::approxPolyDP(this->filteredContours[i], approxCurve, 0.04 * cv::arcLength(this->filteredContours[i], true), true);
                cv::drawContours(this->image,this->filteredContours,i,color,2);
                for (size_t j = 0; j < approxCurve.size(); ++j) {
                    cv::circle(this->grayMat, approxCurve[j], 5, this->cornerColors[j], -1);
                }
            }
            std::cout<<"Estimating Arena Position"<<std::endl;
            if(this->EstimateArenaPosition(approxCurve, this->baseArenaLength,this->heightArenaLength, this->rvecs, this->tvecs))
            {
               
                this->RobotariumData.x.push_back(this->tvecs[0][0]-this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]-this->heightArenaLength/2);
                this->RobotariumData.x.push_back(this->tvecs[0][0]+this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]-this->heightArenaLength/2);
                this->RobotariumData.x.push_back(this->tvecs[0][0]+this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]+this->heightArenaLength/2);
                this->RobotariumData.x.push_back(this->tvecs[0][0]-this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]+this->heightArenaLength/2);
               
                // this->RobotariumData.x.push_back(this->tvecs[0][0]-this->baseArenaLength/2);
                // this->RobotariumData.y.push_back(this->tvecs[0][1]);
                // this->RobotariumData.x.push_back(this->tvecs[0][0]+this->baseArenaLength/2);
                // this->RobotariumData.y.push_back(this->tvecs[0][1]);
                // this->RobotariumData.x.push_back(this->tvecs[0][0]+this->baseArenaLength/2);
                // this->RobotariumData.y.push_back(this->tvecs[0][1]-this->heightArenaLength);
                // this->RobotariumData.x.push_back(this->tvecs[0][0]-this->baseArenaLength/2);
                // this->RobotariumData.y.push_back(this->tvecs[0][1]-this->heightArenaLength);
                //agentCommunication->setRobotariumData(RobotariumData);
                ArenaFound=true;
                std::cout<<"ARENA FOUND"<<std::endl;
                for (long unsigned int i = 0; i < rvecs.size(); ++i) {
                    
                    cv::drawFrameAxes(this->image,  this->camera_matrix, this->dist_coeffs, this->rvecs[i], this->tvecs[i], 0.1);
                }
                
            }
        }
        imshow("Pose estimation", this->image);
                char key = (char)cv::waitKey(1);
                if (key == 27)
                    break;
         

    }    
    return ArenaFound;
}

void Localization::FindRobot()
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<float> reprojectionError;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->adaptiveThreshWinSizeMin = 4;
    parameters->adaptiveThreshWinSizeMax = 24;
    parameters->adaptiveThreshWinSizeStep = 4;
    // parameters->adaptiveThreshConstant = 7;
    // parameters->minMarkerPerimeterRate = 0.8;
    // parameters->maxMarkerPerimeterRate = 1.2;
    parameters->polygonalApproxAccuracyRate = 0.02;
    // parameters->minCornerDistanceRate = 0.05;
    // parameters->minDistanceToBorder = 3;
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    // parameters->minMarkerDistanceRate = 2.5; // Puede ajustarse según la distancia de los marcadores a la cámara
    // parameters->cornerRefinementWinSize = 5; // Tamaño de la ventana de refinamiento de esquinas
    parameters->cornerRefinementMaxIterations = 30; // Número máximo de iteraciones de refinamiento de esquinas
    parameters->cornerRefinementMinAccuracy = 0.01; // Precisión mínima de refinamiento de esquinas
    parameters->detectInvertedMarker = true; // Permitir la detección de marcadores invertidos si es necesario
cv::Scalar color(0,0,255);
    cv::Mat smoothed_image;
    std::vector<std::vector<cv::Point2f>> rejected;
    while (this->in_video.grab())
    {
        this->rvecs.clear();
        this->tvecs.clear();
        this->in_video.retrieve(this->image);
        
        cvtColor(this->image,this->grayMat,cv::COLOR_BGR2GRAY);
        cv::threshold(this->grayMat, this->binary_image, 50, 255, cv::THRESH_BINARY);

        cv::GaussianBlur(this->grayMat, this->grayMat, cv::Size(3,3), 0.0,0.0); // Tamaño del kernel y desviación estándar
        this->image.copyTo(this->image_copy);
        
        //falta estimar la posicion de la camara con respecto a la arena

        
        
        //cv::aruco::detectMarkers(this->binary_image, this->dictionary, corners, ids);
        cv::aruco::detectMarkers(this->grayMat, this->dictionary, corners, ids, parameters, rejected);
       
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(this->grayMat, corners, ids);
            
            // MyestimatePoseSingleMarkers(corners, this->marker_length_m,
            //         camera_matrix, dist_coeffs, this->rvecs, this->tvecs,reprojectionError);
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m, camera_matrix, dist_coeffs, this->rvecs, this->tvecs);
            // cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
            //         camera_matrix, dist_coeffs, rvecs, tvecs);
           
            // // Draw axis for each marker
             
            for(long unsigned int i=0; i < ids.size(); i++)
            {
                 cv::aruco::drawAxis(this->grayMat, this->camera_matrix, this->dist_coeffs,
                        this->rvecs[i], this->tvecs[i], 0.1);
                
                        
                this->data.x=this->tvecs[i](0);
                
                this->data.y=this->tvecs[i](1);
               
                this->data.z=this->tvecs[i](2);
                cv::Rodrigues(this->rvecs[i], this->rot_mat);
                this->ang_yaw=atan2(this->rot_mat.at<double>(1,0),this->rot_mat.at<double>(0,0));//range [-pi,pi]
                this->data.yaw=this->ang_yaw;

               // cout<<"data y "<<data.y<<endl;
                this->data.id=ids.at(i);
                std::cout<<"data id "<<this->data.id<<std::endl;  
                std::cout<<"data x "<<this->data.x<<std::endl;  
                std::cout<<"data y "<<this->data.y<<std::endl;
                //arucoInfo.at(i)=data;
                this->buffer->push(data);
                // pthread_cond_signal(&listBlock);
                
            }
            std::vector<cv::Point2f> approxCurve;
             for( long unsigned int i=0;i<this->filteredContours.size();i++){
                cv::approxPolyDP(this->filteredContours[i], approxCurve, 0.04 * cv::arcLength(this->filteredContours[i], true), true);
                cv::drawContours(this->image,this->filteredContours,i,color,2);
                for (size_t j = 0; j < approxCurve.size(); ++j) {
                    cv::circle(this->grayMat, approxCurve[j], 5, this->cornerColors[j], -1);
                }
            }
        }

         
         imshow("Pose estimation", this->grayMat);
            char key = (char)cv::waitKey(1);
            if (key == 27)
                break;
    }
    in_video.release();
}

void Localization::MyestimatePoseSingleMarkers(const std::vector<std::vector<cv::Point2f>> &corners, float markerLength, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, std::vector<float> &reprojectionError)
{
    std::vector<cv::Point3f> objPoints;
    cv::Point3f p;
    objPoints.push_back(cv::Point3f(-markerLength/2,markerLength/2,0));
    objPoints.push_back(cv::Point3f(markerLength/2,markerLength/2,0));
    objPoints.push_back(cv::Point3f(markerLength/2,-markerLength/2,0));
    objPoints.push_back(cv::Point3f(-markerLength/2,-markerLength/2,0));

    for (auto c = corners.begin(); c != corners.end(); c++)
    {
        cv::Mat rvec, tvec;
        
        if (c->size() >= 4)
        {
            cv::Mat rvec, tvec;
            cv::solvePnP(objPoints,*c , cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            rvecs.push_back(rvec);
            tvecs.push_back(tvec);

            // Resto del código para el cálculo del error de reproyección
            // ...
        }
        else
        {
        }
    }
}

bool Localization::EstimateArenaPosition(const std::vector<cv::Point2f>& corners, float baseLength,float heightLength,
                               std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs)
{

    std::vector<cv::Point3f> objPoints;
    cv::Point3f p;
    objPoints.push_back(cv::Point3f(-baseLength/2,heightLength/2,0));
    objPoints.push_back(cv::Point3f(baseLength/2,heightLength/2,0));
    objPoints.push_back(cv::Point3f(baseLength/2,-heightLength/2,0));
    objPoints.push_back(cv::Point3f(-baseLength/2,-heightLength/2,0));

    // objPoints.push_back(cv::Point3f(0,heightLength,0));
    // objPoints.push_back(cv::Point3f(baseLength,heightLength,0));
    // objPoints.push_back(cv::Point3f(baseLength,0,0));
    // objPoints.push_back(cv::Point3f(0,-heightLength,0));
        cv::Mat rvec, tvec;
        
        if ((corners.size() == 4) && (this->camera_matrix.rows > 0) && (this->dist_coeffs.rows > 0))
        {
            std::cout<< "corners: "<<corners<<std::endl;
            cv::Mat rvec, tvec;
            //cv::solvePnP(objPoints,corners , this->camera_matrix, this->dist_coeffs, rvec, tvec,cv::SOLVEPNP_ITERATIVE);
            cv::solvePnP(objPoints,corners , this->camera_matrix, this->dist_coeffs, rvec, tvec,false,cv::SOLVEPNP_IPPE_SQUARE);
            std::cout<< "tvec: "<<tvec<<std::endl;
            rvecs.push_back(rvec);
            tvecs.push_back(tvec);
            return true;

        }
        else
        {

            std::cout << "wrong number of corners detected" << std::endl;
            return false;
        }
}

ArenaLimits Localization::getRobotariumData()
{
    return this->RobotariumData;
}
