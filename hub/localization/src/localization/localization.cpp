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

    if (marker_length_m<= 0) {
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
    this->weightArenaLength = parser.get<float>("w");
    if (weightArenaLength<= 0) {
        std::cerr << "weight arena length must be a positive value in meter" 
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
     this->in_video.set(cv::CAP_PROP_FPS,30.0);
     this->in_video.set(cv::CAP_PROP_AUTOFOCUS,0);
     this->in_video.set(cv::CAP_PROP_SETTINGS,1);

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
    
    cv::FileStorage fs("/home/admin/workspace/robotarium-hub/hub/localization/calibration_params.yml", cv::FileStorage::READ);
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

    while (this->in_video.grab() && ArenaFound==false)
    {

        this->in_video.retrieve(this->image);
        cv::Mat bothImages;
        cvtColor(this->image,this->grayMat,cv::COLOR_BGR2GRAY);
        this->image.copyTo(this->image_copy);
        //Finding the contours of the arena
        cv::threshold(this->grayMat,this->binary_image,200,255,cv::CHAIN_APPROX_NONE);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(binary_image,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        cv::Scalar color(0,0,255);
        std::vector<std::vector<cv::Point>> filteredContours;
        double minContourArea = 100000;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > minContourArea) {
               
            
             std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.04 * cv::arcLength(contour, true), true);

            if (approx.size() == 4 && std::fabs(cv::contourArea(approx)) > minContourArea * 0.9) {  // Check if the contour has 4 vertices (rectangle)
                filteredContours.push_back(approx);

                // Optionally, you can draw the rectangles on the image
                cv::polylines(image_copy, approx, true, color, 2);
            }
        }
        }
        std::vector<cv::Point2f> approxCurve;
        std::cout<<"filtered"<<std::endl;
        std::cout<<"filtered size: "<<filteredContours.size()<<std::endl;
        if (filteredContours.size()>0){
            for( long unsigned int i=0;i<filteredContours.size();i++){
                cv::approxPolyDP(filteredContours[i], approxCurve, 0.04 * cv::arcLength(filteredContours[i], true), true);
                cv::drawContours(this->image_copy,filteredContours,i,color,2);
                for (size_t j = 0; j < approxCurve.size(); ++j) {
                    cv::circle(this->image_copy, approxCurve[j], 5, this->cornerColors[j], -1);
                }
            }
            std::cout<<"Estimating Arena Position"<<std::endl;
            if(this->EstimateArenaPosition(approxCurve, this->baseArenaLength,this->weightArenaLength, this->rvecs, this->tvecs))
            {
                this->RobotariumData.x.push_back(this->tvecs[0][0]-this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]+this->weightArenaLength/2);
                this->RobotariumData.x.push_back(this->tvecs[0][0]+this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]+this->weightArenaLength/2);
                this->RobotariumData.x.push_back(this->tvecs[0][0]+this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]-this->weightArenaLength/2);
                this->RobotariumData.x.push_back(this->tvecs[0][0]-this->baseArenaLength/2);
                this->RobotariumData.y.push_back(this->tvecs[0][1]-this->weightArenaLength/2);

               // agentCommunication->setRobotariumData(RobotariumData);
                ArenaFound=true;
                for (long unsigned int i = 0; i < rvecs.size(); ++i) {
                    auto rvec = rvecs[i];
                    auto tvec = tvecs[i];
                    cv::drawFrameAxes(this->image_copy,  this->camera_matrix, this->dist_coeffs, rvec, tvec, 0.1);
                }
                
            }
        }
        imshow("Pose estimation", image_copy);
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
   // pthread_create(&_detectAruco,NULL,AgentCommunication::sendArucoPosition,static_cast<void*>(this->agentCommunication));

    while (this->in_video.grab())
    {
        this->rvecs.clear();
        this->tvecs.clear();
        this->in_video.retrieve(this->image);
        
        cvtColor(this->image,this->grayMat,cv::COLOR_BGR2GRAY);
        this->image.copyTo(this->image_copy);
        
        //falta estimar la posicion de la camara con respecto a la arena

        
        
        cv::aruco::detectMarkers(this->grayMat, this->dictionary, corners, ids);
       
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(this->image_copy, corners, ids);
            
            MyestimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, this->rvecs, this->tvecs,reprojectionError);
            // cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
            //         camera_matrix, dist_coeffs, rvecs, tvecs);
           
            // // Draw axis for each marker
             
            for(long unsigned int i=0; i < ids.size(); i++)
            {
                 cv::aruco::drawAxis(this->image_copy, this->camera_matrix, this->dist_coeffs,
                        this->rvecs[i], this->tvecs[i], 0.1);
                
                        
                this->data.x=this->tvecs[i](0);
                
                this->data.y=this->tvecs[i](1);
                //cout<<ids.at(i)<<"datax "<<data.x<<","<<data.y<<endl;
                this->data.z=this->tvecs[i](2);
                cv::Rodrigues(this->rvecs[i], this->rot_mat);
                this->ang_yaw=atan2(this->rot_mat.at<double>(1,0),this->rot_mat.at<double>(0,0));//range [-pi,pi]
                this->data.yaw=this->ang_yaw;

               // cout<<"data y "<<data.y<<endl;
                this->data.id=ids.at(i);
                
                //arucoInfo.at(i)=data;
                this->buffer->push(data);
                // pthread_cond_signal(&listBlock);
                
            }

            
        }

      
         imshow("Pose estimation", image_copy);
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

bool Localization::EstimateArenaPosition(const std::vector<cv::Point2f>& corners, float baseLength,float weightLength,
                               std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs)
{

    std::vector<cv::Point3f> objPoints;
    cv::Point3f p;
    // objPoints.push_back(cv::Point3f(-baseLength/2,weightLength/2,0));
    // objPoints.push_back(cv::Point3f(baseLength/2,weightLength/2,0));
    // objPoints.push_back(cv::Point3f(baseLength/2,-weightLength/2,0));
    // objPoints.push_back(cv::Point3f(-baseLength/2,-weightLength/2,0));

    objPoints.push_back(cv::Point3f(0,weightLength,0));
    objPoints.push_back(cv::Point3f(baseLength,weightLength,0));
    objPoints.push_back(cv::Point3f(baseLength,0,0));
    objPoints.push_back(cv::Point3f(0,-weightLength,0));
        cv::Mat rvec, tvec;
        
        if ((corners.size() >= 4 && corners.size() < 5) && (this->camera_matrix.rows > 0) && (this->dist_coeffs.rows > 0))
        {
            std::cout<< "corners: "<<corners<<std::endl;
            cv::Mat rvec, tvec;
            cv::solvePnP(objPoints,corners , this->camera_matrix, this->dist_coeffs, rvec, tvec,cv::SOLVEPNP_ITERATIVE);
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
