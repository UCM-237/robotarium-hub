
/*
 * Copyright (c) 2019 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
    #include "common.hh"
    #include <opencv2/opencv.hpp>
    #include <opencv2/aruco.hpp>
    #include <opencv2/core.hpp> //PREDEFINED_DICTIONARY
    #include <opencv2/highgui/highgui.hpp>  // Video write
    #include <zmqpp/zmqpp.hpp>
    #include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std;
using namespace tinyxml2;



namespace {
const char* about = "Pose estimation of ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0'}"
	;
}
int registerAgent();
void *dataAruco(void *arg);
void SetupRobots();
vector<cv::Point3f> getCornersInCameraWorld(double side, cv::Vec3d rvec, cv::Vec3d tvec);

void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
string convertToString(char* a, int size)
{
    string s = a;
    return s;
}
//----prototipos-----//
void error(const char *msg)
{
    perror(msg);
    exit(-1);
}
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

   // assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);

}

struct record_data//struct for share information between threads
{
    int id; //id of every robot
    double x, y , z;
    double yaw;
    int n;  //counter for know how many robots there are
    
};
list<record_data> arucoInfo;//list for save the information of all the arucos
list<record_data>::iterator it, it2;//for save data

struct signalAlarm{
    bool finish=false;
};
signalAlarm end_thread1,end_thread2,end_thread3;
enum {r1, r2, r3, r4,r5};
     //definition of robots
Robot robot1,robot2,robot3,robot4;//se define la clase para los distintos robots.

static const string PUBLISH_ENDPOINT = "tcp://*:5557";
static const string HUB_IP = "127.0.0.1";




pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER,mutex2_ = PTHREAD_MUTEX_INITIALIZER;
pthread_t  _detectAruco; 
pthread_cond_t listBlock;
bool EmptyList = false;

const size_t bufferSize = 2;
std::vector<record_data> buffer(bufferSize);
size_t readPos = 0;
size_t writePos = 0;
bool full = false;

pthread_mutex_t bufferMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t bufferNotEmpty = PTHREAD_COND_INITIALIZER;
void push(const record_data& data) {
    pthread_mutex_lock(&bufferMutex);
    buffer[writePos] = data;
    writePos = (writePos + 1) % bufferSize;
    if (writePos == readPos) {
        full = true;
        readPos = (readPos + 1) % bufferSize;
    }
   pthread_cond_signal(&bufferNotEmpty); // Señalar que hay datos disponibles
    pthread_mutex_unlock(&bufferMutex);
}

record_data pop() {
    pthread_mutex_lock(&bufferMutex);
    while (!full && writePos == readPos) {
        // Esperar hasta que haya datos disponibles en el buffer
        pthread_cond_wait(&bufferNotEmpty, &bufferMutex);
    }
    if (!full && writePos == readPos) {
        // El buffer está vacío
        pthread_mutex_unlock(&bufferMutex);
        throw std::runtime_error("Buffer is empty.");
    }
    record_data data = buffer[readPos];
    readPos = (readPos + 1) % bufferSize;
    full = false;
    pthread_mutex_unlock(&bufferMutex);
    return data;
}

void MyestimatePoseSingleMarkers(const std::vector<std::vector<cv::Point2f> >& corners, float markerLength,
                               const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                               std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                               std::vector<float>& reprojectionError);
int main(int argc,char **argv)
{
    
    record_data data;//struct for save in linked list

    //init aruco code----------
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 1;
    }

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");

    float marker_length_m = parser.get<float>("l");
    int wait_time = 10;
    if (marker_length_m<= 0) {
        std::cerr << "marker length must be a positive value in meter" 
                  << std::endl;
        return 1;
    }
    cv::String videoInput = "0";//se selecciona la entrada de la camara
    cv::VideoCapture in_video;

    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
            parser.printMessage();
            return 1;
        }
     char* end = nullptr;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        if (!end || end == videoInput.c_str()) {
          /*  in_video.open(videoInput); // url
            in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
			in_video.set(cv::CAP_PROP_FPS,20);*/
            
        } else {
            in_video.open(source); // id
            
        }
    } else {
        in_video.open(0);
    }
    in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('D', 'I', 'V', 'X'));
    //in_video.set(cv::CAP_PROP_FRAME_WIDTH,800);
    // in_video.set(cv::CAP_PROP_FRAME_HEIGHT,600);
     in_video.set(cv::CAP_PROP_FPS,20.0);
     in_video.set(cv::CAP_PROP_AUTOFOCUS,0);
     in_video.set(cv::CAP_PROP_SETTINGS,1);
    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }
    
    cv::Mat grayMat;
    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    
    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    cv::Mat rotated_image;
    //std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    //std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
    // cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(frame_width,frame_height).0);
    //---------------------------------end aruco code-----
    arucoInfo.clear();
   
    
    
     
     
    int frame_width = in_video.get(cv::CAP_PROP_FRAME_WIDTH);
	int frame_height = in_video.get(cv::CAP_PROP_FRAME_HEIGHT);
    cout<<frame_width<<endl;
    cout<<frame_height<<endl;
    cout<<in_video.get(cv::CAP_PROP_FPS)<<endl;;
    cout<<in_video.get(cv::CAP_PROP_FOURCC)<<endl;
    cv::Size S = cv::Size(800,600);
    cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),30, S,1);

    cv :: Mat Raux = cv::Mat::zeros(3,3,CV_64F);;
	double ang_roll = 0;
	double ang_pitch = 0;
	double ang_yaw = 0;

    
       // compute rot_mat
    cv::Mat rot_mat,rot_mat2;
    cv::Mat binary_image;//for detect the arena   
    pthread_create(&_detectAruco,NULL,dataAruco,NULL);//create thread for store the values of the markers
    
    //registerAgent();
     data.n =  0;
    //main loop
    std::vector<cv::Scalar> cornerColors = {
    cv::Scalar(0, 255, 0),  // Verde para la primera esquina
    cv::Scalar(255, 0, 0),  // Azul para la segunda esquina
    cv::Scalar(0, 0, 255),  // Rojo para la tercera esquina
    cv::Scalar(255, 255, 0) // Amarillo para la cuarta esquina (ajusta según sea necesario)
    };
    while (in_video.grab())
    {
        in_video.retrieve(image);
        
        cvtColor(image,grayMat,cv::COLOR_BGR2GRAY);
        image.copyTo(image_copy);
        //Finding the contours of the arena
        cv::threshold(grayMat,binary_image,50,255,cv::CHAIN_APPROX_NONE);
        //cv::adaptiveThreshold(grayMat, binary_image, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 11, 2);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(binary_image,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        cv::Scalar color(0,0,255);
        std::vector<std::vector<cv::Point>> filteredContours;
        double minContourArea = 20000;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > minContourArea) {
                filteredContours.push_back(contour);
            }
        }
        std::vector<cv::Point2f> approxCurve;
        
        for(int i=0;i<filteredContours.size();i++){
            cv::approxPolyDP(filteredContours[i], approxCurve, 0.04 * cv::arcLength(filteredContours[i], true), true);
            cv::drawContours(image_copy,filteredContours,i,color,2);
            for (size_t j = 0; j < approxCurve.size(); ++j) {
                 cv::circle(image_copy, approxCurve[j], 5, cornerColors[j], -1);
            }
        }
        
        //falta estimar la posicion de la camara con respecto a la arena

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(grayMat, dictionary, corners, ids);
       
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            std::vector<float> reprojectionError;
            std::vector<cv::Vec3d> rvecs, tvecs,rvecs2,tvecs2;
            MyestimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs2, tvecs2,reprojectionError);
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs, tvecs);
            // cout<<"corners: "<<getCornersInCameraWorld(marker_length_m, rvecs[0],tvecs[0])<<endl;
            /*std::cout << "Translation: " << tvecs[0]
                << "\tRotation: " << rvecs[0] 
                << std::endl;
            */
            // Draw axis for each marker
            pthread_mutex_lock(&mutex_);
            for(int i=0; i < ids.size(); i++)
            {
                std::cout<<"tvec: "<<tvecs[i]<<std::endl;
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);
                        
                data.x=tvecs[i](0);
                
                data.y=tvecs[i](1);
                //cout<<ids.at(i)<<"datax "<<data.x<<","<<data.y<<endl;
                data.z=tvecs[i](2);
                cv::Rodrigues(rvecs[i], rot_mat);
                ang_yaw=atan2(rot_mat.at<double>(1,0),rot_mat.at<double>(0,0));
                data.yaw=ang_yaw;

               // cout<<"data y "<<data.y<<endl;
                data.id=ids.at(i);
                data.n++;
                
                //arucoInfo.at(i)=data;
                push(data);
                EmptyList=false;
                pthread_cond_signal(&listBlock);
                
                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
				
              
                
                
        

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x: " << std::setw(8) << tvecs[0](0);
                            
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
                           
            }

            int status = pthread_mutex_unlock (&mutex_);
                if (status != 0)
                    exit(status);
            
        }

         //outputVideo << image_copy;
        // cv::resize(image,image,cv::Size(1200,1600));
      
        video.write(image_copy);
        video<<image;
         imshow("Pose estimation", image_copy);
            char key = (char)cv::waitKey(1);
            if (key == 27)
                break;
    }

    in_video.release();
    video.release();
    //destroyALLWindows();
    
    return 0;
}


void *dataAruco(void *arg)
{//thread function
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;
    //const string endpoint = "tcp://127.0.0.1:4242"; //5555
    // initialize the 0MQ context
    zmqpp::context context;

    // generate a push socket
    zmqpp::socket_type type = zmqpp::socket_type::publish;
    zmqpp::socket publisher (context, type);
   
    // open the connection
    cout << "Binding to " << PUBLISH_ENDPOINT << "..." << endl;
     publisher.bind(PUBLISH_ENDPOINT);
    

    
    //while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected
    std::string data2;
    //main code for read variables
    cout<<"hola"<<endl;
     while (true) {
        for(int i=0;i<2;i++){
        record_data data = pop();
        string topic ="data";
        string id = to_string(data.id);
        string x = to_string(data.x);
        string y = to_string(data.y);
        string yaw = to_string(data.yaw);
        
        string sep = "/";
        data2=  x+ sep +y+sep+yaw;
        cout<<id+sep+data2<<endl;
        json message;
        json position;
    
        position[id]["x"] =x;
        position[id]["y"] =y;
        position[id]["yaw"] =yaw;
        zmqpp::message_t ztopic;
        ztopic<<topic;
        publisher.send(topic,0);
        message["topic"]="position";
        //message["operation"] = "position";
        message["source_id"] = "Camara_0";
        message["payload"] = position;
        message["timestamp"] = 1000 * time(nullptr);
        std::string jsonStr = message.dump();
        zmqpp::message_t zmqMessage;
        
        
        zmqMessage<<jsonStr;
        
        
        publisher.send(zmqMessage,0);
        }
        usleep(400*1000);
    }
    
    
    publisher.close();
    pthread_exit(NULL);
    return NULL;
}

int registerAgent()
{
    //Register to the control hub
    cout<<"Registering Localization"<<endl;
    zmqpp::context context;

    // generate a push socket
    zmqpp::socket_type type = zmqpp::socket_type::request;
    zmqpp::socket control (context, type);
    cout << "connecting" << endl;
    control.connect("tcp://127.0.0.1:5555");

    json message;
    json size;
    // size["x"] =x;
    // size["y"] =y;
    message["operation"] = "hello";
    message["source_id"] = "0";
    message["payload"]["url"] = "tcp://127.0.0.1:5557";
    message["payload"]["size"] = size;
    message["timestamp"] = 1000 * time(nullptr);
    std::string jsonStr = message.dump();

    zmqpp::message_t zmqMessage;
    // memcpy(zmqMessage.data(), jsonStr.c_str(), jsonStr.size());
    zmqMessage<<jsonStr;
    control.send(zmqMessage);


    zmqpp::message_t response;
    
    control.close();
    return 0;
}


vector<cv::Point3f> getCornersInCameraWorld(double side, cv::Vec3d rvec, cv::Vec3d tvec){

     double half_side = side/2;


     // compute rot_mat
     cv::Mat rot_mat;
     cv::Rodrigues(rvec, rot_mat);

     // transpose of rot_mat for easy columns extraction
     cv::Mat rot_mat_t = rot_mat.t();

     // the two E-O and F-O vectors
     double * tmp = rot_mat_t.ptr<double>(0);
     cv::Point3f camWorldE(tmp[0]*half_side,
                       tmp[1]*half_side,
                       tmp[2]*half_side);

     tmp = rot_mat_t.ptr<double>(1);
     cv::Point3f camWorldF(tmp[0]*half_side,
                       tmp[1]*half_side,
                       tmp[2]*half_side);

     // convert tvec to point
     cv::Point3f tvec_3f(tvec[0], tvec[1], tvec[2]);

     // return vector:
     vector<cv::Point3f> ret(4,tvec_3f);

     ret[0] +=  camWorldE + camWorldF;
     ret[1] += -camWorldE + camWorldF;
     ret[2] += -camWorldE - camWorldF;
     ret[3] +=  camWorldE - camWorldF;

     return ret;
}

void MyestimatePoseSingleMarkers(const std::vector<std::vector<cv::Point2f>> &corners, float markerLength, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs, std::vector<float> &reprojectionError)
{
    vector<cv::Point3f> objPoints;
    cv::Point3f p;
    objPoints.push_back(cv::Point3f(-markerLength/2,markerLength/2,0));
    objPoints.push_back(cv::Point3f(markerLength/2,markerLength/2,0));
    objPoints.push_back(cv::Point3f(markerLength/2,-markerLength/2,0));
    objPoints.push_back(cv::Point3f(-markerLength/2,-markerLength/2,0));

    for (auto c = corners.begin(); c != corners.end(); ++c)
    {
        cv::Mat rvec, tvec;
        
        if (c->size() >= 4)
        {
            cv::Mat rvec, tvec;
            cv::solvePnP(objPoints,*c , cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
            std::cout<< "tvec: "<<tvec<<std::endl;
            rvecs.push_back(rvec);
            tvecs.push_back(tvec);

            // Resto del código para el cálculo del error de reproyección
            // ...
        }
        else
        {
        }
        
        // Calcular el error de reproyección
        // for (size_t i = 0; i < projectedPoints.size(); ++i)
        // {
        //     float error = cv::norm(c[i][0] - projectedPoints[i][0], c[i][1] - projectedPoints[i][1]);

        //     sum += error;
        // }

        // // Calcular el error promedio y almacenarlo
        // float avgError = sum / projectedPoints.size();
        // reprojectionError.push_back(avgError);
    }
    
}
