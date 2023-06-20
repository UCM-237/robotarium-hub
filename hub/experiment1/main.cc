/*
** listener.c -- a datagram sockets "server" demo
*/


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
    // in_video.set(cv::CAP_PROP_FPS,20.0);
    // in_video.set(cv::CAP_PROP_AUTOFOCUS,0);
    // in_video.set(cv::CAP_PROP_SETTINGS,1);
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
    cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),30, S,1);

    cv :: Mat Raux = cv::Mat::zeros(3,3,CV_64F);;
	double ang_roll = 0;
	double ang_pitch = 0;
	double ang_yaw = 0;

    
       // compute rot_mat
    cv::Mat rot_mat,rot_mat2;
    
   
    pthread_create(&_detectAruco,NULL,dataAruco,NULL);//create thread for store the values of the markers
    sleep(2);
    registerAgent();
     data.n =  0;
    //main loop
    while (in_video.grab())
    {
        in_video.retrieve(image);
        
        cvtColor(image,grayMat,cv::COLOR_BGR2GRAY);
        //cv::resize(image,image,cv::Size(resized_width,resized_height));
        image.copyTo(image_copy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(grayMat, dictionary, corners, ids);
       
        // if at least one marker detected
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image, corners, ids);

            std::vector<cv::Vec3d> rvecs, tvecs;

            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs, tvecs);
            // cout<<"corners: "<<getCornersInCameraWorld(marker_length_m, rvecs[0],tvecs[0])<<endl;
            /*std::cout << "Translation: " << tvecs[0]
                << "\tRotation: " << rvecs[0] 
                << std::endl;
            */
            // Draw axis for each marker
            for(int i=0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);
                        
                data.x=tvecs[i](0);
                
                data.y=tvecs[i](1);
                //cout<<ids.at(i)<<"datax "<<data.x<<","<<data.y<<endl;
                data.z=tvecs[i](2);
                cv::Rodrigues(rvecs[i], rot_mat);
                ang_yaw=atan2(rot_mat.at<double>(1,0),rot_mat.at<double>(0,0));
                /*     r11=rot_mat.at<double>(0,0);
            r12=rot_mat.at<double>(0,1);
            r13=rot_mat.at<double>(0,2);
            r21=rot_mat.at<double>(1,0);
            r22=rot_mat.at<double>(1,1);
            r23=rot_mat.at<double>(1,2);
            r31=rot_mat.at<double>(2,0);
            r32=rot_mat.at<double>(2,1);
            r33=rot_mat.at<double>(2,2);*/

             //   data.rx=rvecs[i](0);
              //  data.ry=rvecs[i](1);
                data.yaw=ang_yaw;

               // cout<<"data y "<<data.y<<endl;
                data.id=ids.at(i);
                data.n++;
                pthread_mutex_lock(&mutex_);
                arucoInfo.push_back(data);
                EmptyList=false;
                pthread_cond_signal(&listBlock);
                int status = pthread_mutex_unlock (&mutex_);
                if (status != 0)
                    exit(status);
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
            
        }

        pthread_mutex_lock(&mutex_);
        EmptyList=true;
        arucoInfo.erase(arucoInfo.begin(),arucoInfo.end());
        pthread_mutex_unlock (&mutex_);
         //outputVideo << image_copy;
        // cv::resize(image,image,cv::Size(1200,1600));
      
        video.write(image_copy);
        //video<<image;
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




/*def register(self, server_url: str) -> None: #Hello op
    '''Register to the control hub'''
    print(f'Registering agent')
    self.control.connect(server_url)
    self.control.send_json({
      'operation': 'hello',
      'source_id': self.id,
      'payload': {
        'url' : f'{self.proto}://{self.ip}:{self.port}'
      },
      'timestamp': 1000*time.time(),
    })
    response = self.control.recv_json()
    self.connected = response['result'] == 'ok'*/

void *dataAruco(void *arg)
{//thread function

    //const string endpoint = "tcp://127.0.0.1:4242"; //5555
    // initialize the 0MQ context
    zmqpp::context context;

    // generate a push socket
    zmqpp::socket_type type = zmqpp::socket_type::publish;
    zmqpp::socket publisher (context, type);
   
    // open the connection
    cout << "Binding to " << PUBLISH_ENDPOINT << "..." << endl;
     publisher.bind(PUBLISH_ENDPOINT);
    

    
    while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected
    std::string data;
    //main code for read variables
    while(true){
        bool ExitCondition = false;
        if(arucoInfo.size()>0)
        {
            
            for(it=arucoInfo.begin();it !=arucoInfo.end();it++)
            {   if(arucoInfo.size()>0)
                { 
                pthread_mutex_lock(&mutex_);
                while(EmptyList)
                {
                    pthread_cond_wait(&listBlock,&mutex_);
                    ExitCondition=true;
                }
                if (ExitCondition)
                {   
                    pthread_mutex_unlock (&mutex_);
                    ExitCondition=false;
                    break;
                }

                /*longToBytes(it->id,&data[0]);
                doubleToBytes(it->x,&data[4]);
                doubleToBytes(it->y,&data[12]);
                doubleToBytes(it->yaw,&data[20]);*/
                string topic ="position";
                string id = to_string(it->id);
                string x = to_string(it->x);
                string y = to_string(it->y);
                string yaw = to_string(it->yaw);
                
                string sep = "/";
                data=  x+ sep +y+sep+yaw;
                cout<<data<<endl;
                json message;
                

                 message["operation"] = "position";
                message["source_id"] = "0";
                message["payload"] = data;
                message["timestamp"] = 1000 * time(nullptr);
                std::string jsonStr = message.dump();
                zmqpp::message_t zmqMessage;
                 zmqpp::message_t ztopic;
                zmqMessage<<jsonStr;
                ztopic<<topic;
                //publisher.send(ztopic);
                publisher.send(zmqMessage);
    // self.control.send_json({
    //   'operation': 'hello',
    //   'source_id': self.id,
    //   'payload': {
    //     'url' : f'tcp://mi_ip:5555'
    //   },
    //   'timestamp': 1000*time.time(),
    // })

                pthread_mutex_unlock (&mutex_);
                }
                else{
                    break;
                }

                }
           
        }
	    
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
    message["operation"] = "Camera";
    message["source_id"] = "1";
    message["payload"]["url"] = "tcp://127.0.0.1:5557";
    message["timestamp"] = 1000 * time(nullptr);
    std::string jsonStr = message.dump();

    zmqpp::message_t zmqMessage;
    // memcpy(zmqMessage.data(), jsonStr.c_str(), jsonStr.size());
    zmqMessage<<jsonStr;
    control.send(zmqMessage);


    zmqpp::message_t response;
    //control.receive(response);

   /*std::string jsonStr(static_cast<const char*>(response.data()), response.size());
    json jsonResponse = json::parse(jsonStr);
    std::string result = jsonResponse["result"].get<std::string>();
    bool connected = (result == "ok");*/
    
    control.close();
    return 0;
}


vector<cv::Point3f> getCornersInCameraWorld(double side, cv::Vec3d rvec, cv::Vec3d tvec){

     double half_side = side/2;


     // compute rot_mat
     cv::Mat rot_mat

;
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
