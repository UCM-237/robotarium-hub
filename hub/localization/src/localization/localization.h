#pragma once
#include "common.h"
#include "comunication.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp> //PREDEFINED_DICTIONARY
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <zmqpp/zmqpp.hpp>
#include <nlohmann/json.hpp>
#include <memory>
using json = nlohmann::json;
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
        "{b        |      | Actual base arena length in m }"
        "{w        |      | Actual weight arena length in m }"
        "{h        |false | Print help }"
	;
}
class Localization

{

    public:
        Localization();
        ~Localization();
        void initRingBuffer();
        int init(int argc,char **argv);
        void registerAgent();
        void FindArena();
        void FindRobot();
        void MyestimatePoseSingleMarkers(const std::vector<std::vector<cv::Point2f> >& corners, float markerLength,
                               const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
                               std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                               std::vector<float>& reprojectionError);
        bool EstimateArenaPosition(const std::vector<cv::Point2f>& corners, float baseLength,float weightLength,
                               std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs);
        ArenaLimits getRobotariumData();
    private:
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    pthread_t  _detectAruco; 
    pthread_cond_t listBlock;

    cv::VideoCapture in_video;
    cv::String videoInput = "0";//se selecciona la entrada de la camara
    int frame_width ;
	int frame_height ;

    float marker_length_m=0;
    float baseArenaLength=0;
    float weightArenaLength=0;
    double ang_roll = 0;
	double ang_pitch = 0;
	double ang_yaw = 0;

    cv::Mat grayMat;
    cv::Mat rot_mat;
    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
     cv::Mat binary_image;//for detect the arena 
     std::vector<cv::Vec3d> rvecs, tvecs;
    std::ostringstream vector_to_marker;

    int arenaSize=30;//cm default value
    ArenaLimits RobotariumData;
    std::vector<cv::Scalar> cornerColors = {
    cv::Scalar(0, 255, 0),  // Verde para la primera esquina
    cv::Scalar(255, 0, 0),  // Azul para la segunda esquina
    cv::Scalar(0, 0, 255),  // Rojo para la tercera esquina
    cv::Scalar(255, 255, 0) // Amarillo para la cuarta esquina (ajusta según sea necesario)
    };

    AgentCommunication *agentCommunication;
    ringBuffer *buffer;
    record_data data;
};
