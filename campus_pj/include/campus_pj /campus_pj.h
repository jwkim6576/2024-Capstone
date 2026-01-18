#ifndef CAMPUS_PJ_H
#define CAMPUS_PJ_H
#define PI 3.141592
#define DEGREE_TO_RADIAN(degree) ((PI/180.0)*(degree)) // 60분법 -> 호도법
#define RADIAN_TO_DEGREE(radian) ((180.0/PI)*(radian)) // 호도법 -> 60분법

#include <QWidget>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
// #include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv4/opencv2/core/eigen.hpp>

#include <opencv4/opencv2/aruco/charuco.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/aruco/dictionary.hpp>

#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/calib3d.hpp>

#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/GetCurrentPose.h>
#include <dsr_msgs/GetCurrentPosx.h>
#include <dsr_msgs/GetCurrentRotm.h>

#include <QFileDialog>
#include <QString>
#include <qtimer.h>
#include <QPixmap>
#include <QLabel>

#include <QMessageBox>

#include <cmath>
#include <random>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#include <vector>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/Vertices.h>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/project_inliers.h>

namespace {
    const char* about = "Create a ChArUco board image";
    const char* keys =
        "{@outfile |<none> | Output image }"
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in pixels) }"
        "{ml       |       | Marker side length (in pixels) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{m        |       | Margins size (in pixels). Default is (squareLength-markerLength) }"
        "{bb       | 1     | Number of bits in marker borders }"
        "{si       | false | show generated image }";
}

namespace Ui {
class campus_pj;
}

enum class FOV{
    LARGE,
    SMALL,
    SMALLCOR
};

class campus_pj : public QWidget
{
  Q_OBJECT

public:
  explicit campus_pj(QWidget *parent = 0);
  ~campus_pj();
  //ros::NodeHandlePtr n;
  ros::Subscriber color_image_sub_;
  ros::Subscriber depth_image_sub_;
  ros::Subscriber color_camera_info_sub_;
  ros::Subscriber pointcloud_sub_;
  

  void wait(float time_);

  int movel(float fTargetPos[6], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType);
  int movej(float fTargetPos[6], float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType);
  void get_markers_pos_and_orientation(float* startingPos, FOV fov, std::vector<int> &checking_mID, std::vector<int> &world_view_mID, std::vector<cv::Point3f>& output_point, std::vector<float> &outputPos, int current_markers, int cor);

  void calculateEnd2Base(float& x, float& y, float& z, float& r, float& p, float& yw);
  // void campus_pj::calculateEnd2Base2(float& x, float& y, float& z, float& r, float& p, float& yw, cv::Matx44f T_cam2b )
  void align_resolution(float* input2d);

  void color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
  void depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw);
  void color_camera_info_sub_cb(const sensor_msgs::CameraInfoConstPtr &depth_camera_info);
  void pointcloud_sub_cb(const sensor_msgs::PointCloud2ConstPtr &pointcloud_raw);
  pcl::PointCloud<pcl::PointXYZRGB> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg);
  pcl::PointCloud<pcl::PointXYZ> depth_to_pointcloud(cv::Mat _depth_image);

  std::vector<cv::Point2f> chessboard_corners;
  std::vector<std::vector<cv::Point2f>> corners, rejected;
  std::vector<std::vector<cv::Point2f>> current_corners; // new!
  std::vector<int> ids;

  float robot_current_pose[6];
  float robot_current_posx[6];

  cv::Mat color_image;
  cv::Mat color_image_raw;
  cv::Mat color_image_calibration;
  cv::Mat depth_image;
  cv::Mat depth_image_raw;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<cv::Mat> g2b_r;
  std::vector<cv::Mat> g2b_t;
  std::vector<cv::Mat> t2c_r;
  std::vector<cv::Mat> t2c_t;

  /* 이 밑은 그거 입니다. 
  그거는 임의의 좌표 A B C 에 대해서 A B C의 조인트 값을 받아서 기억했다가 반복하는 것입니다.
  A는 최초 설정한 값, B는 최초 동작 때 마커를 인식해서 얻은 값, C는 랜덤 생성한 값 이 친구는 그냥 좌표로 ㄱㄱ*/

  float Posx_A[6]; float Joint_A[6];
  float Posx_B[6]; float Joint_B[6];
  int repeat_counts;
  // float Posx_C[6]; // is it useless?


  /*void FIX_26082015_getBoardObjectAndImagePoints(const cv::Ptr<cv::aruco::Board> board,
                                            std::vector<cv::Point2f> _corners,
                                            std::vector<int>         _ids,
                                            std::vector<cv::Point3f> *objPoints,
                                            std::vector<cv::Point2f> *imgPoints);*/




private slots:
  void spinOnce();
  
  void on_pushButton_widget_process_home_clicked();

  void on_pushButton_calibration_clicked();

  void on_pushButton_start_process_clicked();

  void on_pushButton_start_processB_clicked();

  void on_pushButton_currentPosx_clicked();

  void on_pushButton_widget_processB_home_clicked();

  void on_pushButton_haneye_calibration_home_clicked();

  void on_pushButton_haneye_calibration_intrpara_clicked();

  void on_pushButton_haneye_calibration_disto_clicked();

  void on_pushButton_haneye_calibration_campara_clicked();

  void on_pushButton_haneye_calibration_showimage_clicked();

  void on_pushButton_haneye_calibration_findchess_clicked();

  void on_pushButton_haneye_calibration_getmatch_clicked();

  void on_pushButton_haneye_calibration_calculate_clicked();

  void on_pushButton_process_start_clicked();

  void on_pushButton_process_printPixels_clicked();

  void on_pushButton_processB_get_reps_clicked();

  void on_pushButton_processB_get_markers_pos_clicked();

  void on_pushButton_processB_get_markers_posj_clicked();

  void on_pushButton_processB_moveA2B_clicked();

  void on_pushButton_processB_moveA2C2B_clicked();

  void on_pushButton_processB_moveA2C2B_X_clicked();
  
  void on_pushButton_currentPosx_get_clicked();

  void on_pushButton_currentPosx_home_clicked();

private:

  Ui::campus_pj *ui;
  QTimer *ros_timer;

};

#endif //CAMPUS_PJ_H
