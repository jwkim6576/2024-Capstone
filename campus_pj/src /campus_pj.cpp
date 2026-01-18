#include "campus_pj.h"
#include "ui_campus_pj.h"


campus_pj::campus_pj(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::campus_pj)
{
  ui->setupUi(this);

  int argc = 0; char **argv = NULL;

  ros::init(argc, argv,"ros_pj");
  ros::NodeHandlePtr n;
  n.reset(new ros::NodeHandle("~"));

  ros_timer = new QTimer(this);
   connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
   ros_timer->start(1);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

  color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &campus_pj::color_image_sub_cb, this);
  depth_image_sub_ = n->subscribe("/camera/aligned_depth_to_color/image_raw", 1000, &campus_pj::depth_image_sub_cb, this);
  //color_image_sub_ = n->subscribe("/camera/color/image_raw", 1000, &campus_pj::color_image_sub_cb, this);
  color_camera_info_sub_ = n->subscribe("/camera/color/camera_info", 1000, &campus_pj::color_camera_info_sub_cb, this);
   ui->stackedWidget->setCurrentIndex(0);
  repeat_counts = 1;



   /*cv::FileStorage fs1(filename, cv::FileStorage::READ);
   std::cout << "reading R and T" << std::endl;
   fs1["R"] >> R;
   fs1["T"] >> T;
   std::cout << "R = " << R << "\n";
   std::cout << "T = " << T << std::endl;
   fs1.release();*/
}

// enum FOV{
//   LARGE,
//   SMALL,
//   SMALLCOR
// };
// template<typename OUTPUTPOINT>;
int color_info_count = 0;
double intrinsic_parameter[9];
double discoeffs[5];
int match_count = 0;
rs2_intrinsics RS_camera_info_;

int squaresX = 8;//인쇄한 보드의 가로방향 마커 갯수
int squaresY = 5;//인쇄한 보드의 세로방향 마커 갯수
float squareLength = 30;//검은색 테두리 포함한 정사각형의 한변 길이, mm단위로 입력
float markerLength_chess = 22;//인쇄물에서의 마커 한변의 길이, mm단위로 입력
float markerLength = 22;//100;//single인쇄물에서의 마커 한변의 길이, mm단위로 입력
int dictionaryId = 10;//DICT_6X6_250=10
//std::string outputFile = "output.txt";


 cv::Matx44f c2g ={ 0.9996215749858904, -0.01240737295999214, -0.02455125086343612, -25.14063285848638,
                    0.01112504270560043, 0.9986009416289027, -0.05169519129152499, -105.0359579400699,
                    0.02515830374898681, 0.05140249482369946, 0.9983610786075213, -12.21662738431984,
                    0, 0, 0, 1};








int calibrationFlags = 0;
float aspectRatio = 1;

cv::Mat A(3, 3, CV_64FC1, intrinsic_parameter);	// camera matrix
cv::Mat distCoeffs(5, 1, CV_64FC1, discoeffs);
//cv::Mat rvec, tvec;	// rotation & translation vectors
cv::Mat R;

cv::Mat c2g_rvec = (cv::Mat_<float>(3, 3));
cv::Mat  c2g_tvec = (cv::Mat_<float>(3, 1));

cv::Mat t2c_rvec = (cv::Mat_<float>(3, 3));
cv::Mat  t2c_tvec = (cv::Mat_<float>(3, 1));

cv::Mat g2b_rvec = (cv::Mat_<float>(3, 3));
cv::Mat g2b_tvec = (cv::Mat_<float>(3, 1));

campus_pj::~campus_pj()
{
  ROS_INFO("Campus ampus SHUTDOWN!");
  delete ui;
}

void campus_pj::spinOnce()
{
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}

void campus_pj::wait(float time_)
{
    double time_lf = static_cast<double>(time_);

    auto start_time = std::chrono::steady_clock::now();

    std::chrono::duration<double> loop_duration(time_lf);

    while(true){        
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time >= loop_duration) {
            break; 
        }
        spinOnce();
    }
}

int campus_pj::movej(float *fTargetPos, float fTargetVel, float fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveMode, int nBlendingType, int nSyncType)
{
  ui->textEdit_log->append("Move_joint START!");
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
      ros::ServiceClient srvMoveJoint = node->serviceClient<dsr_msgs::MoveJoint>( "/dsr01m1013/motion/move_joint");



      dsr_msgs::MoveJoint srv;

      for(int i=0; i<6; i++)
          srv.request.pos[i] = fTargetPos[i];
      srv.request.vel = fTargetVel;
      srv.request.acc = fTargetAcc;
      srv.request.time = fTargetTime;
      srv.request.radius = fBlendingRadius;
      srv.request.mode = nMoveMode;
      srv.request.blendType = nBlendingType;
      srv.request.syncType = nSyncType;


      QString text_for_append;

      text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
      ui->textEdit_log->append(text_for_append);

      text_for_append.sprintf("  <vel> %7.3f , <acc> %7.3f, <time> %7.3f",srv.request.vel, srv.request.acc, srv.request.time);
      ui->textEdit_log->append(text_for_append);

      text_for_append.sprintf("  <mode> %d , <radius> %7.3f, <blendType> %d",srv.request.mode, srv.request.radius, srv.request.blendType);
      ui->textEdit_log->append(text_for_append);

      if(srvMoveJoint.call(srv))
      {
         text_for_append.sprintf("  receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
          ui->textEdit_log->append(text_for_append);
          return (srv.response.success);
      }
      else
      {        
           ui->textEdit_log->append("  Failed to call service dr_control_service : move_joint");
          ros::shutdown();
          return -1;
      }

      return 0;

}

int campus_pj::movel(float *fTargetPos, float *fTargetVel, float *fTargetAcc, float fTargetTime, float fBlendingRadius, int nMoveReference, int nMoveMode, int nBlendingType, int nSyncType)
{
      ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
      ros::ServiceClient srvMoveLine = node->serviceClient<dsr_msgs::MoveLine>( "/dsr01m1013/motion/move_line");
      dsr_msgs::MoveLine srv;

      for(int i=0; i<6; i++)
          srv.request.pos[i] = fTargetPos[i];
      for(int i=0; i<2; i++){
          srv.request.vel[i] = fTargetVel[i];
          srv.request.acc[i] = fTargetAcc[i];
      }
      srv.request.time = fTargetTime;
      srv.request.radius = fBlendingRadius;
      srv.request.ref  = nMoveReference;
      srv.request.mode = nMoveMode;
      srv.request.blendType = nBlendingType;
      srv.request.syncType = nSyncType;


       QString text_for_append;

       text_for_append.sprintf("  <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",srv.request.pos[0],srv.request.pos[1],srv.request.pos[2],srv.request.pos[3],srv.request.pos[4],srv.request.pos[5]);
       ui->textEdit_log->append(text_for_append);
      text_for_append.sprintf("  <vel> %7.3f,%7.3f <acc> %7.3f,%7.3f <time> %7.3f",srv.request.vel[0],srv.request.vel[1],srv.request.acc[0],srv.request.acc[1], srv.request.time);
      ui->textEdit_log->append(text_for_append);
      text_for_append.sprintf("  <mode> %d, <ref> %d, <radius> %7.3f, <blendType> %d",srv.request.mode,srv.request.ref, srv.request.radius, srv.request.blendType);
      ui->textEdit_log->append(text_for_append);

      if(srvMoveLine.call(srv))
      {
          text_for_append.sprintf("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
          ui->textEdit_log->append(text_for_append);
          return (srv.response.success);
      }
      else
      {
          text_for_append.sprintf("Failed to call service dr_control_service : move_line\n");
          ui->textEdit_log->append(text_for_append);
          ros::shutdown();
          return -1;
      }

      return 0;
}

void campus_pj::color_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
  color_image_raw = cv_ptr->image.clone();
  color_image = cv_ptr->image.clone();
}

void campus_pj::depth_image_sub_cb(const sensor_msgs::Image::ConstPtr &image_raw)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::TYPE_16UC1);
  cv_ptr->image.convertTo(depth_image, CV_32F, 0.001);

}

void campus_pj::color_camera_info_sub_cb(const sensor_msgs::CameraInfoConstPtr &depth_camera_info)
{
    if(color_info_count >0) return;

    for(int i=0;i<9;i++)
    {
      intrinsic_parameter[i] = depth_camera_info->K[i];
      //std::cout << intrinsic_parameter[i] << std::endl;
    }

    RS_camera_info_.width = depth_camera_info->width;    // Image Resoltusion width
    RS_camera_info_.height = depth_camera_info->height;  // Image Resoltusion height
    RS_camera_info_.fx = depth_camera_info->K[0];        // 초점거리 x
    RS_camera_info_.fy = depth_camera_info->K[4];        // 초점거리 y
    RS_camera_info_.ppx = depth_camera_info->K[2];       // 주점 x
    RS_camera_info_.ppy = depth_camera_info->K[5];       // 주점 y
    RS_camera_info_.model = RS2_DISTORTION_MODIFIED_BROWN_CONRADY;

    for(int i=0;i<5;i++)
    {
      discoeffs[i]= depth_camera_info->D[i];
      RS_camera_info_.coeffs[i] = depth_camera_info->D[i];
      //std::cout << RS_camera_info_.coeffs[i] << std::endl;
    }

    color_info_count++;
}

pcl::PointCloud<pcl::PointXYZ> campus_pj::depth_to_pointcloud(cv::Mat _depth_image)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int width = _depth_image.cols;
    int height = _depth_image.rows;
    cloud.clear();
    cloud.is_dense = false;

    // Get the camera intrinsics
    double fx = intrinsic_parameter[0];  // 초점거리
    double fy = intrinsic_parameter[4];
    double cx = intrinsic_parameter[2];  // 주점
    double cy = intrinsic_parameter[5];

    // K = [fx 0 cx;
    //      0 fy cy;
    //      0  0  1]

    for (int v = 0; v < height; v++)
    {
      for (int u = 0; u < width; u++)
      {
        // https://github.com/IntelRealSense/librealsense/blob/5e73f7bb906a3cbec8ae43e888f182cc56c18692/include/librealsense2/rsutil.h#L46
        // get a data of an element from depth image

        // (u,v): 정규좌표계 (카메라 내부 파라미터의 영향을 제거한 이미지 좌표계)
        // 
        uint16_t depth = _depth_image.at<uint16_t>(v, u);
        // Skip over pixels with a depth value of zero, which is used to indicate no data
        if(depth==0) continue;

        float x = (u - cx) / fx;
        float y = (v - cy) / fy;
 
        // // Apply distortion
        float r2 = x * x + y * y;
        float f = 1 + discoeffs[0] * r2 + discoeffs[1] * r2 * r2 + discoeffs[4] * r2 * r2 * r2;
        float ux = x * f + 2 * discoeffs[2] * x * y + discoeffs[3] * (r2 + 2 * x * x);
        float uy = y * f + 2 * discoeffs[3] * x * y + discoeffs[2] * (r2 + 2 * y * y);
        x = ux;
        y = uy;

        pcl::PointXYZ point;
        point.x = float(depth * x / 1000.0);
        point.y = float(depth * y / 1000.0);
        point.z = float(depth / 1000.0);

        
        // 22, 70, 424

        cloud.push_back(point);
        // if (v%100 == 0 & u%100 == 0){ 
        //   ROS_INFO("%f", depth);
        //   std::cout<<"u : "<<u<<" | v : "<<v<<" | x : "<<point.x<<" | y : "<<point.y<<" | z : "<<point.z<<""<<std::endl;
        // }
      }
    }

    return cloud;
  }



void campus_pj::calculateEnd2Base(float& x, float& y, float& z, float& r, float& p, float& yw){
    // 초기 설정
    float r_rad = r * M_PI / 180.0;
    float p_rad = p * M_PI / 180.0;
    float yw_rad = yw * M_PI / 180.0;   
 
 

    // cv::Matx44d camera2end( 0.997384999638619, -0.0719288009035676, -0.007029231568742221, -27.02221594881259,
    //                         0.07161484260894792, 0.9967185601557664, -0.03772832038550635, -100.8828287678684,
    //                         0.009719918413633304, 0.03712626350160663, 0.9992633105165232, -11.98360880536063,
    //                         0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00

    // );
    

    // base to camera의 변환 행렬 계산
    cv::Vec3f translation(x, y, z);

    cv::Matx33f Rz1(
        std::cos(r_rad), -std::sin(r_rad), 0,
        std::sin(r_rad), std::cos(r_rad), 0,
        0, 0, 1
    );
    cv::Matx33f Ry(
        std::cos(p_rad), 0, std::sin(p_rad),
        0, 1, 0,
        -std::sin(p_rad), 0, std::cos(p_rad)
    );
    cv::Matx33f Rz2(
        std::cos(yw_rad), -std::sin(yw_rad), 0,
        std::sin(yw_rad), std::cos(yw_rad), 0,
        0, 0, 1
    );

    cv::Matx33f R_cam2b = Rz1 * Ry * Rz2;
    cv::Matx44f T_cam2b = cv::Matx44f::eye();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T_cam2b(i, j) = R_cam2b(i, j);
        }
        T_cam2b(i, 3) = translation(i);
    }

    // end effector to camera의 역변환 행렬
    // cv::Matx44d T_end2camera = camera2end.inv();
    cv::Matx44f T_end2camera = c2g.inv();

    // end effector의 위치 추출
    cv::Matx44f T_end2base = T_cam2b * T_end2camera;
    x = T_end2base(0, 3);
    y = T_end2base(1, 3);
    z = T_end2base(2, 3);

    // end effector의 방향 추출
    cv::Matx33f R_end2base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base(i, j) = T_end2base(i, j);
        }
    }
    
    p_rad = std::acos(R_end2base(2, 2));
    if (std::sin(p_rad) == 0.0)
    {
      r_rad = 0; yw_rad = std::acos(R_end2base(0,0)) - r_rad;
    }
    else
    {
      if (R_end2base(1, 2) / std::sin(p_rad) > 0) {
          r_rad = std::acos(R_end2base(0, 2) / std::sin(p_rad));
      } else {
          r_rad = -std::acos(R_end2base(0, 2) / std::sin(p_rad));
      }
      if (R_end2base(2, 1) / std::sin(p_rad) > 0) {
          yw_rad = std::acos(-R_end2base(2, 0) / std::sin(p_rad));
      } else {
          yw_rad = -std::acos(-R_end2base(2, 0) / std::sin(p_rad));
      }
      }


    r = r_rad * 180.0 / M_PI;
    p = p_rad * 180.0 / M_PI;
    yw = yw_rad * 180.0 / M_PI;
}

void campus_pj::align_resolution(float* input2d)
{
  cv::Mat color_clone = color_image_raw.clone();
  cv::Mat depth_clone = depth_image.clone();
  // cout << "color w: " << color_clone.cols << " h: " << color_clone.rows << endl;
  // cout << "depth w: " << depth_clone.cols << " h: " << depth_clone.rows << endl;
  if(color_clone.cols != depth_clone.cols  || color_clone.rows != depth_clone.rows)
  {
    input2d[0] = input2d[0] * depth_clone.cols / color_clone.cols;
    input2d[1] = input2d[1] * depth_clone.rows / color_clone.rows;
  }

}

///////////////////////////////////////////HOME////////////////////////////////////////////

void campus_pj::on_pushButton_widget_process_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  // mb.move(470, 350);

  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}

void campus_pj::on_pushButton_calibration_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void campus_pj::on_pushButton_start_process_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
    // cv::Mat showimage;
    // showimage = color_image.clone();
    // cv::resize(showimage, showimage, cv::Size(640, 360));
    // ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}

void campus_pj::on_pushButton_start_processB_clicked()
{
    ui->stackedWidget->setCurrentIndex(4);
}

void campus_pj::on_pushButton_widget_processB_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  // mb.move(470, 350);

  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}

///////////////////////////////////////////CALIBRATION////////////////////////////////////////////////////

void campus_pj::on_pushButton_haneye_calibration_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  // mb.move(470, 350);
  
  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}

void campus_pj::on_pushButton_haneye_calibration_intrpara_clicked()
{
  double ffx, ffy, ccx, ccy;

  QString fxText = ui->textEdit_get_fx->toPlainText();
  QString fyText = ui->textEdit_get_fy->toPlainText();
  QString cxText = ui->textEdit_get_cx->toPlainText();
  QString cyText = ui->textEdit_get_cy->toPlainText();

  ffx = !fxText.isEmpty() ? fxText.toDouble() : intrinsic_parameter[0];
  ffy = !fyText.isEmpty() ? fyText.toDouble() : intrinsic_parameter[4];
  ccx = !cxText.isEmpty() ? cxText.toDouble() : intrinsic_parameter[2];
  ccy = !cyText.isEmpty() ? cyText.toDouble() : intrinsic_parameter[5];


  intrinsic_parameter[0] = ffx; intrinsic_parameter[4] = ffy; // 초점거리 x y
  intrinsic_parameter[2] = ccx; intrinsic_parameter[5] = ccy; // 주점 x y

  QString text_for_append01;

  text_for_append01.sprintf(" fx = %.5lf, fy = %.5lf, cx = %.5lf, cy = %.5lf) ",intrinsic_parameter[0] , intrinsic_parameter[4], intrinsic_parameter[2], intrinsic_parameter[5]);
  ui->textEdit_haneye_calibration_log->append(text_for_append01);

}

void campus_pj::on_pushButton_haneye_calibration_disto_clicked()
{
  double kk1, kk2, kk3, tt1, tt2;

  QString k1Text = ui->textEdit_get_k1->toPlainText();
  QString k2Text = ui->textEdit_get_k2->toPlainText();
  QString k3Text = ui->textEdit_get_k3->toPlainText();
  QString t1Text = ui->textEdit_get_t1->toPlainText();
  QString t2Text = ui->textEdit_get_t2->toPlainText();

  kk1 = !k1Text.isEmpty() ? k1Text.toDouble() : discoeffs[0];
  kk2 = !k2Text.isEmpty() ? k2Text.toDouble() : discoeffs[1];
  kk3 = !k3Text.isEmpty() ? k3Text.toDouble() : discoeffs[4];
  tt1 = !t1Text.isEmpty() ? t1Text.toDouble() : discoeffs[2];
  tt2 = !t2Text.isEmpty() ? t2Text.toDouble() : discoeffs[3];

  discoeffs[0] = kk1; discoeffs[1] = kk2; discoeffs[4] = kk3;
  discoeffs[2] = tt1; discoeffs[3] = tt2;

  QString text_for_append02;

  text_for_append02.sprintf(" k1 = %.5lf, k2 = %.5lf, t1 = %.5lf, t2 = %.5lf, k3 = %.5lf) ", discoeffs[0], discoeffs[1], discoeffs[2], discoeffs[3], discoeffs[4]);
  ui->textEdit_haneye_calibration_log->append(text_for_append02);

}

void campus_pj::on_pushButton_haneye_calibration_campara_clicked()
{
  color_info_count = 0;
  QString text_for_append03;

  text_for_append03.sprintf(" fx = %.5lf, fy = %.5lf, cx = %.5lf, cy = %.5lf) ",intrinsic_parameter[0] , intrinsic_parameter[4], intrinsic_parameter[2], intrinsic_parameter[5]);
  ui->textEdit_haneye_calibration_log->append(text_for_append03);

  QString text_for_append04;

  text_for_append04.sprintf(" k1 = %.5lf, k2 = %.5lf, t1 = %.5lf, t2 = %.5lf, k3 = %.5lf) ", discoeffs[0], discoeffs[1], discoeffs[2], discoeffs[3], discoeffs[4]);
  ui->textEdit_haneye_calibration_log->append(text_for_append04);
}

void campus_pj::on_pushButton_haneye_calibration_showimage_clicked()
{
  cv::Mat showimage;

  showimage = color_image.clone();
  cv::resize(showimage, showimage, cv::Size(640, 480));
  ui->label_handeye_pic->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
}

void campus_pj::on_pushButton_haneye_calibration_findchess_clicked() //charuco
{
  cv::Mat image_findcorners = color_image_raw.clone();

  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength_chess, dictionary);


  cv::aruco::detectMarkers(image_findcorners, board->dictionary, corners, ids, detectorParams, rejected);
  // for(int i = 0; i < corners.size(); i++) cout << "Corners x : " << corners[i][0].x << " y : " << corners[i][0].y << endl;
  if (ids.size() > 0)
  {
    cv::aruco::drawDetectedMarkers(image_findcorners, corners);
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(corners, ids, image_findcorners, board, charucoCorners, charucoIds);
    // if at least one charuco corner detected
    if (charucoIds.size() > 0)
    {
      cv::aruco::drawDetectedCornersCharuco(image_findcorners, charucoCorners, charucoIds, cv::Scalar(255, 255, 0));
      bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, A, distCoeffs, t2c_rvec, t2c_tvec);

      // cout << "charucoCorners x : " << charucoCorners[0].x << " y : " << charucoCorners[0].y << endl;

      if (valid) cv::drawFrameAxes(image_findcorners, A, distCoeffs, t2c_rvec, t2c_tvec, 100);

      std::vector<cv::Point3f> axesPoints;
      axesPoints.push_back(cv::Point3f(0, 0, 0));
      axesPoints.push_back(cv::Point3f(100, 0, 0));
      axesPoints.push_back(cv::Point3f(0,100, 0));
      axesPoints.push_back(cv::Point3f(0, 0, 100));
      std::vector<cv::Point2f> imagePoints;
      cv::projectPoints(axesPoints, t2c_rvec, t2c_tvec, A, distCoeffs, imagePoints);

      float distance = depth_image.at<float>(imagePoints[0].y, imagePoints[0].x)*1000;

      std::cout << "distance = " << distance << std::endl;



      cv::Point2f center;

      center.x = imagePoints[0].x;
      center.y = imagePoints[0].y;

      cv::putText(image_findcorners, cv::format("(%f, %f)",imagePoints[0].x, imagePoints[0].y), center, cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
      float op_point[3];
      float pixel[2];

      pixel[0] = imagePoints[0].x;
       pixel[1] = imagePoints[0].y;
      rs2_deproject_pixel_to_point(op_point, &RS_camera_info_, pixel, distance);

      std::cout << " origin = " << t2c_tvec << std::endl;
      cv::Mat RR;
      cv::Rodrigues(t2c_rvec,RR);
      t2c_tvec = -RR.inv() * t2c_tvec;

      t2c_tvec.at<float>(0,0) = op_point[0];
      t2c_tvec.at<float>(1,0) = op_point[1];
      t2c_tvec.at<float>(2,0) = op_point[2];

      //cv::Rodrigues(rvec, R);

    }
  }
  cv::resize(image_findcorners, image_findcorners, cv::Size(640, 320));
  ui->label_handeye_pic->setPixmap(QPixmap::fromImage(QImage(image_findcorners.data, image_findcorners.cols, image_findcorners.rows, image_findcorners.step, QImage::Format_RGB888)));
}

void campus_pj::on_pushButton_haneye_calibration_getmatch_clicked()
{

  cv::Mat rvec;
  cv::Rodrigues(t2c_rvec.clone(), rvec);

  t2c_r.push_back(rvec.clone());
  t2c_t.push_back(t2c_tvec.clone());

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;
  srv.request.space_type = 1;

  QString text_for_append;

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
      ui->textEdit_haneye_calibration_log->append(text_for_append.sprintf(
      " <pos> %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f",robot_current_pose[0],robot_current_pose[1],robot_current_pose[2],robot_current_pose[3],robot_current_pose[4],robot_current_pose[5]));

      //return (srv.response.success);
  }
  else
  {
      ui->textEdit_haneye_calibration_log->append("fail!");
      ros::shutdown();
     // return -1;
  }

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  float data[9];
  int k = 0;

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[k] = srv2.response.rot_matrix[i].data[j] ;
         k++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  cv::Mat rotation_mat(3,3, CV_32FC1, data);

  std::cout << rotation_mat << std::endl;
  //rotation_mat = rotation_mat.inv();*/

  float data_trans[] = {robot_current_pose[0],robot_current_pose[1],robot_current_pose[2]};

  g2b_tvec.at<float>(0,0) = robot_current_pose[0];
  g2b_tvec.at<float>(1,0) = robot_current_pose[1];
  g2b_tvec.at<float>(2,0) = robot_current_pose[2];

  //cv::Mat trans_mat(3,1,CV_32FC1,data_trans);
  //std::cout << "g2b_t_raw(compare)" << std::endl << trans_mat<< std::endl;


  //trans_mat = -rotation_mat.inv() * -rotation_mat * trans_mat;
  //rotation_mat = rotation_mat.inv();
 //trans_mat = -rotation_mat.inv() * trans_mat;



  //rotation_mat.copyTo(g2b_r);
  //trans_mat.copyTo(g2b_t);
  cv::Mat g2b_rvec = rotation_mat.clone();
  //cv::Rodrigues(rotation_mat, g2b_rvec);

  /*g2b_rvec.at<float>(0,0) = floorf(g2b_rvec.at<float>(0,0)*100) / 100;
  g2b_rvec.at<float>(0,1) = floorf(g2b_rvec.at<float>(0,1)*100) / 100;
  g2b_rvec.at<float>(0,2) = floorf(g2b_rvec.at<float>(0,2)*100) / 100;

  g2b_tvec.at<float>(0,0) = floorf(g2b_tvec.at<float>(0,0)*100) / 100;
  g2b_tvec.at<float>(0,1) = floorf(g2b_tvec.at<float>(0,1)*100) / 100;
  g2b_tvec.at<float>(0,2) = floorf(g2b_tvec.at<float>(0,2)*100) / 100;*/

  g2b_r.push_back(g2b_rvec.clone());
  g2b_t.push_back(g2b_tvec.clone());



  cv::Mat test = g2b_tvec.clone();
  //g2b_tvec.at<float>(0,0) = -g2b_tvec.at<float>(0,0) ;

  g2b_tvec = - g2b_rvec * g2b_tvec;
  g2b_tvec = g2b_rvec.inv() * g2b_tvec + test;
  std::cout << "g2b_test = " << std::endl << g2b_tvec << std::endl;





  std::cout << "t2c_r" << std::endl << t2c_r[t2c_r.size()-1]<< std::endl;
  std::cout << "t2c_t" << std::endl << t2c_t[t2c_t.size()-1]<< std::endl;


  std::cout << "g2b_r" << std::endl << g2b_r[g2b_r.size()-1]<< std::endl;
  std::cout << "g2b_t" << std::endl << g2b_t[g2b_t.size()-1]<< std::endl;


  ui->listWidget_haneye_calibration_t2c->addItem(text_for_append.sprintf("Match %d ",match_count));
  match_count++;

  /*cv::calibrateHandEye(
        g2b_r,
        g2b_t,
        t2c_r,
        t2c_t,
        c2g_rvec,
        c2g_tvec,
        cv::CALIB_HAND_EYE_TSAI);
  std::cout << "RESULT!!!" << std::endl;
  std::cout << "c2g_rvec : " << c2g_rvec << ", c2g_tvec : "  << c2g_tvec << std::endl;*/

}

void campus_pj::on_pushButton_haneye_calibration_calculate_clicked()
{

  for(int i=0; i<g2b_r.size() ; i++)
  {
    std::cout << i+1 << " Match ===== " << std::endl;
    std::cout << "t2c_r" << std::endl <<t2c_r[i] << std::endl;
    std::cout << "t2c_t" << std::endl <<t2c_t[i] << std::endl;
    std::cout << "g2b_r" << std::endl <<g2b_r[i] << std::endl;
    std::cout << "g2b_t" << std::endl <<g2b_t[i] << std::endl;
  }
  cv::calibrateHandEye(
          g2b_r,
          g2b_t,
          t2c_r,
          t2c_t,
          c2g_rvec,
          c2g_tvec,cv::CALIB_HAND_EYE_DANIILIDIS);


    std::cout << "===========================" << std::endl;
    std::cout << "RESULT!!!_daniilids" << std::endl;
    std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;

     std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

     std::string filename = "src/campus_pj/config/test1.yaml";
     cv::FileStorage fs(filename, cv::FileStorage::WRITE);
     fs << "R_daniilids" << c2g_rvec;
     fs << "T_daniilids" << c2g_tvec;

     std::string filename1 = "src/campus_pj/config/test2.yaml";
     cv::FileStorage fs1(filename1, cv::FileStorage::WRITE);
     fs1 << "t2c_r" << t2c_r;
     fs1 << "t2c_t" << t2c_t;
     fs1 << "g2b_r" << g2b_r;
     fs1 << "g2b_t" << g2b_t;
     fs1.release();



    cv::calibrateHandEye(
            g2b_r,
            g2b_t,
            t2c_r,
            t2c_t,
            c2g_rvec,
            c2g_tvec,cv::CALIB_HAND_EYE_PARK);


      std::cout << "===========================" << std::endl;
      std::cout << "RESULT!!!_park" << std::endl;
      std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;

      fs << "R_park" << c2g_rvec;
      fs << "T_park" << c2g_tvec;


      std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

      cv::calibrateHandEye(
              g2b_r,
              g2b_t,
              t2c_r,
              t2c_t,
              c2g_rvec,
              c2g_tvec,cv::CALIB_HAND_EYE_TSAI);


        std::cout << "===========================" << std::endl;
        std::cout << "RESULT!!!_TSAI" << std::endl;
        std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
        fs << "R_tsai" << c2g_rvec;
        fs << "T_tsai" << c2g_tvec;


        std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

        cv::calibrateHandEye(
                g2b_r,
                g2b_t,
                t2c_r,
                t2c_t,
                c2g_rvec,
                c2g_tvec,cv::CALIB_HAND_EYE_HORAUD);


          std::cout << "===========================" << std::endl;
          std::cout << "RESULT!!!_horaud" << std::endl;
          std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
          fs << "R_horaud" << c2g_rvec;
          fs << "T_horaud" << c2g_tvec;
          std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

          cv::calibrateHandEye(
                  g2b_r,
                  g2b_t,
                  t2c_r,
                  t2c_t,
                  c2g_rvec,
                  c2g_tvec,cv::CALIB_HAND_EYE_ANDREFF);


            std::cout << "===========================" << std::endl;
            std::cout << "RESULT!!!_andreff" << std::endl;
            std::cout << "c2g_rvec : " << std::endl << c2g_rvec << std::endl << ", c2g_tvec : "  <<std::endl<< c2g_tvec << std::endl;
            fs << "R_andreff" << c2g_rvec;
            fs << "T_andreff" << c2g_tvec;

                 fs.release();
            std::cout << "c2g_rvec_inv * t : " << std::endl << c2g_rvec.inv()*c2g_tvec <<std::endl;

}

void campus_pj::on_pushButton_currentPosx_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
}

void campus_pj::on_pushButton_currentPosx_get_clicked()
{
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetposx = node->serviceClient<dsr_msgs::GetCurrentPosx>("/dsr01m1013/aux_control/get_current_posx");
  dsr_msgs::GetCurrentPosx srv;
  srv.request.ref = 0;

  cv::Matx44d c2t(1.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 1.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                    0.00000000e+00, 0.00000000e+00, 1.00000000e+00, -1.000000e+02,
                    0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
  QString text_for_append;

  if(srvGetposx.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_posx[i] = srv.response.task_pos_info[0].data[i];
      }
      float r_rad = robot_current_posx[3] * M_PI / 180.0;
      float p_rad = robot_current_posx[4] * M_PI / 180.0;
      float yw_rad =robot_current_posx[5] * M_PI / 180.0;   


      // base to camera의 변환 행렬 계산
      cv::Vec3d translation(robot_current_posx[0], robot_current_posx[1], robot_current_posx[2]);

      cv::Matx33d Rz1(
          std::cos(r_rad), -std::sin(r_rad), 0,
          std::sin(r_rad), std::cos(r_rad), 0,
          0, 0, 1
      );
      cv::Matx33d Ry(
          std::cos(p_rad), 0, std::sin(p_rad),
          0, 1, 0,
          -std::sin(p_rad), 0, std::cos(p_rad)
      );
      cv::Matx33d Rz2(
          std::cos(yw_rad), -std::sin(yw_rad), 0,
          std::sin(yw_rad), std::cos(yw_rad), 0,
          0, 0, 1
      );

      cv::Matx33d R_t2b = Rz1 * Ry * Rz2;
      cv::Matx44d T_t2b = cv::Matx44d::eye();
      for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
              T_t2b(i, j) = R_t2b(i, j);
          }
          T_t2b(i, 3) = translation(i);
      }

      // end effector to camera의 역변환 행렬
      cv::Matx44d T_end2tool = c2t.inv();

      // end effector의 위치 추출
      cv::Matx44d T_end2base = T_t2b * T_end2tool;
      float xt = T_end2base(0, 3);
      float yt = T_end2base(1, 3);
      float zt = T_end2base(2, 3);

      ui->textEdit_currentPosx_log->append(text_for_append.sprintf(
      " <pos> %7.5f %7.5f %7.5f ",xt,yt,zt));

      //return (srv.response.success);
  }
  else
  {
      ui->textEdit_currentPosx_log->append("fail!");
      ros::shutdown();
     // return -1;
  }

}

void campus_pj::on_pushButton_currentPosx_home_clicked()
{
  QMessageBox mb;

  mb.setText("Are you sure you want to return to home?");
  mb.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
  mb.setDefaultButton(QMessageBox::Cancel);
  mb.setIcon(QMessageBox::Icon::Warning);

  // mb.move(470, 350);
  
  int ret = mb.exec();

  switch(ret)
  {
  case QMessageBox::Ok :
    ui->stackedWidget->setCurrentIndex(0);
    break;

  case QMessageBox::Cancel:
    break;
  }
}

// startingPos: OUTPUTPOINT
void campus_pj::get_markers_pos_and_orientation(float* startingPos, FOV fov, std::vector<int> &checking_mID, std::vector<int> &world_view_mID, std::vector<cv::Point3f>& output_point, std::vector<float> &outputPos, int current_markers = 0, int cor = 0)
{
  // initializing
  float velx[2] = {0,0};
  float accx[2] = {0,0};

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;

  srv.request.space_type = 1; // 0이면 joint 1이면 position

  ros::NodeHandlePtr node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  movel(startingPos,velx,accx,4.5,0,0,0,0,0); // move to starting position
  wait(1.0);
  cv::waitKey(1);

  cv::Mat image_aruco = color_image_raw.clone();

  std::vector<int> markerIds;
  std::vector<cv::Vec3d> rvecs, tvecs;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
  // 마커와 관련된 것
  std::vector<std::vector<cv::Point3f>> arucomarkers_camera_3d;
  std::vector<cv::Point3f> arucomarkers_world_3d;
  cv::Point3f arucomarker_world_3d;
  std::vector<cv::Point3f> arucomarker_3d(4);
  int current_marker_num = 0;
  float distance[4];
  float aruco_2d[4][2], aruco_3d[4][3];
  bool foundDifference = false;
  float x, y, z, r, p, w;
  float p_rad, r_rad, yw_rad;
  // float output_pos[6];
  std::vector<float> output_pos(6, 0.0f);
  // 
  std::vector<cv::Matx41f> aruco_cal_in(4);
  std::vector<cv::Matx41f> aruco_cal_out(4);

  // starting detecting markers

  cv::Mat image_Bit, image_Gray, image_brightened;
  cv::cvtColor(image_aruco, image_Gray, cv::COLOR_BGR2GRAY);
  cv::adaptiveThreshold(image_Gray, image_Bit, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 131, 15); // 이미지 이진화 

  double bright_fac = 1.4;
  image_brightened = image_aruco * bright_fac;


  parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX; // corner 정밀화 방법 추가

  char fnm[255];

  switch (fov) {
    case FOV::LARGE:
        cv::imwrite("/home/bkyb/Images/image_brightened.jpg", image_brightened);
        break;
    case FOV::SMALL:
        sprintf(fnm, "/home/bkyb/Images/image_brightened%02d_small.jpg",current_markers); 
        cv::imwrite(fnm, image_brightened); 
        break;
    case FOV::SMALLCOR:
        sprintf(fnm, "/home/bkyb/Images/image_brightened%02d_%02d_corrected.jpg",current_markers, cor);
        cv::imwrite(fnm, image_brightened); 
        break;
    default:
        break;
  }

  cv::aruco::detectMarkers(image_brightened, dictionary, markerCorners, markerIds, parameters, rejectedCandidates); // 이미지에서 마커 인식

  if(fov == FOV::LARGE) world_view_mID.assign(markerIds.begin(), markerIds.end()); //for(int i = 0; i <sizeof(markerIds); i++) world_view_mID[i] = markerIds[i]; // 
  else 
  {  
    for(int j = 0; j < sizeof(markerIds); j++)
    {
      for(int k = 0; k < sizeof(checking_mID); k++)
      {
        if (markerIds[j] == checking_mID[k]) {foundDifference = false; break;}
        if (k == 4 && markerIds[j] != checking_mID[k] && world_view_mID[current_markers] == markerIds[j]) 
        {
          foundDifference = true;
          checking_mID[current_markers] = markerIds[j];
          current_marker_num = j; 
        }        
      }
      if(foundDifference) break;
    }
  }

  cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, A, distCoeffs, rvecs, tvecs); // 마커 자세추정

  cv::Mat outputImage = color_image_raw.clone();
  // cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
  }

  float data[9];
  int l = 0;

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
      for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
      {
        for(int j=0 ; j<3 ; j++)
        {
          data[l] = srv2.response.rot_matrix[i].data[j] ;
          l++;
        }
      }
  }
  else
  {
      ros::shutdown();
      // return -1;
  }
  l = 0;
  
  cv::Matx44f g2b = {
    data[0], data[1], data[2], robot_current_pose[0],
    data[3], data[4], data[5], robot_current_pose[1],
    data[6], data[7], data[8], robot_current_pose[2],
    0, 0, 0, 1};  
  
  /// cout << "g2b : " <<data[0]<< ", "<<data[1] << ", "<<data[2]<<", "<<robot_current_pose[0]<< endl;
  /// cout << "      " <<data[3]<< ", "<<data[4] << ", "<<data[5]<<", "<<robot_current_pose[1]<< endl;
  /// cout << "      " <<data[6]<< ", "<<data[7] << ", "<<data[8]<<", "<<robot_current_pose[2]<< endl;

  if(fov == FOV::LARGE) for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);
  else for(int i = 0; i < 4; i++) cv::circle(outputImage, markerCorners[current_marker_num][i], 2, cv::Scalar(0, 255, 0), 2); // 수정

  // for(int j = 0 ; j < markerCorners[current_marker_num].size(); j++) cout << "x : " << markerCorners[current_marker_num][j].x << " y : "<< markerCorners[current_marker_num][j].y << endl;

  int startNum, endNum;

  if(fov == FOV::LARGE) {endNum = markerCorners.size(); startNum = 0;}
  else {endNum = current_marker_num + 1; startNum = current_marker_num; cout << "markers number : " << markerIds[current_marker_num] << endl; } // add print current markers num

  // camera 3d
  cout << "camera z offset is : ";
  for(int i = startNum ; i < endNum ; i++)
  {
    for(int j = 0; j < 4; j++) 
    {
      distance[j] = depth_image.at<float>(markerCorners[i][j].y, markerCorners[i][j].x)*1000;
      aruco_2d[j][0] = markerCorners[i][j].x;
      aruco_2d[j][1] = markerCorners[i][j].y;      
      rs2_deproject_pixel_to_point(aruco_3d[j], &RS_camera_info_, aruco_2d[j], distance[j]);
      // cout << "x : " << markerCorners[i][j].x << " y : "<< markerCorners[i][j].y << endl; //
      arucomarker_3d[j].x = aruco_3d[j][0];
      arucomarker_3d[j].y = aruco_3d[j][1];
      arucomarker_3d[j].z = aruco_3d[j][2];
    }
    // // for(int j = 0; j<4; j++) cout << "x : "<< arucomarker_3d[j].x << " y : "<< arucomarker_3d[j].y<<" z : "<<arucomarker_3d[j].z <<endl;
    
    for(int j = 0; j<4; j++) cout <<arucomarker_3d[j].z << " ";
    // cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    arucomarkers_camera_3d.push_back(arucomarker_3d);
  } 
  cout << endl;
  int index = 0;
  char buf[255];

  // world 3d
  for(int i = 0 ; i < arucomarkers_camera_3d.size() ; i++)
  {
    int wantedMarker;

    if(fov == FOV::LARGE) wantedMarker = i;//
    else wantedMarker = current_marker_num;

    for(int j = 0 ; j < 4 ; j++)
    {
      aruco_cal_in[j].val[0] = arucomarkers_camera_3d[i][j].x;
      aruco_cal_in[j].val[1] = arucomarkers_camera_3d[i][j].y;
      aruco_cal_in[j].val[2] = arucomarkers_camera_3d[i][j].z;
      aruco_cal_in[j].val[3] = 1;

      aruco_cal_out[j] = g2b * c2g * aruco_cal_in[j];
      // cout<<"x : "<< aruco_cal_out[j].val[0] <<" y : "<<aruco_cal_out[j].val[1]<<" z : "<<aruco_cal_out[j].val[2]<<endl;
      
    }
    cout<<"x : "<< aruco_cal_out[3].val[0] <<" y : "<<aruco_cal_out[3].val[1]<<" z : "<<aruco_cal_out[3].val[2]<<endl; // for 4th corner
    cv::putText(outputImage, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out[3].val[0], aruco_cal_out[3].val[1], aruco_cal_out[3].val[2] ), markerCorners[wantedMarker][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);

    QString text_for_append;
    text_for_append.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[wantedMarker], aruco_cal_out[3].val[0], aruco_cal_out[3].val[1], aruco_cal_out[3].val[2] );
    ui->textEdit_process_log->append(text_for_append);

    arucomarker_world_3d.x = aruco_cal_out[3].val[0];
    arucomarker_world_3d.y = aruco_cal_out[3].val[1];
    arucomarker_world_3d.z = aruco_cal_out[3].val[2];    

    // 그냥 2d 점으로 중앙 찾는 방식
    float aruco_2d_center[2] = {0,0}, aruco_3d_center[3]= {0,0,0};
    for(int j = 0; j < markerCorners[wantedMarker].size(); j++)
    {
      aruco_2d_center[0] += markerCorners[wantedMarker][j].x;
      aruco_2d_center[1] += markerCorners[wantedMarker][j].y;//소시야 개수에 맞게 개수 늘려야 되네 그런 이슈가 있네dMarker][j].y;
    }

    aruco_2d_center[0] = aruco_2d_center[0] / 4.;
    aruco_2d_center[1] = aruco_2d_center[1] / 4.;

    float distance_center = depth_image.at<float>(aruco_2d_center[1], aruco_2d_center[0])*1000;
    rs2_deproject_pixel_to_point(aruco_3d_center, &RS_camera_info_, aruco_2d_center, distance_center);

    // std::cout << aruco_3d_center[0] << std::endl;
    // std::cout << aruco_3d_center[1] << std::endl;
    
    //tvec 값이랑 rvec 값 사용해서 굳이 2d 좌표계를 만든 뒤 그 원점을 사용하는 방식
    std::vector<cv::Point3f> axesPoints;
    axesPoints.push_back(cv::Point3f(0, 0, 0));
    axesPoints.push_back(cv::Point3f(100, 0, 0));
    axesPoints.push_back(cv::Point3f(0,100, 0));
    axesPoints.push_back(cv::Point3f(0, 0, 100));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axesPoints, rvecs[wantedMarker], tvecs[wantedMarker], A, distCoeffs, imagePoints);

    ///cout << "imagePoint " << imagePoints[0].x << "," <<imagePoints[0].y<<endl;

    float distance_cent = depth_image.at<float>(imagePoints[0].y, imagePoints[0].x)*1000;

    float center_3d[3];
    float center_pixel[2];

    center_pixel[0] = imagePoints[0].x;
    center_pixel[1] = imagePoints[0].y;
    rs2_deproject_pixel_to_point(center_3d, &RS_camera_info_, center_pixel, distance_cent);

    cv::Matx33d R_aruco; //

    cv::Matx44f T_c2b;
    T_c2b = g2b * c2g;

    cv::Rodrigues(rvecs[wantedMarker],R_aruco);
    // if(fov == LARGE) cv::Rodrigues(rvecs[i],R_aruco);//
    // else cv::Rodrigues(rvecs[current_marker_num],R_aruco);
    
    cv::Matx44f T_aruco2cam(
        static_cast<float>(R_aruco(0, 0)), static_cast<float>(R_aruco(0, 1)), static_cast<float>(R_aruco(0, 2)), center_3d[0],
        static_cast<float>(R_aruco(1, 0)), static_cast<float>(R_aruco(1, 1)), static_cast<float>(R_aruco(1, 2)), center_3d[1],
        static_cast<float>(R_aruco(2, 0)), static_cast<float>(R_aruco(2, 1)), static_cast<float>(R_aruco(2, 2)), center_3d[2],
        0.0f, 0.0f, 0.0f, 1.0f
    );
    //static_cast<float>(tvec[current_marker_num](0))

    cv::Matx44f T_b2a = T_aruco2cam.inv() * T_c2b.inv(); // it's mean c2a * b2c

    cv::Matx44f T_c2a_want(
          1.0f, 0.0f, 0.0f, 0.0f,
          0.0f, -1.0f, 0.0f, 0.0f,
          0.0f, 0.0f, -1.0f, 300.0f,
          0.0f, 0.0f, 0.0f, 1.0f
    );

    ///cout << "rvec : " <<rvecs[wantedMarker][0]<< ", "<<rvecs[wantedMarker][1] << ", "<<rvecs[wantedMarker][2]<< endl;
    ///cout << "tvec : " <<tvecs[wantedMarker][0]<< ", "<<tvecs[wantedMarker][1] << ", "<<tvecs[wantedMarker][2]<< endl;
    ///cout << "tvec(use depth) : " <<center_3d[0]<< ", "<<center_3d[1] << ", "<<center_3d[2]<< endl;
    ///cout << "tvec? : " << aruco_3d_center[0]<< ", "<<aruco_3d_center[1]<<", "<<aruco_3d_center[2]<<endl;

    cv::Matx44f T_c2b_want = T_b2a.inv() * T_c2a_want;
    cv::Matx44f T_end2camera = c2g.inv();

    // end effector의 위치 추출
    cv::Matx44f T_end2base = T_c2b_want * T_end2camera;
    x = T_end2base(0, 3);
    y = T_end2base(1, 3);
    z = T_end2base(2, 3);// + 250;

    // end effector의 방향 추출
    cv::Matx33f R_end2base;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_end2base(i, j) = T_end2base(i, j);
        }
    }
    p_rad = std::acos(R_end2base(2, 2));
    if (std::sin(p_rad) == 0.0)
    {
      r_rad = 0; yw_rad = std::acos(R_end2base(0,0)) - r_rad;
    }
    else
    {
      if (R_end2base(1, 2) / std::sin(p_rad) > 0) {
          r_rad = std::acos(R_end2base(0, 2) / std::sin(p_rad));
      } else {
          r_rad = -std::acos(R_end2base(0, 2) / std::sin(p_rad));
      }
      if (R_end2base(2, 1) / std::sin(p_rad) > 0) {
          yw_rad = std::acos(-R_end2base(2, 0) / std::sin(p_rad));
      } else {
          yw_rad = -std::acos(-R_end2base(2, 0) / std::sin(p_rad));
      }
    }
    r = r_rad * 180.0 / M_PI;
    p = p_rad * 180.0 / M_PI;
    w = yw_rad * 180.0 / M_PI;    

    arucomarkers_world_3d.push_back(arucomarker_world_3d);
  }

  cv::Mat showimage = outputImage.clone();
  cv::resize(showimage, showimage, cv::Size(640, 360));
  ui->label_process_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

  // /home/bkyb/Images/
  index = markerIds[current_marker_num];
  output_pos[0] = x; output_pos[1] = y; output_pos[2] = z; output_pos[3] = r; output_pos[4] = p; output_pos[5] = w;

  switch (fov) {
    case FOV::LARGE:
        cv::imwrite("/home/bkyb/Images/world_3d_aruco.jpg", outputImage);
        output_point = arucomarkers_world_3d;
        break;
    case FOV::SMALL:
        sprintf(buf, "/home/bkyb/Images/marker_id%02d_world_small_FOV.jpg",index);
        cv::imwrite(buf, outputImage);
        // for(int i = 0; i < sizeof(output_pos); i++) outputPos[i] = output_pos[i];
        outputPos.assign(output_pos.begin(), output_pos.end()); 
        break;
    case FOV::SMALLCOR:
        sprintf(buf, "/home/bkyb/Images/marker_id%02d_world_%02dcorrected_FOV.jpg",index, cor);
        cv::imwrite(buf, outputImage);
        outputPos.assign(output_pos.begin(), output_pos.end()); 
        break;
    default:
        break;
  }
}

void campus_pj::on_pushButton_process_start_clicked()
{
  QString text;
  ui->textEdit_process_log->append(text.sprintf("Process Start!"));

  float velx[2] = {0,0};
  float accx[2] = {0,0};
  float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
  float pos_home[6] = {650, 340, 865, 0, -180.0, 180.0};

  float pos_to_go[6] = {0,0,0,0,0,0};
  float pos_to_go2[6] = {0,0,0,0,0,0};
  float x, y, z, r, p, w;

  std::vector<float> pos_empty(6,0.0f), pos_vec(6,0.0f);
  
  std::vector<int> checking_markeids, checking_markeids2, checking_markeids3, worldview_markeids;//소시야 개수에 맞게 개수 늘려야 되네 그런 이슈가 있네

  std::vector<cv::Point3f> arucomarkers_world_3d, emptyVec;
  arucomarkers_world_3d.emplace_back(0.0, 0.0, 0.0);
  emptyVec.emplace_back(0.0, 0.0, 0.0);

  ui->textEdit_process_log->append(text.sprintf("Move Home Position"));
  movej(joint_home,0,0,4.5,0,0,0,0);

  ui->textEdit_process_log->append(text.sprintf("Move to large FOV Position"));
  cout<<"Large FOV start!"<<endl;
  get_markers_pos_and_orientation(pos_home, FOV::LARGE, checking_markeids, worldview_markeids, arucomarkers_world_3d, pos_empty);
  // 위의 과정이 내가 생각한대로 동작한다면 large FOV로 이동하고 arucomarkers_world_3d에 내용이 채워져있어야 한다.
  checking_markeids.resize(sizeof(worldview_markeids), -1);checking_markeids2.resize(sizeof(worldview_markeids), -1);checking_markeids3.resize(sizeof(worldview_markeids), -1);

  int repeatCounts = arucomarkers_world_3d.size();
  cout << "times : " << repeatCounts << endl;

  for(int i = 0; i < repeatCounts; i++)
  {   
    x = arucomarkers_world_3d[i].x; 
    y = arucomarkers_world_3d[i].y; 
    z = 300.0 + arucomarkers_world_3d[i].z; // 거리 조절 여기서
    r = 0.0; p = -180.0; w = 180.0;

    calculateEnd2Base(x, y, z, r, p, w);
    pos_to_go[0] = x; pos_to_go[1] = y; pos_to_go[2] = z; pos_to_go[3] = r; pos_to_go[4] = p; pos_to_go[5] = w; 
    cout<<"\nSmall FOV start!"<<endl;
    get_markers_pos_and_orientation(pos_to_go, FOV::SMALL, checking_markeids, worldview_markeids, emptyVec, pos_vec, i);
    for(int j = 0; j < 6; j++) pos_to_go2[j] = pos_vec[j];
    // 위의 과정이 내가 생각한대로 동작한다면, small FOV로 이동하고 checking_markeids가 변경되고 pos_next가 마커 중앙을 기준으로 변경되어야 한다.
    cout<<"\nCorrected FOV 1 start!"<<endl;
    get_markers_pos_and_orientation(pos_to_go2, FOV::SMALLCOR, checking_markeids2, worldview_markeids, emptyVec, pos_vec, i, 1);
    // for(int j = 0; j < 6; j++) pos_to_go2[j] = pos_vec[j];
    //
    // cout<<"\nCorrected FOV 2 start!"<<endl;
    // get_markers_pos_and_orientation(pos_to_go2, FOV::SMALLCOR, checking_markeids3, worldview_markeids, emptyVec, pos_vec, i, 2);
  }
}

void campus_pj::on_pushButton_process_printPixels_clicked()
{
  for(int i = 0 ; i < current_corners.size(); i++)
  {
    cout<< i << " : ";
    for(int j = 0; j < current_corners[i].size(); j++)
    {
      cout << "x : "<< current_corners[i][j].x << " y : "<< current_corners[i][j].y << " z : " <<  depth_image.at<float>(current_corners[i][j].y, current_corners[i][j].x)*1000 << endl;
    }
  }
}

void campus_pj::on_pushButton_processB_get_reps_clicked()
{
  QString repsText = ui->textEdit_get_reps->toPlainText();

  repeat_counts = !repsText.isEmpty() ? repsText.toInt() : 1;
}

void campus_pj::on_pushButton_processB_get_markers_pos_clicked(){
  /*여기서는 뭐를 해야 되느냐
    포즈를 읽어와서 저장합니다.
  */
  QString text;

  ui->textEdit_processB_log->append(text.sprintf("Process Start!"));


  float velx[2] = {0,0};
  float accx[2] = {0,0};
  float joint_home[6] = {90.0, 0.0, 90.0, 0.0, 90.0, 0.0};
  float pos_home[6] = {650, 340, 565, 0, -180.0, 180.0};

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;

  srv.request.space_type = 1; // 0일때 관절, 1일때 작업공간 이래요

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  ros::NodeHandlePtr node_3 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose2 = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv3;

  srv3.request.space_type = 0; // 0일때 관절, 1일때 작업공간 이래요

  ui->textEdit_processB_log->append(text.sprintf("Move Home Position"));
  movej(joint_home,0,0,4.5,0,0,0,0);
  movel(pos_home,velx,accx,4.5,0,0,0,0,0); // move home position
  cv::waitKey(1);

  cv::Mat image_aruco = color_image_raw.clone();

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
  cv::aruco::detectMarkers(image_aruco, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  cv::Mat outputImage = color_image_raw.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);


  cv::Matx41f aruco_cal_in;
  cv::Matx41f aruco_cal_out;
  cv::Point3f arucomarker_3d;
  //  cv::Matx33d Rz1, Ry, Rz2;

  cv::Mat outputImage_world_3d = color_image_raw.clone();

  // cv::Matx44f c2g = {0.997384999638619, -0.0719288009035676, -0.007029231568742221, -27.02221594881259,
  //                   0.07161484260894792, 0.9967185601557664, -0.03772832038550635, -100.8828287678684,
  //                   0.009719918413633304, 0.03712626350160663, 0.9992633105165232, -11.98360880536063,
  //                   0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

  if(srvGetpose.call(srv))
  {
     for(int i=0; i<6; i++)
     {
       campus_pj::robot_current_pose[i] = srv.response.pos[i];
     }
  }

  float data[9];
  int l = 0;

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
     for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
     {
       for(int j=0 ; j<3 ; j++)
       {
         data[l] = srv2.response.rot_matrix[i].data[j] ;
         l++;
       }
     }
  }
  else
  {
      ros::shutdown();
     // return -1;
  }
  l = 0;
  
  cv::Matx44f g2b = {
    data[0], data[1], data[2], robot_current_pose[0],
    data[3], data[4], data[5], robot_current_pose[1],
    data[6], data[7], data[8], robot_current_pose[2],
    0, 0, 0, 1};

  for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);

  float distance, distance2;
  float aruco_2d[2], aruco_3d[3];
  std::vector<cv::Point3f> arucomarkers_camera_3d, arucomarkers_world_3d_1st;

  for(int i = 0 ; i < markerCorners.size() ; i++)
  {
    distance = depth_image.at<float>(markerCorners[i][3].y, markerCorners[i][3].x)*1000;

    aruco_2d[0] = markerCorners[i][3].x;
    aruco_2d[1] = markerCorners[i][3].y;

    rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance);
    cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    
    arucomarker_3d.x = aruco_3d[0];
    arucomarker_3d.y = aruco_3d[1];
    arucomarker_3d.z = aruco_3d[2];

    arucomarkers_camera_3d.push_back(arucomarker_3d);

    cv::Mat showimage = outputImage.clone();
    cv::resize(showimage, showimage, cv::Size(640, 360));
    ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    for(int i = 0 ; i < arucomarkers_camera_3d.size() ; i++)
    {
      aruco_cal_in.val[0] = arucomarkers_camera_3d[i].x;
      aruco_cal_in.val[1] = arucomarkers_camera_3d[i].y;
      aruco_cal_in.val[2] = arucomarkers_camera_3d[i].z;
      aruco_cal_in.val[3] = 1;

      aruco_cal_out = g2b * c2g * aruco_cal_in;

      cv::putText(outputImage_world_3d, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);

      // QString text_for_append000;

      // text_for_append000.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[i], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
      // ui->textEdit_processB_log->append(text_for_append000);

      cv::Point3f arucomarker_world_3d;
      arucomarker_world_3d.x = aruco_cal_out.val[0];
      arucomarker_world_3d.y = aruco_cal_out.val[1];
      arucomarker_world_3d.z = aruco_cal_out.val[2];

      arucomarkers_world_3d_1st.push_back(arucomarker_world_3d);
    }
  }

  if(srvGetpose2.call(srv3))
  {
      for(int i=0; i<6; i++)
      {
        Joint_A[i] = srv3.response.pos[i];
      }
  }

  cv::Mat showimage = outputImage_world_3d.clone();
  cv::resize(showimage, showimage, cv::Size(640, 360));
  ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

  cv::imwrite("world_3d_aruco.jpg", outputImage_world_3d);

  int numOfMarkers = markerCorners.size();
  

  for(int i = 0 ; i < numOfMarkers; i++)
  {
    float xx, yy, zz, rr, pp, ww;
    xx = arucomarkers_world_3d_1st[i].x; 
    yy = arucomarkers_world_3d_1st[i].y; 
    zz = 300.0 + arucomarkers_world_3d_1st[i].z; 
    rr = 0.0; pp = -180.0; ww = 180.0;

    calculateEnd2Base(xx, yy, zz, rr, pp, ww);

    float targetpos[6] = {xx, yy, zz, rr, pp, ww};
    movel(targetpos,velx,accx,4.0,0,0,0,0,0);
    wait(1.5);

    cv::waitKey(1);
    cv::Mat image_marker_1st = color_image_raw.clone();
    cv::aruco::detectMarkers(image_marker_1st, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    aruco_2d[0] = markerCorners[0][3].x;
    aruco_2d[1] = markerCorners[0][3].y;

    if(srvGetpose.call(srv))
    {
        for(int i=0; i<6; i++)
        {
          campus_pj::robot_current_pose[i] = srv.response.pos[i];
        }
    }

    if(srvGetrotm.call(srv2))
    {
      // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
      for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
      {
        for(int j=0 ; j<3 ; j++)
        {
          data[l] = srv2.response.rot_matrix[i].data[j] ;
          l++;
        }
      }
    }
    else
    {
        ros::shutdown();
      // return -1;
    }
    l = 0;
  
    cv::Matx44f g2b1 = {
      data[0], data[1], data[2], robot_current_pose[0],
      data[3], data[4], data[5], robot_current_pose[1],
      data[6], data[7], data[8], robot_current_pose[2],
      0, 0, 0, 1};

    distance2 = depth_image.at<float>(markerCorners[0][3].y, markerCorners[0][3].x)*1000;

    rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance2 );


    cv::Mat showimage = image_marker_1st.clone();

    cv::putText(showimage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    cv::resize(showimage, showimage, cv::Size(640, 360));
    ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    aruco_cal_in.val[0] = aruco_3d[0];
    aruco_cal_in.val[1] = aruco_3d[1];
    aruco_cal_in.val[2] = aruco_3d[2];
    aruco_cal_in.val[3] = 1;

    aruco_cal_out = g2b1 * c2g * aruco_cal_in;
  
    cv::putText(image_marker_1st, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    cv::circle(image_marker_1st, markerCorners[0][3], 2, cv::Scalar(0, 255, 0), 2);

    QString text_for_append001;

    text_for_append001.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[0], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
    ui->textEdit_processB_log->append(text_for_append001);

    text_for_append001.sprintf("id %d : pixel = (%f, %f) ",markerIds[0], markerCorners[0][3].x, markerCorners[0][3].y);
    ui->textEdit_processB_log->append(text_for_append001);

    if(srvGetpose2.call(srv3))
    {
        for(int i=0; i<6; i++)
        {
          Joint_B[i] = srv3.response.pos[i];
        }
    }

    if(srvGetpose.call(srv))
    {
        for(int i=0; i<6; i++)
        {
          Posx_B[i] = srv.response.pos[i];
        }
    }
  }
}

void campus_pj::on_pushButton_processB_get_markers_posj_clicked()
{
  //이동이 없는 동작인듯? 

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;

  srv.request.space_type = 1; // 0일때 관절, 1일때 작업공간 이래요

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 

  cv::Mat image_marker_1st = color_image_raw.clone();
  cv::aruco::detectMarkers(image_marker_1st, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);


  float distance, distance2;
  float aruco_2d[2], aruco_3d[3];
  std::vector<cv::Point3f> arucomarkers_camera_3d, arucomarkers_world_3d_1st;

  cv::Matx41f aruco_cal_in;
  cv::Matx41f aruco_cal_out;
  cv::Point3f arucomarker_3d;

  float data[9];
  int l =0;

  aruco_2d[0] = markerCorners[0][3].x;
  aruco_2d[1] = markerCorners[0][3].y;

  // cv::Matx44f c2g = {0.997384999638619, -0.0719288009035676, -0.007029231568742221, -27.02221594881259,
  //                 0.07161484260894792, 0.9967185601557664, -0.03772832038550635, -100.8828287678684,
  //                 0.009719918413633304, 0.03712626350160663, 0.9992633105165232, -11.98360880536063,
  //                 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
  }

  if(srvGetrotm.call(srv2))
  {
    // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
    for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
    {
      for(int j=0 ; j<3 ; j++)
      {
        data[l] = srv2.response.rot_matrix[i].data[j] ;
        l++;
      }
    }
  }
  else
  {
      ros::shutdown();
    // return -1;
  }
  l = 0;

  cv::Matx44f g2b1 = {
    data[0], data[1], data[2], robot_current_pose[0],
    data[3], data[4], data[5], robot_current_pose[1],
    data[6], data[7], data[8], robot_current_pose[2],
    0, 0, 0, 1};

  distance2 = depth_image.at<float>(markerCorners[0][3].y, markerCorners[0][3].x)*1000;

  rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance2 );


  cv::Mat showimage = image_marker_1st.clone();

  cv::putText(showimage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
  cv::resize(showimage, showimage, cv::Size(640, 360));
  ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

  aruco_cal_in.val[0] = aruco_3d[0];
  aruco_cal_in.val[1] = aruco_3d[1];
  aruco_cal_in.val[2] = aruco_3d[2];
  aruco_cal_in.val[3] = 1;

  aruco_cal_out = g2b1 * c2g * aruco_cal_in;

  cv::putText(image_marker_1st, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[0][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
  cv::circle(image_marker_1st, markerCorners[0][3], 2, cv::Scalar(0, 255, 0), 2);

  QString text_for_append001;

  text_for_append001.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) ",markerIds[0], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] );
  ui->textEdit_processB_log->append(text_for_append001);

  text_for_append001.sprintf("id %d : pixel = (%f, %f) ",markerIds[0], markerCorners[0][3].x, markerCorners[0][3].y);
  ui->textEdit_processB_log->append(text_for_append001);
}

void campus_pj::on_pushButton_processB_moveA2B_clicked()
{
  /* 여기서는 뭘 하느느냐 
  아까 저장했던 조인트 값들로 이동을 합니다. 
  그러고 나서 마커의 좌표를 찾는 그런 행위를 한다고 보면 됩니다.
  */
  QString text;

  ui->textEdit_processB_log->append(text.sprintf("Process Start!"));
  
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;

  srv.request.space_type = 1; // 0일때 관절, 1일때 작업공간 이래요

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  for(int k = 0; k < repeat_counts; k++){



  movej(Joint_A,0,0,4.5,0,0,0,0);

  movej(Joint_B,0,0,4.5,0,0,0,0);
  wait(1.5);

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
  }

    float data[9];
    int l = 0;

    if(srvGetrotm.call(srv2))
    {
      // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
      for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
      {
        for(int j=0 ; j<3 ; j++)
        {
          data[l] = srv2.response.rot_matrix[i].data[j] ;
          l++;
        }
      }
    }
    else
    {
        ros::shutdown();
      // return -1;
    }
    l = 0;
    
  cv::Matx44f g2b = {
    data[0], data[1], data[2], robot_current_pose[0],
    data[3], data[4], data[5], robot_current_pose[1],
    data[6], data[7], data[8], robot_current_pose[2],
    0, 0, 0, 1};

  cout << k << " : " <<data[0]<< ", "<<data[1] << ", "<<data[2]<<", "<<robot_current_pose[0]<< endl;
  cout << "    " <<data[3]<< ", "<<data[4] << ", "<<data[5]<<", "<<robot_current_pose[1]<< endl;
  cout << "    " <<data[6]<< ", "<<data[7] << ", "<<data[8]<<", "<<robot_current_pose[2]<< endl;

  // cv::Matx44f c2g = {0.997384999638619, -0.0719288009035676, -0.007029231568742221, -27.02221594881259,
  //                   0.07161484260894792, 0.9967185601557664, -0.03772832038550635, -100.8828287678684,
  //                   0.009719918413633304, 0.03712626350160663, 0.9992633105165232, -11.98360880536063,
  //                   0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

  cv::Mat image_aruco = color_image_raw.clone();

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
  cv::aruco::detectMarkers(image_aruco, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  cv::Mat outputImage = color_image_raw.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  cv::Matx41f aruco_cal_in;
  cv::Matx41f aruco_cal_out;

  cv::Mat outputImage_world_3d = color_image_raw.clone();

  for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);

  float distance, distance2;
  float aruco_2d[2], aruco_3d[3];
  std::vector<cv::Point3f> arucomarkers_camera_3d;
  cv::Point3f arucomarker_3d;

  for(int i = 0 ; i < markerCorners.size() ; i++)
  {
    distance = depth_image.at<float>(markerCorners[i][3].y, markerCorners[i][3].x)*1000;

    aruco_2d[0] = markerCorners[i][3].x;
    aruco_2d[1] = markerCorners[i][3].y;

    rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance);
    cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    
    arucomarker_3d.x = aruco_3d[0];
    arucomarker_3d.y = aruco_3d[1];
    arucomarker_3d.z = aruco_3d[2];

    arucomarkers_camera_3d.push_back(arucomarker_3d);

    cv::Mat showimage = outputImage.clone();
    cv::resize(showimage, showimage, cv::Size(640, 360));
    ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    for(int j = 0 ; j < arucomarkers_camera_3d.size() ; j++)
    {
      aruco_cal_in.val[0] = arucomarkers_camera_3d[j].x;
      aruco_cal_in.val[1] = arucomarkers_camera_3d[j].y;
      aruco_cal_in.val[2] = arucomarkers_camera_3d[j].z;
      aruco_cal_in.val[3] = 1;

      aruco_cal_out = g2b * c2g * aruco_cal_in;

      cv::putText(outputImage_world_3d, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[j][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);

      QString text_for_append000;
      text_for_append000.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) \nid %d : pixel = (%f, %f) ",markerIds[j], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2],
      markerIds[0], markerCorners[j][3].x, markerCorners[j][3].y);
      ui->textEdit_processB_log->append(text_for_append000);

      // text_for_append000.sprintf("\n id %d : pixel = (%f, %f) ",markerIds[0], markerCorners[j][3].x, markerCorners[j][3].y);
      // ui->textEdit_process_log->append(text_for_append000);
    }
  }
  wait(0.5);
  }
}

void campus_pj::on_pushButton_processB_moveA2C2B_clicked()
{
  /* 여기서는 뭘 하느느냐 
  아까 저장했던 조인트 값들로 이동을 합니다. 
  그러고 나서 마커의 좌표를 찾는 그런 행위를 한다고 보면 됩니다.
  근데 여기서 랜덤 좌표로 한 번 이동했다가 
  */

  float velx[2] = {0,0};
  float accx[2] = {0,0};
  QString text;

  ui->textEdit_processB_log->append(text.sprintf("\nProcess Start!"));
  
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;

  srv.request.space_type = 1; // 0일때 관절, 1일때 작업공간 이래요

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  for(int k = 0; k < repeat_counts; k++){

  movej(Joint_A,0,0,4.5,0,0,0,0);

  std::random_device rd;
  std::mt19937 generator(rd());

  float min_value = 300.0f;
  float max_value = 500.0f;

  std::uniform_real_distribution<float> distribution(min_value, max_value);

  float random_value_x = distribution(generator);
  float random_value_y = distribution(generator);
  float random_value_z = distribution(generator);
  float pos_c[6] = {random_value_x, random_value_y, random_value_z, 0, -180.0, 180.0};

  QString text_for_append;
  text_for_append.sprintf("Coordinate of C : (%.5lf, %.5lf, %.5lf) ",random_value_x, random_value_y,random_value_z);
  ui->textEdit_processB_log->append(text_for_append);

  movel(pos_c,velx,accx,4.5,0,0,0,0,0); 
  movej(Joint_B,0,0,4.5,0,0,0,0);
  wait(1.5);

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
  }

    float data[9];
    int l = 0;

    if(srvGetrotm.call(srv2))
    {
      // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
      for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
      {
        for(int j=0 ; j<3 ; j++)
        {
          data[l] = srv2.response.rot_matrix[i].data[j] ;
          l++;
        }
      }
    }
    else
    {
        ros::shutdown();
      // return -1;
    }
    l = 0;
    
  cv::Matx44f g2b = {
    data[0], data[1], data[2], robot_current_pose[0],
    data[3], data[4], data[5], robot_current_pose[1],
    data[6], data[7], data[8], robot_current_pose[2],
    0, 0, 0, 1};

  cout << k << " : " <<data[0]<< ", "<<data[1] << ", "<<data[2]<<", "<<robot_current_pose[0]<< endl;
  cout << "    " <<data[3]<< ", "<<data[4] << ", "<<data[5]<<", "<<robot_current_pose[1]<< endl;
  cout << "    " <<data[6]<< ", "<<data[7] << ", "<<data[8]<<", "<<robot_current_pose[2]<< endl;

  // cv::Matx44f c2g = {0.997384999638619, -0.0719288009035676, -0.007029231568742221, -27.02221594881259,
  //                   0.07161484260894792, 0.9967185601557664, -0.03772832038550635, -100.8828287678684,
  //                   0.009719918413633304, 0.03712626350160663, 0.9992633105165232, -11.98360880536063,
  //                   0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

  cv::Mat image_aruco = color_image_raw.clone();

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
  cv::aruco::detectMarkers(image_aruco, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  cv::Mat outputImage = color_image_raw.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  cv::Matx41f aruco_cal_in;
  cv::Matx41f aruco_cal_out;

  cv::Mat outputImage_world_3d = color_image_raw.clone();

  for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);

  float distance, distance2;
  float aruco_2d[2], aruco_3d[3];
  std::vector<cv::Point3f> arucomarkers_camera_3d;
  cv::Point3f arucomarker_3d;

  for(int i = 0 ; i < markerCorners.size() ; i++)
  {
    distance = depth_image.at<float>(markerCorners[i][3].y, markerCorners[i][3].x)*1000;

    aruco_2d[0] = markerCorners[i][3].x;
    aruco_2d[1] = markerCorners[i][3].y;

    rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance);
    cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    
    arucomarker_3d.x = aruco_3d[0];
    arucomarker_3d.y = aruco_3d[1];
    arucomarker_3d.z = aruco_3d[2];

    arucomarkers_camera_3d.push_back(arucomarker_3d);

    cv::Mat showimage = outputImage.clone();
    cv::resize(showimage, showimage, cv::Size(640, 360));
    // ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    for(int j = 0 ; j < arucomarkers_camera_3d.size() ; j++)
    {
      aruco_cal_in.val[0] = arucomarkers_camera_3d[j].x;
      aruco_cal_in.val[1] = arucomarkers_camera_3d[j].y;
      aruco_cal_in.val[2] = arucomarkers_camera_3d[j].z;
      aruco_cal_in.val[3] = 1;

      aruco_cal_out = g2b * c2g * aruco_cal_in;

      cv::putText(outputImage_world_3d, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[j][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);

      QString text_for_append000;

      text_for_append000.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) \nid %d : pixel = (%f, %f) ",markerIds[j], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2],markerIds[0], markerCorners[j][3].x, markerCorners[j][3].y );
      ui->textEdit_processB_log->append(text_for_append000);

      // text_for_append000.sprintf("\n id %d : pixel = (%f, %f) ",markerIds[0], markerCorners[j][3].x, markerCorners[j][3].y);
      // ui->textEdit_process_log->append(text_for_append000);

      showimage = outputImage_world_3d.clone();
      cv::resize(showimage, showimage, cv::Size(640, 360));
      ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
    }
  }
  wait(0.5);
  }
}

void campus_pj::on_pushButton_processB_moveA2C2B_X_clicked()
{
  /* 여기서는 뭘 하느느냐 
  아까 저장했던 조인트 값들로 이동을 합니다. 
  그러고 나서 마커의 좌표를 찾는 그런 행위를 한다고 보면 됩니다.
  근데 여기서 랜덤 좌표로 한 번 이동했다가 
  */

  float velx[2] = {0,0};
  float accx[2] = {0,0};
  QString text;

  ui->textEdit_processB_log->append(text.sprintf("\nProcess Start!"));
  
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetpose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
  dsr_msgs::GetCurrentPose srv;

  srv.request.space_type = 1; // 0일때 관절, 1일때 작업공간 이래요

  ros::NodeHandlePtr  node_2 = boost::make_shared<ros::NodeHandle>();
  ros::ServiceClient srvGetrotm = node_2->serviceClient<dsr_msgs::GetCurrentRotm>("/dsr01m1013/aux_control/get_current_rotm");
  dsr_msgs::GetCurrentRotm srv2;

  srv2.request.ref = 0;

  for(int k = 0; k <repeat_counts; k++){


  movej(Joint_A,0,0,4.5,0,0,0,0);

  std::random_device rd;
  std::mt19937 generator(rd());

  float min_value = 300.0f;
  float max_value = 500.0f;

  std::uniform_real_distribution<float> distribution(min_value, max_value);

  float random_value_x = distribution(generator);
  float random_value_y = distribution(generator);
  float random_value_z = distribution(generator);
  float pos_c[6] = {random_value_x, random_value_y, random_value_z, 0, -180.0, 180.0};

  QString text_for_append;
  text_for_append.sprintf("Coordinate of C : (%.5lf, %.5lf, %.5lf) ",random_value_x, random_value_y,random_value_z);
  ui->textEdit_processB_log->append(text_for_append);

  movel(pos_c,velx,accx,4.5,0,0,0,0,0);
  movel(Posx_B,velx,accx,4.5,0,0,0,0,0); 
  // movej(Joint_B,0,0,4.5,0,0,0,0);
  wait(1.5);

  if(srvGetpose.call(srv))
  {
      for(int i=0; i<6; i++)
      {
        campus_pj::robot_current_pose[i] = srv.response.pos[i];
      }
  }

    float data[9];
    int l = 0;

    if(srvGetrotm.call(srv2))
    {
      // std::cout << "size : " << srv2.response.rot_matrix.size() << std::endl;
      for(int i = 0 ; i < srv2.response.rot_matrix.size() ; i++)
      {
        for(int j=0 ; j<3 ; j++)
        {
          data[l] = srv2.response.rot_matrix[i].data[j] ;
          l++;
        }
      }
    }
    else
    {
        ros::shutdown();
      // return -1;
    }
    l = 0;
    
  cv::Matx44f g2b = {
    data[0], data[1], data[2], robot_current_pose[0],
    data[3], data[4], data[5], robot_current_pose[1],
    data[6], data[7], data[8], robot_current_pose[2],
    0, 0, 0, 1};

  cout << k << " : " <<data[0]<< ", "<<data[1] << ", "<<data[2]<<", "<<robot_current_pose[0]<< endl;
  cout << "    " <<data[3]<< ", "<<data[4] << ", "<<data[5]<<", "<<robot_current_pose[1]<< endl;
  cout << "    " <<data[6]<< ", "<<data[7] << ", "<<data[8]<<", "<<robot_current_pose[2]<< endl;

  // cv::Matx44f c2g = {0.997384999638619, -0.0719288009035676, -0.007029231568742221, -27.02221594881259,
  //                   0.07161484260894792, 0.9967185601557664, -0.03772832038550635, -100.8828287678684,
  //                   0.009719918413633304, 0.03712626350160663, 0.9992633105165232, -11.98360880536063,
  //                   0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00};

  cv::Mat image_aruco = color_image_raw.clone();

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); 
  cv::aruco::detectMarkers(image_aruco, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

  cv::Mat outputImage = color_image_raw.clone();
  cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

  cv::Matx41f aruco_cal_in;
  cv::Matx41f aruco_cal_out;

  cv::Mat outputImage_world_3d = color_image_raw.clone();

  for(int i = 0; i < markerCorners.size(); i++) cv::circle(outputImage, markerCorners[i][3], 2, cv::Scalar(0, 255, 0), 2);

  float distance, distance2;
  float aruco_2d[2], aruco_3d[3];
  std::vector<cv::Point3f> arucomarkers_camera_3d;
  cv::Point3f arucomarker_3d;

  for(int i = 0 ; i < markerCorners.size() ; i++)
  {
    distance = depth_image.at<float>(markerCorners[i][3].y, markerCorners[i][3].x)*1000;

    aruco_2d[0] = markerCorners[i][3].x;
    aruco_2d[1] = markerCorners[i][3].y;

    rs2_deproject_pixel_to_point(aruco_3d, &RS_camera_info_, aruco_2d, distance);
    cv::putText(outputImage, cv::format("camera3d = (%.1lf, %.1lf, %.1lf)",aruco_3d[0], aruco_3d[1], aruco_3d[2] ), markerCorners[i][3], cv::FONT_HERSHEY_DUPLEX, 0.45, cv::Scalar(255,255,225), 0);
    
    arucomarker_3d.x = aruco_3d[0];
    arucomarker_3d.y = aruco_3d[1];
    arucomarker_3d.z = aruco_3d[2];

    arucomarkers_camera_3d.push_back(arucomarker_3d);

    cv::Mat showimage = outputImage.clone();
    cv::resize(showimage, showimage, cv::Size(640, 360));
    // ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));

    for(int j = 0 ; j < arucomarkers_camera_3d.size() ; j++)
    {
      aruco_cal_in.val[0] = arucomarkers_camera_3d[j].x;
      aruco_cal_in.val[1] = arucomarkers_camera_3d[j].y;
      aruco_cal_in.val[2] = arucomarkers_camera_3d[j].z;
      aruco_cal_in.val[3] = 1;

      aruco_cal_out = g2b * c2g * aruco_cal_in;

      cv::putText(outputImage_world_3d, cv::format("world3d = (%.1lf, %.1lf, %.1lf)",aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2] ), markerCorners[j][3], cv::FONT_HERSHEY_DUPLEX, 0.35, cv::Scalar(255,255,225), 0);

      QString text_for_append000;

      text_for_append000.sprintf("id %d : world3d = (%.5lf, %.5lf, %.5lf) \nid %d : pixel = (%f, %f) ",markerIds[j], aruco_cal_out.val[0], aruco_cal_out.val[1], aruco_cal_out.val[2],markerIds[0], markerCorners[j][3].x, markerCorners[j][3].y );
      ui->textEdit_processB_log->append(text_for_append000);

      // text_for_append000.sprintf("\n id %d : pixel = (%f, %f) ",markerIds[0], markerCorners[j][3].x, markerCorners[j][3].y);
      // ui->textEdit_process_log->append(text_for_append000);

      showimage = outputImage_world_3d.clone();
      cv::resize(showimage, showimage, cv::Size(640, 360));
      ui->label_processB_image_raw->setPixmap(QPixmap::fromImage(QImage(showimage.data, showimage.cols, showimage.rows, showimage.step, QImage::Format_RGB888)));
    }
  }
  wait(0.5);
  }
}
