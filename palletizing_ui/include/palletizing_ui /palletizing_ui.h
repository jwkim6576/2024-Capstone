#ifndef PALLETIZING_ROBOT_UI_H
#define PALLETIZING_ROBOT_UI_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <boost/array.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <mutex>
#include <tf/transform_listener.h>
//Realsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
// QWidget
#include <QWidget>
#include <QFileDialog>
#include <QDateTime>
#include <QString>
#include <QDir>
#include <QTimer>
#include <QPixmap>
#include <QLabel>
#include <QMessageBox>
#include <QProcess>
#include <QDesktopWidget>
#include <QDebug>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QFileSystemWatcher>
#include <QPointer>
#include <QEventLoop>
#include <QPlainTextEdit>
#include <QRegularExpression>
// ROS Headers
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
// Doosan Robot
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/MoveStop.h>
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/GetCurrentPose.h>
#include <dsr_msgs/GetCurrentPosx.h>
#include <dsr_msgs/GetCurrentRotm.h>
#include <dsr_msgs/SetCtrlBoxDigitalOutput.h>

#define DEG2RAD(deg) (deg * M_PI / 180.0)
#define INFO QString("[INFO]")
#define ERROR QString("[ERROR]")
#define WARNING QString("[WARNING]")

namespace Ui {
class PalletizingRobotUi;
}

class PalletizingRobotUi : public QWidget
{
  Q_OBJECT

public:
  explicit PalletizingRobotUi(QWidget *parent = 0);
  ~PalletizingRobotUi();
  ros::NodeHandlePtr n;
  ros::Publisher empty_pub;

  ros::Subscriber Packinglog;

  void loadRobotConfigAndExecute();

private slots:
  void servoing();

  void half_sphere();

  void collisionDetectconPlot();

  void palletGazebo();

  void handleYoloError();

  void wait(float time_);

  void movelineToHome(float posx[6], float vel_[2] , float acc_[2], float time_, float object_x =0, float object_y =0, float object_z = 0);

  void movestop();

  void handleYoloFinished(int exitCode, QProcess::ExitStatus exitStatus);

  void packingLogCallBack(const std_msgs::String::ConstPtr &msg);

  void updatePackPlotimage();

  void updateServoingImage();

  void updatePickImage();

  void on_pushButton_activate_clicked();

  void on_pushButton_operate_clicked();

  void on_pushButton_shutdown_clicked();

  void on_pushButton_activate_robot_clicked();

  void on_pushButton_activate_home_clicked();

  void on_pushButton_operate_home_clicked();

  void on_pushButton_activate_camera_clicked();

  void on_pushButton_activate_search_clicked();

  void on_pushButton_activate_pack_clicked();

  void on_pushButton_operate_start_clicked();

  void on_pushButton_operate_stop_2_clicked();

  void on_pushButton_operate_reactivate_clicked();

  void on_pushButton_operate_stop_clicked();

  void on_pushButton_activate_operation_clicked();

  void on_pushButton_stop_deactivate_clicked();

  void on_pushButton_stop_operate_clicked();

  void on_pushButton_stop_activate_clicked();

  void on_pushButton_stop_home_clicked();

  void on_pushButton_activate_stop2_clicked();

private:
  Ui::PalletizingRobotUi *ui;
  QTimer *packingImageUpdateTimer;
  QTimer *servoingImageUpdateTimer;
  QTimer *pickImageUpdateTimer;
  QPixmap pixmap_logo;
  QPixmap pixmap_Packplot;
  QPixmap mapPixmap;
  QProcess *robotcon;
  QProcess *cameracon;
  QProcess *yolocon;
  QProcess *servoingcon;
  QProcess *collisionDetectcon;
  QProcess *collisionDetectPlotcon;
  QProcess *packingcon;
  QProcess *halfSpherecon;
  QProcess *palletGazebocon;
  QProcess *killRosCore;
  QProcess *findRosLaunch;
  QFileSystemWatcher* configWatcher;
  QFileSystemWatcher* imageWatcher1;
  QFileSystemWatcher* imageWatcher2;
  QWidget *currentWidget;
  QDesktopWidget desktop;
  QString yoloScriptPath;
  QString servoingScriptPath;
  QString collisionDetectScriptPath;
  QString collisionDetectPlotScriptPath;
  QString packingAlgorithmScriptPath;
  QString packingPlotPath;
  QString servoingPlotPath;
  QString pickPlotPath;
  QString logFileName;
  QString dsrlogoutput;
  QString camlogoutput;
  QString pathplanlogoutput;
  float current_pose[6];
  float posxg[6];
  float velBB[2] = {80, 40};
  float accBB[2] = {50, 30};
  float posx_home[6] = {500, 400, 650, 0 , 180, 0};
  void checkAndCleanRosLogs();
  void setKeyhelplabel();
  void readConfig(const QString& configFilePath);
  void handleProcessError(QProcess::ProcessError error);
  bool confirmAction(const QString &message, QMessageBox::Icon icon);
  void terminateAndDeleteProcess(QProcess* &process, const QString &processName);
  int showModeSelectionDialog();
  void logMessage(const QString &level,const QString &message, QPlainTextEdit *logArea);
  void startProcess(QProcess* &process, const QString &name, const QString &program, 
                      const QStringList &arguments, const QString &workingDir, QPlainTextEdit *logArea);
  void keyPressEvent(QKeyEvent *event);
  bool movejoint(float posx[6], float vel_, float acc_, float time_);
  void get_current_pose(int space_type);
  void posxG(float posx[6], float object_x =0, float object_y =0, float object_z = 0);
  std::vector<float> matVecMult(const std::vector<std::vector<float>>& mat, const std::vector<float>& vec);
  std::vector<std::vector<float>> rotZ(float angle);
  std::vector<std::vector<float>> rotY(float angle);
  std::vector<std::vector<float>> getZYZRotationMatrix(float yawZ1, float pitchY, float rollZ2);
  std::vector<std::vector<float>> rpy2vector(float yawZ1, float pitchY, float rollZ2);
};

#endif // PALLETIZING_ROBOT_UI_H
