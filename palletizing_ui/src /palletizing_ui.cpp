#include "palletizing_ui.h"
#include "ui_palletizing_ui.h"
#include "RosWorker.h"

// Constructor for initializing UI
PalletizingRobotUi::PalletizingRobotUi(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PalletizingRobotUi)
{
    ui->setupUi(this);

    int argc = 0; char **argv = NULL;

    // Initialize ROS node
    ros::init(argc, argv,"Palletizing_Robot_ui");
    n.reset(new ros::NodeHandle("~"));

    // Set up timers for ROS and image updates
    RosWorker *rosWorker = new RosWorker(this);
    connect(rosWorker, &QThread::finished, this, &QThread::deleteLater);
    packingImageUpdateTimer = new QTimer(this);
    servoingImageUpdateTimer = new QTimer(this);
    pickImageUpdateTimer = new QTimer(this);
    empty_pub = n->advertise<std_msgs::Empty>("/plantfarm/empty", 1);
    Packinglog = n->subscribe<std_msgs::String>("/log_info",10, &PalletizingRobotUi::packingLogCallBack, this);

    rosWorker->start(); // ROS 작업 스레드 시작
    ui->stackedWidget->setCurrentIndex(0);

    // Watcher for configuration file changes
    configWatcher = new QFileSystemWatcher(this);
    QString configFilePath = QCoreApplication::applicationDirPath() + "/../config/config.json";
    configWatcher->addPath(configFilePath);

    connect(configWatcher, &QFileSystemWatcher::fileChanged, this, &PalletizingRobotUi::loadRobotConfigAndExecute);

    // Center the window on the screen
    QRect screenGeometry = desktop.availableGeometry();
    int x = (screenGeometry.width() - width()) / 2;
    int y = (screenGeometry.height() - height()) / 2;
    move(x, y);

    // Load and display the logo
    pixmap_logo.load(":image/home_logo");
    int w = ui->label_logo->width();
    int h = ui->label_logo->height();
    ui->label_logo->setPixmap(pixmap_logo.scaled(w, h, Qt::KeepAspectRatio));
    if (!pixmap_logo.load(":image/home_logo")) {
        qDebug() << ERROR << "Failed to load image";
    }

    setKeyhelplabel();

    // Initialize process pointers to nullptr
    robotcon = nullptr;
    cameracon = nullptr;
    yolocon = nullptr;
    servoingcon = nullptr;
    halfSpherecon = nullptr;
    collisionDetectcon = nullptr;
    collisionDetectPlotcon = nullptr;
    packingcon = nullptr;
    palletGazebocon = nullptr;

    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString timestamp = currentDateTime.toString("yyyy-MM-dd_HH-mm-ss_t");
    logFileName = QString("ui_log_%1.txt").arg(timestamp);

    checkAndCleanRosLogs();
}

// Destructor for cleanup
PalletizingRobotUi::~PalletizingRobotUi()
{
    ROS_INFO("GUI SHUTDOWN");
    delete ui;
}

/* General config method */
void PalletizingRobotUi::checkAndCleanRosLogs() {
    QString message;
    QProcess process;

    // rosclean check 실행
    process.start("rosclean check");
    if (!process.waitForFinished(5000)) {  // 타임아웃 설정
        message = ERROR + "rosclean check timed out.";
        qDebug() << message.toUtf8().constData();
        return;
    }

    QString output = process.readAllStandardOutput();
    QRegularExpression regex("(\\d+(\\.\\d+)?)\\s*(K|G|M)");
    QRegularExpressionMatch match = regex.match(output);

    if (match.hasMatch()) {
        double logSize = match.captured(1).toDouble();
        QString unit = match.captured(3);

        if (unit == "G") {
            logSize *= 1024; // GB to MB
        } else if (unit == "K") {
            logSize /= 1024; // KB to MB
        }

        if (logSize >= 1024) { // 1GB 이상의 경우
            auto reply = QMessageBox::question(
                this, "ROS Log Cleanup",
                QString("ROS log files are taking up %1 MB. Do you want to delete them?").arg(logSize, 0, 'f', 2),
                QMessageBox::Yes | QMessageBox::No
            );

            if (reply == QMessageBox::Yes) {
                // -y 옵션 사용하여 사용자 입력 없이 삭제
                QProcess purgeProcess;
                purgeProcess.start("rosclean purge -y");
                if (!purgeProcess.waitForFinished(5000)) {  // 타임아웃 설정
                    message = ERROR + "rosclean purge timed out.";
                } else if (purgeProcess.exitStatus() == QProcess::NormalExit) {
                    message = ERROR + "ROS log files deleted.";
                } else {
                    message = ERROR + "Failed to delete ROS log files.";
                }
                qDebug() << message.toUtf8().constData();
            } else {
                message = INFO + "User chose not to delete ROS log files.";
                qDebug() << message.toUtf8().constData();
            }
        } else {
            message = QString(INFO + "ROS log size is %1 MB, no cleanup required.").arg(logSize, 0, 'f', 2);
            qDebug() << message.toUtf8().constData();
        }
    } else {
        message = ERROR + "Failed to determine ROS log size.";
        qDebug() << message.toUtf8().constData();
    }
}

void PalletizingRobotUi::setKeyhelplabel() {
    QFont font("Ubuntu",14,QFont::Bold);
    ui->label_home_keyhelp->setText("Active page - F2<br>Operate page - F3<br>Shutdown - ESC");
    ui->label_home_keyhelp->setAlignment(Qt::AlignCenter);
    ui->label_home_keyhelp->setFont(font);
    ui->label_operate_keyhelp->setText("Reactvie - F2<br>Stop system - F4<br>Home page - F1<br>Start - S,&nbsp;Stop - P");
    ui->label_operate_keyhelp->setAlignment(Qt::AlignCenter);
    ui->label_operate_keyhelp->setFont(font);
    ui->label_stop_keyhelp->setText("Reactvie - F2<br>Reoperate - F3<br>Home page - F1<br>Deactivate - Enter");
    ui->label_stop_keyhelp->setAlignment(Qt::AlignCenter);
    ui->label_stop_keyhelp->setFont(font);
    ui->label_activate_keyhelp->setText("Home - F1&nbsp;&nbsp;&nbsp;Operate - F3&nbsp;&nbsp;&nbsp;Stop system - F4<br>"
                                        "Active robot - R,&nbsp;Active camera - C,&nbsp;Active search - S,&nbsp;Active pack - P");
    ui->label_activate_keyhelp->setAlignment(Qt::AlignCenter);
    ui->label_activate_keyhelp->setFont(font);
}

bool PalletizingRobotUi::confirmAction(const QString &message, QMessageBox::Icon icon) {
    // Confirmation dialog utility function
    QMessageBox mb;
    mb.setText(message);
    mb.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    mb.setDefaultButton(QMessageBox::Ok);
    mb.setIcon(icon);
    return (mb.exec() == QMessageBox::Ok);
}

int PalletizingRobotUi::showModeSelectionDialog() {
    // Mode selection dialog for REAL or VIRTUAL mode
    QMessageBox mb;
    mb.setText("Select mode : REAL / VIRTUAL.");
    mb.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel | QMessageBox::Discard);
    mb.setDefaultButton(QMessageBox::Ok);
    mb.setIcon(QMessageBox::Icon::Question);
    mb.setButtonText(QMessageBox::Ok, "REAL");
    mb.setButtonText(QMessageBox::Cancel, "VIRTUAL");
    mb.setButtonText(QMessageBox::Discard, "Cancel");

    return mb.exec();
}

void PalletizingRobotUi::keyPressEvent(QKeyEvent *event) {
    currentWidget = ui->stackedWidget->currentWidget(); // 현재 페이지 가져오기

    switch (event->key()) {
        case Qt::Key_F1: // F1 키 눌림 - 홈 페이지로 이동
            ui->stackedWidget->setCurrentIndex(0);
            break;
        case Qt::Key_F2: // F2 키 눌림 - 활성화 페이지로 이동
            ui->stackedWidget->setCurrentIndex(3);
            break;
        case Qt::Key_F3: // F3 키 눌림 - 작업 페이지로 이동
            ui->stackedWidget->setCurrentIndex(1);
            break;
        case Qt::Key_F4: // F4 키 눌림 - 중지 페이지로 이동
            ui->stackedWidget->setCurrentIndex(2);
            break;
        case Qt::Key_Escape: // ESC 키 눌림 - GUI 종료
            if (currentWidget == ui->widget_home) {
                on_pushButton_shutdown_clicked(); // 홈 페이지에서만 종료 동작 수행
            }
            break;
        case Qt::Key_S: // S 키 눌림 - 시작
            if (currentWidget == ui->widget_operate) {
                on_pushButton_operate_start_clicked(); // 활성화 페이지에서만 시작 동작 수행
            }
            if (currentWidget == ui->widget_activate) {
                on_pushButton_activate_search_clicked(); // 활성화 페이지에서만 시작 동작 수행
            }
            break;
        case Qt::Key_P: // P 키 눌림 - 중지
            if (currentWidget == ui->widget_operate) {
                on_pushButton_operate_stop_clicked(); // 활성화 페이지에서만 중지 동작 수행
            }
            if (currentWidget == ui->widget_activate) {
                on_pushButton_activate_pack_clicked(); // 활성화 페이지에서만 시작 동작 수행
            }
            break;
        case Qt::Key_R: // R 키 눌림 - 시작
            if (currentWidget == ui->widget_activate) {
                on_pushButton_activate_robot_clicked(); // 활성화 페이지에서만 시작 동작 수행
            }
            break;
        case Qt::Key_C: // C 키 눌림 - 중지
            if (currentWidget == ui->widget_activate) {
                on_pushButton_activate_camera_clicked(); // 활성화 페이지에서만 중지 동작 수행
            }
            break;
        case Qt::Key_Enter: // Enter 키 눌림 - 비활성화
        case Qt::Key_Return: // Return 키 눌림 (Enter 키와 동일)
            if (currentWidget == ui->widget_stop) {
                on_pushButton_stop_deactivate_clicked(); // 중지 페이지에서만 비활성화 동작 수행
            }
            break;
        default:
            QWidget::keyPressEvent(event); // 기본 이벤트 처리
    }
}

void PalletizingRobotUi::movestop()
{
    // Service call to stop the robot's motion
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvmoveStop = node->serviceClient<dsr_msgs::MoveStop>("/dsr01m1013/motion/move_stop");
    dsr_msgs::MoveStop srv;
    QString text_for_append;

    srv.request.stop_mode = 3;

    if (srvmoveStop.call(srv))
    {
        text_for_append = "Successfully stopped.";
        logMessage(INFO, text_for_append, ui->plainTextEdit_operate_log);
    }
    else
    {
        text_for_append = "Failed to stop robot.";
        logMessage(ERROR, text_for_append, ui->plainTextEdit_operate_log);
        ros::shutdown();
    }
}

bool PalletizingRobotUi::movejoint(float posx[6], float vel_ , float acc_, float time_ )
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient client = node->serviceClient<dsr_msgs::MoveJoint>("/dsr01m1013/motion/move_joint");

    dsr_msgs::MoveJoint srv;
    for(int i=0; i<6; i++) srv.request.pos[i] = posx[i];
    srv.request.vel = vel_;
    srv.request.acc = acc_;
    srv.request.time = time_;
    srv.request.radius = 0;
    srv.request.mode = 0;
    srv.request.blendType = 0;
    srv.request.syncType = 0;

    if (client.call(srv))
    {
        return true;
    }
    else
    {
        logMessage(ERROR, "Failed to call service dr_control_service : move_joint\n", ui->plainTextEdit_activate_log);
        ros::shutdown();
        return false;
    }

}

void PalletizingRobotUi::movelineToHome(float posx[6], float vel_[2] , float acc_[2], float time_, float object_x, float object_y, float object_z)
{
    const float MAX_ANGLE = 180.0;
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient client = node->serviceClient<dsr_msgs::MoveLine>("/dsr01m1013/motion/move_line");
    dsr_msgs::MoveLine srv;
    QString text_for_append;

    // Adjust current pose angle if necessary
    get_current_pose(0);
    if (current_pose[5] > MAX_ANGLE) {
        current_pose[5] -= 360;
        if (!movejoint(current_pose, 90, 90, 0)) {
            logMessage(ERROR, "Failed to adjust joint angle.", ui->plainTextEdit_activate_log);
            return;
        }
    }
    if (current_pose[5] < -MAX_ANGLE) {
        current_pose[5] += 360;
        if (!movejoint(current_pose, 90, 90, 0)) {
            logMessage(ERROR, "Failed to adjust joint angle.", ui->plainTextEdit_activate_log);
            return;
        }
    }

    // Set position values
    posxG(posx, object_x, object_y, object_z);
    for (int i = 0; i < 6; i++) {
        srv.request.pos[i] = posxg[i];
    }
    for (int i = 0; i < 2; i++) {
        srv.request.vel[i] = vel_[i];
        srv.request.acc[i] = acc_[i];
    }
    srv.request.time = time_;
    srv.request.radius = 0;
    srv.request.mode = 0;
    srv.request.blendType = 0;
    srv.request.syncType = 0;

    // Log the target position
    text_for_append = QString("<pos> %1 %2 %3 %4 %5 %6")
                        .arg(srv.request.pos[0], 0, 'f', 3)
                        .arg(srv.request.pos[1], 0, 'f', 3)
                        .arg(srv.request.pos[2], 0, 'f', 3)
                        .arg(srv.request.pos[3], 0, 'f', 3)
                        .arg(srv.request.pos[4], 0, 'f', 3)
                        .arg(srv.request.pos[5], 0, 'f', 3);
    logMessage(INFO, text_for_append, ui->plainTextEdit_activate_log);

    // Call the service and check for success
    if (client.call(srv) && srv.response.success) {
        logMessage(INFO, "Successfully moved to home position.", ui->plainTextEdit_activate_log);
    } else {
        logMessage(ERROR, "Failed to call service dr_control_service : move_line", ui->plainTextEdit_activate_log);
        ros::shutdown();
    }
}

void PalletizingRobotUi::get_current_pose(int space_type) // 0은 joint 1은 space
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::ServiceClient srvGetCurrentPose = node->serviceClient<dsr_msgs::GetCurrentPose>("/dsr01m1013/system/get_current_pose");
    dsr_msgs::GetCurrentPose srv;

    srv.request.space_type = space_type;

    if(srvGetCurrentPose.call(srv))
    {
        //ROS_INFO("receive srv, srv.response.success: %ld\n", (long int)srv.response.success);
        for (int i = 0; i < 6; ++i)
        {
          //ROS_INFO("current pos[%d] = %f", i, srv.response.pos[i]);
          current_pose[i] = srv.response.pos[i];
        }

        //return (srv.response.pos);
    } else {
        logMessage(ERROR, "Failed to call service dr_control_service : get_current_pose\n", ui->plainTextEdit_activate_log);
        ros::shutdown();

    }
}

void PalletizingRobotUi::posxG(float posx[6], float object_x, float object_y, float object_z)
{
  float gripper_lenght = 230;

  float retreat[3] = {object_x, object_y, object_z + gripper_lenght};

  std::vector<std::vector<float>> unitVec(3, std::vector<float>(3));
  unitVec = rpy2vector(posx[3],posx[4],posx[5]);

  float temp_posx[6];
  for(int i =0; i<6; ++i)
  {
    temp_posx[i]= posx[i];
  }


  for(int i =0; i < 3; ++i)
  {
    for(int j =0; j <3; ++j)
    {
      temp_posx[j] = temp_posx[j] -unitVec[i][j] * retreat[i];
    }
  }

  for(int i =0; i<6; ++i)
  {
    posxg[i]= temp_posx[i];
  }
}

std::vector<float> PalletizingRobotUi::matVecMult(const std::vector<std::vector<float>>& mat, const std::vector<float>& vec) {
    std::vector<float> result(3, 0.0f);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i] += mat[i][j] * vec[j];
        }
    }
    return result;
}

// Z축 회전 행렬
std::vector<std::vector<float>> PalletizingRobotUi::rotZ(float angle) {
    float rad = DEG2RAD(angle);
    return {
        { cosf(rad), -sinf(rad), 0.0f },
        { sinf(rad),  cosf(rad), 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };
}

// Y축 회전 행렬
std::vector<std::vector<float>> PalletizingRobotUi::rotY(float angle) {
    float rad = DEG2RAD(angle);
    return {
        { cosf(rad), 0.0f, sinf(rad) },
        { 0.0f, 1.0f, 0.0f },
        { -sinf(rad), 0.0f, cosf(rad) }
    };
}

// ZYZ 회전 행렬을 얻는 함수
std::vector<std::vector<float>> PalletizingRobotUi::getZYZRotationMatrix(float yawZ1, float pitchY, float rollZ2) {
    std::vector<std::vector<float>> Rz1 = rotZ(yawZ1);
    std::vector<std::vector<float>> Ry = rotY(pitchY);
    std::vector<std::vector<float>> Rz2 = rotZ(rollZ2);

    // Rz1 * Ry * Rz2 순서로 행렬 곱
    std::vector<std::vector<float>> temp(3, std::vector<float>(3, 0.0f));
    std::vector<std::vector<float>> result(3, std::vector<float>(3, 0.0f));

    // Rz1 * Ry
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                temp[i][j] += Rz1[i][k] * Ry[k][j];
            }
        }
    }

    // (Rz1 * Ry) * Rz2
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                result[i][j] += temp[i][k] * Rz2[k][j];
            }
        }
    }

    return result;
}

std::vector<std::vector<float>> PalletizingRobotUi::rpy2vector(float yawZ1, float pitchY, float rollZ2) //rpy값을 받으면 vector를 반환 [x1,x2,x3] [y1,y2,y3] [z1,z2,z3]
{
    std::vector<std::vector<float>> rotationMatrix = getZYZRotationMatrix(yawZ1, pitchY, rollZ2);

    std::vector<std::vector<float>> unitVectors = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    std::vector<std::vector<float>> resultVec(3, std::vector<float>(3));
    for(int i =0; i< 3; ++i)
    {
        std::vector<float> transformedVec = matVecMult(rotationMatrix, unitVectors[i]);
        for(int j = 0; j < 3; ++j)
        {
            resultVec[i][j] = transformedVec[j];
        }
    }

    return resultVec;
}

void PalletizingRobotUi::wait(float time_)
{
    // Custom wait function that performs ROS spin during wait
    QEventLoop loop;
    QTimer::singleShot(static_cast<int>(time_ * 1000), &loop, &QEventLoop::quit);
    loop.exec();
}

/* Config data processing method */
void PalletizingRobotUi::loadRobotConfigAndExecute()
{
    // Load configuration and execute setup
    QString configFilePath = QCoreApplication::applicationDirPath() + "/../config/config.json";
    readConfig(configFilePath);
}

void PalletizingRobotUi::readConfig(const QString& configFilePath) {
    // Read and parse configuration JSON
    QFile file(configFilePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << ERROR + "Unable to open config file:" << file.errorString();
        return;
    }

    // Read the file and create a JSON document
    QByteArray jsonData = file.readAll();
    file.close();

    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);

    // Check if the JSON document is valid
    if (jsonDoc.isNull()) {
        qDebug() << ERROR + "Failed to parse JSON.";
        return;
    }

    // Convert JSON document to JSON object
    QJsonObject jsonObj = jsonDoc.object();

    // Read values from the JSON object
    yoloScriptPath = jsonObj["yolo_script"].toString();
    servoingScriptPath = jsonObj["servoing_script"].toString();
    collisionDetectScriptPath = jsonObj["collision_detect_script"].toString();
    collisionDetectPlotScriptPath = jsonObj["collision_detect_plot_script"].toString();
    packingAlgorithmScriptPath = jsonObj["packing_algorithm"].toString();
    packingPlotPath = jsonObj["packing_plot"].toString();
    servoingPlotPath = jsonObj["servoing_plot"].toString();
    pickPlotPath = jsonObj["pick_plot"].toString();
}

/* Log processing method */
void PalletizingRobotUi::logMessage(const QString &level, const QString &message, QPlainTextEdit *logArea) {
    // Log message utility function
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString timestamp = currentDateTime.toString("yyyy-MM-dd HH:mm:ss,zzz t");
    QString combMessage = QString("%1 %2:\n%3\n").arg(level).arg(timestamp).arg(message);
    logArea->appendPlainText(combMessage);
    qDebug() << combMessage.toUtf8().constData();

    // ROS 패키지 경로를 기반으로 logs 폴더 설정
    std::string packagePath = ros::package::getPath("palletizing_ui"); // "plantfarm_ui" 패키지 경로 가져오기
    QString logDir = QString::fromStdString(packagePath) + "/logs";  // logs 폴더 경로 설정
    QDir dir(logDir);
    if (!dir.exists()) {
        dir.mkpath("."); // 디렉토리가 없으면 생성
    }

    // 최대 허용 파일 개수
    const int maxFileCount = 10; // 최대 파일 개수 (예: 10개로 설정)

    // 파일 목록 가져오기
    QFileInfoList fileList = dir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);

    // 파일 개수가 최대 허용 개수를 초과할 경우, 오래된 파일부터 삭제
    if (fileList.size() > maxFileCount) {
        // 오래된 파일부터 삭제
        std::sort(fileList.begin(), fileList.end(), [](const QFileInfo &a, const QFileInfo &b) {
            return a.lastModified() < b.lastModified();
        });

        int filesToDelete = fileList.size() - maxFileCount;
        for (int i = 0; i < filesToDelete; ++i) {
            if (!QFile::remove(fileList[i].absoluteFilePath())) {
                qDebug() << "Failed to delete log file:" << fileList[i].absoluteFilePath();
            }
        }
    }

    QString logFilePath = logDir + "/" + logFileName;

    QFile file(logFilePath);

    // 로그 파일을 추가 모드로 열기
    if (!file.open(QIODevice::Append | QIODevice::Text)) {
        qDebug() << "Failed to open log file:" << logFilePath;
        return;
    }

    // 로그 메시지 파일에 기록
    QTextStream out(&file);
    out << combMessage << "\n";
    file.close();
}

/* QProcess pre-processing method */
void PalletizingRobotUi::handleProcessError(QProcess::ProcessError error) {
    // Error handling for processes
    QProcess *senderProcess = qobject_cast<QProcess*>(sender());
    if (senderProcess) {
        QString errorMessage = senderProcess->errorString();
        logMessage(ERROR, "Process error: " + errorMessage, ui->plainTextEdit_activate_log);
    }
}

void PalletizingRobotUi::terminateAndDeleteProcess(QProcess* &process, const QString &processName) {
    // Terminate and delete a process safely
    QString text_for_append;
    QStringList resultlog;
    text_for_append = QString("Trying to stop %1 process.").arg(processName);

    // Determine the current widget to log to the correct area
    currentWidget = ui->stackedWidget->currentWidget();
    if (currentWidget == ui->widget_operate) {
        logMessage(INFO, text_for_append, ui->plainTextEdit_operate_log);
    } else if (currentWidget == ui->widget_stop) {
        logMessage(INFO, text_for_append, ui->plainTextEdit_stop_log);
    } else if (currentWidget == ui->widget_activate) {
        logMessage(INFO, text_for_append, ui->plainTextEdit_activate_log);
    }

    // Check if the process is active
    if (process) {
        process->terminate();  // Request graceful termination

        // Loop to check for process termination within a timeout (maximum wait of 5 seconds)
        int waitTime = 5000;  // 5 seconds in milliseconds
        while (!process->waitForFinished(100) && waitTime > 0) {
            QCoreApplication::processEvents();  // Allow event handling during the wait
            waitTime -= 100;  // Decrease remaining wait time
        }

        // After waiting, check if the process has terminated
        if (process->state() != QProcess::NotRunning) {  // If still running after timeout
            process->kill();  // Force termination
            text_for_append = QString("Forcefully killed %1 process.").arg(processName);
            // Log the result of the termination attempt based on the active widget
            if (currentWidget == ui->widget_operate) {
                logMessage(WARNING, text_for_append, ui->plainTextEdit_operate_log);
            } else if (currentWidget == ui->widget_stop) {
                logMessage(WARNING, text_for_append, ui->plainTextEdit_stop_log);
            } else if (currentWidget == ui->widget_activate) {
                logMessage(WARNING, text_for_append, ui->plainTextEdit_activate_log);
            }
        } else {
            text_for_append = QString("Stopped %1 process successfully.").arg(processName);
            // Log the result of the termination attempt based on the active widget
            if (currentWidget == ui->widget_operate) {
                logMessage(INFO, text_for_append, ui->plainTextEdit_operate_log);
            } else if (currentWidget == ui->widget_stop) {
                logMessage(INFO, text_for_append, ui->plainTextEdit_stop_log);
            } else if (currentWidget == ui->widget_activate) {
                logMessage(INFO, text_for_append, ui->plainTextEdit_activate_log);
            }
        }

        // If stopping "path_planner," call movestop to halt robot movement
        if (processName == "path_planner") {
            movestop();
        }

        // // 추가: 로봇 프로세스가 종료되면 가제보도 함께 종료
        // if (processName == "DSR") {
        //     // 가제보와 로봇이 함께 실행된 경우이므로 추가로 종료 명령어를 실행
        //     QProcess::execute("killall -SIGINT gazebo");
        //     QProcess::execute("killall -SIGINT gzserver");
        //     QProcess::execute("killall -SIGINT gzclient");

        //     text_for_append = QString("Stopped Gazebo successfully.");
        //     // Log the result of the termination attempt based on the active widget
        //     logMessage(INFO, text_for_append, ui->plainTextEdit_stop_log);
        // }

        // Use deleteLater() to safely delete the process after returning to the event loop
        process->deleteLater();
        process = nullptr;  // Clear the pointer to avoid dangling references
    } else {
        // Process was not running, so log an error
        text_for_append = QString("%1 process was not running.").arg(processName);

        // Log the message in the appropriate log area based on the active widget
        if (currentWidget == ui->widget_operate) {
            logMessage(ERROR, text_for_append, ui->plainTextEdit_operate_log);
        } else if (currentWidget == ui->widget_stop) {
            logMessage(ERROR, text_for_append, ui->plainTextEdit_stop_log);
        } else if (currentWidget == ui->widget_activate) {
            logMessage(ERROR, text_for_append, ui->plainTextEdit_activate_log);
        }
    }
}

void PalletizingRobotUi::startProcess(QProcess* &process, const QString &name, const QString &program, 
                                      const QStringList &arguments, const QString &workingDir, QPlainTextEdit *logArea) {
    // Start and configure a new process
    terminateAndDeleteProcess(process, name);

    logMessage(INFO, "Trying to start " + name, logArea);
    process = new QProcess(this);

    if ( process == cameracon) connect(cameracon, &QProcess::started, this, &PalletizingRobotUi::servoing);
    if ( process == collisionDetectcon) connect(collisionDetectcon, &QProcess::started, this, &PalletizingRobotUi::collisionDetectconPlot);

    if (!workingDir.isEmpty()) {
        process->setWorkingDirectory(workingDir);
    }
    process->start(program, arguments);

    // if(name == "DSR") dsrlogoutput = process->readAllStandardOutput();
    // if(name == "Realsense") camlogoutput = process->readAllStandardOutput();
    // if(name == "Path_planner") pathplanlogoutput = process->readAllStandardOutput();

    connect(process, &QProcess::errorOccurred, this, &PalletizingRobotUi::handleProcessError);

    connect(process, &QProcess::errorOccurred, this, [=] {
        logMessage(ERROR, "Failed to start " + name + ": " + process->errorString(), logArea);
    });

    if (!process->waitForStarted()) {
        logMessage(ERROR, "Failed to start " + name + ": " + process->errorString(), logArea);
        terminateAndDeleteProcess(process, name);
    } else {
        logMessage(INFO, name + " started successfully.", logArea);
    }
}

void PalletizingRobotUi::handleYoloError()
{
    // Handle errors specific to YOLO process
    if (yolocon) {
        logMessage(ERROR, "YOLO error: " + yolocon->errorString(), ui->plainTextEdit_operate_log);
    } else {
        logMessage(ERROR, "YOLO process is null.", ui->plainTextEdit_operate_log);
    }
}

void PalletizingRobotUi::handleYoloFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    // Handle completion status of YOLO process
    if (exitStatus == QProcess::CrashExit) {
        logMessage(ERROR, "YOLO process crashed.", ui->plainTextEdit_activate_log);
    } else if (exitCode == 0) {
        logMessage(ERROR, "YOLO process finished successfully.", ui->plainTextEdit_activate_log);
    } else {
        logMessage(ERROR, "YOLO process finished with non-standard exit code: " + QString::number(exitCode), ui->plainTextEdit_activate_log);
    }
}

/* Packing Algorithm data processing method */
void PalletizingRobotUi::packingLogCallBack(const std_msgs::String::ConstPtr &msg) {
    // Callback for processing packing log messages
    QString packinglog = QString::fromStdString(msg->data);
    logMessage(INFO, packinglog, ui->plainTextEdit_operate_log);
}

void PalletizingRobotUi::updatePackPlotimage() {
    // 이미지의 마지막 수정 시간을 저장하는 변수 (정적 선언)
    static QDateTime lastModifiedTime;

    // 이미지 파일의 마지막 수정 시간 확인
    QFileInfo fileInfo(packingPlotPath);
    QDateTime currentModifiedTime = fileInfo.lastModified();

    // 파일이 변경된 경우에만 이미지를 다시 로드
    if (currentModifiedTime > lastModifiedTime) {
        lastModifiedTime = currentModifiedTime;
        if (!pixmap_Packplot.load(packingPlotPath)) {
            logMessage(ERROR, "No image found", ui->plainTextEdit_operate_log);
            qDebug() << "Failed to load plot image";
            return;
        }

        // Scaling image size to QLabel size
        int w = ui->label_operate_packing_plot->width();
        int h = ui->label_operate_packing_plot->height();
        ui->label_operate_packing_plot->setPixmap(pixmap_Packplot.scaled(w, h, Qt::KeepAspectRatio));
        logMessage(INFO, "Pack plot image updated successfully", ui->plainTextEdit_operate_log);
    }
}

void PalletizingRobotUi::updateServoingImage() {
    if (!mapPixmap.load(servoingPlotPath)) {
        logMessage(ERROR, QString("Failed to load image from path: %1").arg(servoingPlotPath), ui->plainTextEdit_operate_log);
        qDebug() << "Failed to load map server image from" << servoingPlotPath;
        return;
    }

    // Scaling image size to QLabel size
    int w = ui->label_operate_packing_plot->width();
    int h = ui->label_operate_packing_plot->height();
    ui->label_operate_packing_plot->setPixmap(mapPixmap.scaled(w, h, Qt::KeepAspectRatio));
}

void PalletizingRobotUi::updatePickImage() {
    if (!mapPixmap.load(pickPlotPath)) {
        logMessage(ERROR, QString("Failed to load image from path: %1").arg(pickPlotPath), ui->plainTextEdit_operate_log);
        // Scaling image size to QLabel size
        int w = ui->label_operate_packing_plot->width();
        int h = ui->label_operate_packing_plot->height();
        ui->label_operate_packing_plot->setPixmap(mapPixmap.scaled(w, h, Qt::KeepAspectRatio));
    }
}

/* Button event methods for changing UI pages*/
void PalletizingRobotUi::on_pushButton_activate_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
}

void PalletizingRobotUi::on_pushButton_operate_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void PalletizingRobotUi::on_pushButton_operate_stop_2_clicked()
{
   ui->stackedWidget->setCurrentIndex(2);
}

void PalletizingRobotUi::on_pushButton_operate_reactivate_clicked()
{
    ui->stackedWidget->setCurrentIndex(3);
}

void PalletizingRobotUi::on_pushButton_activate_operation_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void PalletizingRobotUi::on_pushButton_activate_stop2_clicked()
{
    ui->stackedWidget->setCurrentIndex(2);
}

void PalletizingRobotUi::on_pushButton_stop_operate_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);
}

void PalletizingRobotUi::on_pushButton_stop_activate_clicked()
{
  ui->stackedWidget->setCurrentIndex(3);
}

void PalletizingRobotUi::on_pushButton_stop_home_clicked()
{
    if (!confirmAction("Are you sure you want to return to home?", QMessageBox::Information)) {
        return;
    }

    ui->stackedWidget->setCurrentIndex(0);
}

void PalletizingRobotUi::on_pushButton_activate_home_clicked()
{
  if (!confirmAction("Are you sure you want to return to home?", QMessageBox::Information)) {
        return;
    }

    ui->stackedWidget->setCurrentIndex(0);
}

void PalletizingRobotUi::on_pushButton_operate_home_clicked()
{
  if (!confirmAction("Are you sure you want to return to home?", QMessageBox::Information)) {
        return;
    }

    ui->stackedWidget->setCurrentIndex(0);
}

/* Shutdown procedure for GUI and ROS processes */
void PalletizingRobotUi::on_pushButton_shutdown_clicked()
{
    if (!confirmAction("Are you sure you want to shutdown the system?", QMessageBox::Question)) {
        return;
    }

    logMessage(INFO, "Attempting to shutdown system.", ui->plainTextEdit_activate_log);

    // Shutdown all ROS processes
    QProcess::execute("killall -SIGINT roscore");
    QProcess::execute("killall -SIGINT roslaunch");
    QProcess::execute("killall -SIGINT rosout");

    // Wait 5 seconds for shutdown
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // GUI shutdown after process shutdown
    logMessage(INFO, "All ROS processes terminated. Shutting down GUI.", ui->plainTextEdit_activate_log);
    ROS_INFO("GUI SHUTDOWN");
    QApplication::quit();
}

/* Event handlers for activate/deactivate function */
void PalletizingRobotUi::on_pushButton_activate_robot_clicked() {
    QStringList arguments;

    int modeSelection = showModeSelectionDialog();

    // Set arguments based on mode selection
    if (modeSelection == QMessageBox::Ok) {
        arguments << "dsr_launcher" << "single_robot_gazebo.launch"
                  << "mode:=real" << "host:=192.168.137.100" << "port:=12345";
        logMessage(INFO, "Selected mode: REAL", ui->plainTextEdit_activate_log);
    } else if (modeSelection == QMessageBox::Cancel) {
        arguments << "dsr_launcher" << "single_robot_gazebo.launch";
        logMessage(INFO, "Selected mode: VIRTUAL", ui->plainTextEdit_activate_log);
    } else if (modeSelection == QMessageBox::Discard) {
        logMessage(INFO, "No mode selected, aborting.", ui->plainTextEdit_activate_log);
        return;
    }

    // Start process with error handling
    startProcess(robotcon, "DSR", "roslaunch", arguments, "", ui->plainTextEdit_activate_log);
    wait(10);

    // Wait for necessary services to be available
    if (ros::service::waitForService("/dsr01m1013/system/get_current_pose", ros::Duration(10)) &&
        ros::service::waitForService("/dsr01m1013/motion/move_line", ros::Duration(10))) {

        logMessage(INFO, "DSR services are available. Moving robot to home position.", ui->plainTextEdit_activate_log);
        movelineToHome(posx_home, velBB, accBB, 0);
    } else {
        logMessage(INFO, "DSR services are not available. Robot activation failed.", ui->plainTextEdit_activate_log);
    }
}

void PalletizingRobotUi::on_pushButton_activate_camera_clicked() {
    QStringList arguments{"realsense2_camera", "rs_camera.launch", "filters:=pointcloud", "align_depth:=true"};
    startProcess(cameracon, "Realsense", "roslaunch", arguments, "", ui->plainTextEdit_activate_log);
}

void PalletizingRobotUi::on_pushButton_activate_search_clicked()
{
    // Terminate and reset any existing YOLO process
    terminateAndDeleteProcess(yolocon, "YOLO");

    // Log attempt to activate YOLO process
    logMessage(INFO, "Trying to activate YOLO", ui->plainTextEdit_activate_log);

    yolocon = new QProcess(this);
    yolocon->setWorkingDirectory("/home/lee");

    // Connect signals for error handling, start event, and finish event

    bool connection2 = connect(yolocon, &QProcess::started, this, &PalletizingRobotUi::half_sphere);

    if (!connection2) {
        logMessage(ERROR, "Failed to connect YOLO started signal to one or more slots.", ui->plainTextEdit_activate_log);
    }

    // Start YOLO process
    yolocon->start("python3", QStringList() << "p2planetest2.py");

    // QString text_for_append = yolocon->readAllStandardOutput();
    // logMessage(INFO, text_for_append, ui->plainTextEdit_activate_log);

    connect(yolocon, &QProcess::errorOccurred, this, &PalletizingRobotUi::handleYoloError);
    connect(yolocon, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &PalletizingRobotUi::handleYoloFinished);

    // Check if YOLO process started successfully, if not log an error and terminate
    if (!yolocon->waitForStarted()) {
        logMessage(ERROR, "Failed to start YOLO.", ui->plainTextEdit_activate_log);
        terminateAndDeleteProcess(yolocon, "YOLO");
    }
}

void PalletizingRobotUi::servoing() {
    QStringList arguments{"beginner_tutorials", "servoing.py"};
    startProcess(servoingcon, "Servoing", "rosrun", arguments, "", ui->plainTextEdit_activate_log);
}

void PalletizingRobotUi::half_sphere()
{
    QStringList arguments{"beginner_tutorials", "half_sphere.py"};
    // Start collision detect process in parallel
    startProcess(halfSpherecon, "Half Sphere", "rosrun", arguments, "", ui->plainTextEdit_activate_log);
}

void PalletizingRobotUi::collisionDetectconPlot() {

    QStringList arguments{"beginner_tutorials", "collision_detect_plot.py"};
    // Start collision detect plot process
    startProcess(collisionDetectPlotcon, "Collision Detection Plot", "rosrun", arguments, "", ui->plainTextEdit_operate_log);

    // Disconnect previous connections to prevent redundant connections
    disconnect(servoingImageUpdateTimer, &QTimer::timeout, this, &PalletizingRobotUi::updateServoingImage);
    disconnect(pickImageUpdateTimer, &QTimer::timeout, this, &PalletizingRobotUi::updatePickImage);

    // Connect timer to image update slot and start timer
    connect(servoingImageUpdateTimer, &QTimer::timeout, this, &PalletizingRobotUi::updateServoingImage);
    if (!servoingImageUpdateTimer->isActive()) {
        servoingImageUpdateTimer->start(1000);
    }
    connect(pickImageUpdateTimer, &QTimer::timeout, this, &PalletizingRobotUi::updatePickImage);
    if (!pickImageUpdateTimer->isActive()) {
        pickImageUpdateTimer->start(1000);
    }
}

void PalletizingRobotUi::palletGazebo() {
    QStringList arguments{"pallet_gazebo", "pallet_gazebo.launch"};
    startProcess(palletGazebocon, "Palletizing Gazebo", "roslaunch", arguments, "", ui->plainTextEdit_operate_log);
}

void PalletizingRobotUi::on_pushButton_activate_pack_clicked()
{
    QStringList arguments{"beginner_tutorials", "Packing-service3.py"};
    // Start packing algorithm process
    startProcess(packingcon, "Packing Algorithm", "rosrun", arguments, "", ui->plainTextEdit_activate_log);

    // Disconnect previous connections to prevent redundant connections
    disconnect(packingImageUpdateTimer, &QTimer::timeout, this, &PalletizingRobotUi::updatePackPlotimage);

    // Connect timer to image update slot and start timer
    connect(packingImageUpdateTimer, &QTimer::timeout, this, &PalletizingRobotUi::updatePackPlotimage);
    if (!packingImageUpdateTimer->isActive()) {
        packingImageUpdateTimer->start(1000);
    }
}

void PalletizingRobotUi::on_pushButton_operate_start_clicked()
{
    // Start path planner process with arguments
    QStringList arguments{"beginner_tutorials", "Collision_detect.py"};
    startProcess(collisionDetectcon, "Collision_detect", "rosrun", arguments, "", ui->plainTextEdit_operate_log);
}

void PalletizingRobotUi::on_pushButton_operate_stop_clicked()
{
    // Stop path planner process if running
    terminateAndDeleteProcess(collisionDetectcon, "Collision_detect");
}

void PalletizingRobotUi::on_pushButton_stop_deactivate_clicked() {
    bool anyProcessTerminated = false; // 플래그 변수 추가

    // Shutdown DSR process
    if (ui->checkBox_stop_robot->isChecked()) {
        terminateAndDeleteProcess(robotcon, "DSR");
        anyProcessTerminated = true;
    }
    // Shutdown Realsense process
    if (ui->checkBox_stop_camera->isChecked()) {
        terminateAndDeleteProcess(cameracon, "Realsense");
        anyProcessTerminated = true;
    }
    // Shutdown Yolo process
    if (ui->checkBox_stop_yolo->isChecked()) {
        terminateAndDeleteProcess(yolocon, "Yolo");
        if(!yolocon) terminateAndDeleteProcess(servoingcon, "Servoing");
        if(!yolocon && !servoingcon) terminateAndDeleteProcess(halfSpherecon, "Half Sphere");
        if(!yolocon && !servoingcon && !halfSpherecon) terminateAndDeleteProcess(palletGazebocon, "Palletizing Gazebo");
        anyProcessTerminated = true;
    }
    // Shutdown collision detect process
    if (ui->checkBox_stop_collision_detect->isChecked()) {
        terminateAndDeleteProcess(collisionDetectcon, "Collision Detect");
        if(collisionDetectPlotcon) terminateAndDeleteProcess(collisionDetectPlotcon, "Collision Detect Plot");
        anyProcessTerminated = true;

        // When process closed, stop image update timer
        if (servoingImageUpdateTimer->isActive()) {
            servoingImageUpdateTimer->stop();
        }
        if (pickImageUpdateTimer->isActive()) {
            pickImageUpdateTimer->stop();
        }
    }
    // Shutdown Packing Algorithm process
    if (ui->checkBox_stop_packing->isChecked()) {
        terminateAndDeleteProcess(packingcon, "Packing Algorithm");
        anyProcessTerminated = true;

        // When process closed, stop image update timer
        if (packingImageUpdateTimer->isActive()) {
            packingImageUpdateTimer->stop();
        }
    }

    // 플래그 변수를 사용하여 어떤 체크박스도 선택되지 않은 경우에만 로그 메시지 출력
    if (!anyProcessTerminated) {
        logMessage(INFO, "No process selected to terminate.", ui->plainTextEdit_stop_log);
    }
}
