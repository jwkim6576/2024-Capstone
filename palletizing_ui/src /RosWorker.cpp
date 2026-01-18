#include "RosWorker.h"

// RosWorker 생성자 정의
RosWorker::RosWorker(QObject *parent) : QThread(parent) {}

// run 메서드 정의
void RosWorker::run() {
    // ROS 메시지 처리 루프 실행
    while (ros::ok()) {
        ros::spinOnce(); // ROS 콜백 처리
        QThread::msleep(10); // CPU 사용률 낮추기 위해 대기
    }
}
