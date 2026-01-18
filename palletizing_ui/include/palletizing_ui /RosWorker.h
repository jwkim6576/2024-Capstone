#ifndef ROSWORKER_H
#define ROSWORKER_H

#include <QThread>
#include <ros/ros.h>

class RosWorker : public QThread {
    Q_OBJECT

public:
    explicit RosWorker(QObject *parent = nullptr);

protected:
    void run() override;
};

#endif // ROSWORKER_H
