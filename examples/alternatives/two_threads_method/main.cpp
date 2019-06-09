#include <QCoreApplication>
#include <ros/ros.h>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <thread>

void onTimeoutFromROS(const ros::TimerEvent&)
{
    qDebug() << "Timeout from ros in thread : " << QThread::currentThreadId() << " at " << ros::Time::now().toSec();
}

void onTimeoutFromQt()
{
    qDebug() << "Timeout from Qt in thread : " << QThread::currentThreadId() << " at " << ros::Time::now().toSec();
}

int main(int _argc, char ** _argv)
{
    QCoreApplication app(_argc, _argv);
    ros::init(_argc, _argv, "polling_example", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;

    std::thread thread_ros([](){
        ros::spin();
    });

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), onTimeoutFromROS, false, true);

    QTimer pollingTimer;
    QObject::connect(&pollingTimer, &QTimer::timeout, onTimeoutFromQt);
    pollingTimer.start(1000); // 1 Hz

    return app.exec();
}
