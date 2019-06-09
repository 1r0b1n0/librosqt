#include <QCoreApplication>
#include <ros/ros.h>
#include <QDebug>
#include <QThread>
#include <QTimer>

void onTimeoutFromROS(const ros::TimerEvent&)
{
    qDebug() << "Timeout from ros in thread : " << QThread::currentThreadId() << " at " << ros::Time::now().toSec();
}

int main(int _argc, char ** _argv)
{
    QCoreApplication app(_argc, _argv);
    ros::init(_argc, _argv, "polling_example", ros::init_options::NoSigintHandler);

    QTimer pollingTimer;
    QObject::connect(&pollingTimer, &QTimer::timeout, [](){
        ros::spinOnce();
    });
    pollingTimer.start(100); // 10 Hz

    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), onTimeoutFromROS, false, true);

    return app.exec();
}
