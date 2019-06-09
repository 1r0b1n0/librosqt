#include <QCoreApplication>
#include <ros/ros.h>
#include <librosqt/QRosCallBackQueue.h>
#include "TestObject.h"

#include <std_msgs/String.h>
#include <QDebug>
#include <QThread>

void onNewMessage(std_msgs::StringConstPtr _msg)
{
    qDebug() << "Message received in thread : " << QThread::currentThreadId() << " : " << _msg->data.c_str();
}

void onTimeoutFromROS(const ros::TimerEvent& _event)
{
    qDebug() << "Timeout from ros in thread : " << QThread::currentThreadId() << " at " << ros::Time::now().toSec();
}

int main(int _argc, char ** _argv)
{
    QCoreApplication app(_argc, _argv);
    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(_argc, _argv, "librosqt_simple_example", ros::init_options::NoSigintHandler);

    ros::NodeHandle nh;
    TestObject obj;

    ros::Subscriber sub = nh.subscribe("chat", 1, onNewMessage);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), onTimeoutFromROS, false, true);
    app.exec();
    return 0;
}
