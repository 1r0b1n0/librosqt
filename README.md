# librosqt
This is a simple library that merges the event loop of ROS and Qt.

# Usage
In your main.cpp init the node in the following way :

```c++
int main(int _argc, char ** _argv)
{
    QCoreApplication app(_argc, _argv);
    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(_argc, _argv, "librosqt_simple_example", ros::init_options::NoSigintHandler);
    
//  [...]
    
    app.exec();
    return 0;
}

```

## Examples
* [Simple example](examples/simple)


# Motivations
The Qt library can be useful for ROS nodes when developping GUI applications, or even in headless nodes using network communications (tcp/ip, modbus, canbus, bluetooth ...).

Unfortunatly both Qt and ROS need to have their own event loop spinning : `ros::spin()` for ROS, `QCoreApplication::exec()` for Qt.

Multiple methods can be used to take care of both event loops :

# Alternatives
## Polling using a timer

```c++
void onTimeoutFromROS(const ros::TimerEvent&)
{
    qDebug() << "Timeout from ros timer";
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
```
[Complete source code](examples/alternatives/polling_method)

One way to easily merge the event loops is to periodically call `spinOnce()` from Qt's main thread. However this method reduces the application reactivity to the frequency of the `pollingTimer` (10 Hz in this case).

## Spawning another thread
```c++
void onTimeoutFromROS(const ros::TimerEvent&)
{
    qDebug() << "Timeout from ros in thread : " << QThread::currentThreadId()";
}

void onTimeoutFromQt()
{
    qDebug() << "Timeout from Qt in thread : " << QThread::currentThreadId()";
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
```
[Complete source code](examples/alternatives/two_threads_method)

Using this method the callbacks from Qt (`onTimeoutFromQt()`) and from ROS (`onTimeoutFromROS()`) are called without any latency.
The ROS callbacks are called from another thread, so we have to take care to protect any shared ressources with locking mechanisms like mutexes or mailboxes.
