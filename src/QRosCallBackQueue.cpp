#include <librosqt/QRosCallBackQueue.h>
#include <QMetaObject>
#include <QCoreApplication>
#include <signal.h>
#include <sys/socket.h>
#include <QDebug>
#include <ros/ros.h>

namespace ros{
    extern CallbackQueuePtr g_global_queue;
}

int QRosCallBackQueue::m_sigtermFd[2];

QRosCallBackQueue::QRosCallBackQueue()
{
    // On installe le handler du Ctrl+C via un QSocketNotifier :
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, QRosCallBackQueue::m_sigtermFd)) qFatal("Couldn't create TERM socketpair");
    m_snTerm = new QSocketNotifier(QRosCallBackQueue::m_sigtermFd[1], QSocketNotifier::Read, this);
    connect(m_snTerm, SIGNAL(activated(int)), this, SLOT(handleSigTerm()));

    struct sigaction term;
    term.sa_handler = QRosCallBackQueue::termSignalHandler;
    sigemptyset(&term.sa_mask);
    term.sa_flags |= SA_RESTART;
    if (sigaction(SIGINT, &term, 0) > 0)
        qFatal("Can't perform sigaction for INT Signal.");
    if (sigaction(SIGTERM, &term, 0) > 0)
        qFatal("Can't perform sigaction for TERM Signal.");

    connect(this, &QRosCallBackQueue::onCallback, this, &QRosCallBackQueue::processCallbacks, Qt::QueuedConnection);
}

QRosCallBackQueue::~QRosCallBackQueue()
{

}

void QRosCallBackQueue::addCallback(const ros::CallbackInterfacePtr &callback, uint64_t owner_id)
{
    ros::CallbackQueue::addCallback(callback, owner_id);
    onCallback();
}

void QRosCallBackQueue::processCallbacks()
{
    callAvailable();
}

void QRosCallBackQueue::termSignalHandler(int signal)
{
    // This method is called in the POSIX "signal" context.
    // Only reentrant methods can be called from here ! eg. any call to new or malloc could cause deadlocks

    // set flag to shutdown ros ; this is needed if for example a nodehandle is waiting for the rosmaster
    ros::requestShutdown();

    // send something on the socket to notify Qt a shutdown is requested
    char receivedSignal = static_cast<char>(signal);
    (void)::write(QRosCallBackQueue::m_sigtermFd[0], &receivedSignal, sizeof(receivedSignal));
}

void QRosCallBackQueue::replaceGlobalQueue()
{
    ros::g_global_queue.reset(new QRosCallBackQueue());
}

void QRosCallBackQueue::handleSigTerm()
{
    // we received something on the socket : this means the user wants to quit the application

    m_snTerm->setEnabled(false);
    char receivedSignal;
    (void)::read(QRosCallBackQueue::m_sigtermFd[1], &receivedSignal, sizeof(receivedSignal));

    // do Qt stuff
    QString signalStr;
    switch (receivedSignal)
    {
    case SIGINT:
        signalStr = QStringLiteral("SIGINT");
        break;
    case SIGTERM:
        signalStr = QStringLiteral("SIGTERM");
        break;
    default:
        signalStr = QString::number(receivedSignal);
        break;
    }

    qDebug() << "Received" << signalStr << "signal, properly quitting Qt eventloop...";
    QCoreApplication::quit();
    m_snTerm->setEnabled(true);
}
