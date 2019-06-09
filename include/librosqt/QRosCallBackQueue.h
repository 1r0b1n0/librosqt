#ifndef QROSCALLBACKQUEUE_H
#define QROSCALLBACKQUEUE_H

#include <QObject>
#include <ros/callback_queue.h>
#include <QSocketNotifier>

class QRosCallBackQueue : public QObject, public ros::CallbackQueue{
    Q_OBJECT
public:
    QRosCallBackQueue();
    virtual ~QRosCallBackQueue();

    virtual void addCallback(const ros::CallbackInterfacePtr &callback, uint64_t owner_id);
    static void termSignalHandler(int signal);

    static void replaceGlobalQueue();
protected slots:
    void handleSigTerm();
    void processCallbacks(void);
signals:
    void onCallback();

private :
    static int m_sigtermFd[2];
    QSocketNotifier* m_snTerm;
};
#endif // QROSCALLBACKQUEUE_H
