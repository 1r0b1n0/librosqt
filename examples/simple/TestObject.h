#ifndef TESTOBJECT_H
#define TESTOBJECT_H

#include <QObject>
#include <QTimer>

class TestObject : public QObject
{
    Q_OBJECT
public:
    TestObject();
    virtual ~TestObject();
public slots :
    void sl_onTimeout();
private :
    QTimer m_timer;
};

#endif // TESTOBJECT_H
