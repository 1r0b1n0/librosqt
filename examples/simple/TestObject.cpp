#include "TestObject.h"
#include <QThread>
#include <QDebug>

TestObject::TestObject()
{
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(sl_onTimeout()));
    m_timer.start(1000);

    qDebug() << "TestObject";
}

TestObject::~TestObject()
{
    qDebug() << "~TestObject";
}

void TestObject::sl_onTimeout()
{
    qDebug() << "Timeout from thread : " << QThread::currentThreadId();
}
