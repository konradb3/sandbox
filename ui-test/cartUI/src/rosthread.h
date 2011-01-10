#ifndef ROSTHREAD_H
#define ROSTHREAD_H

#include <QThread>
#include <ros/ros.h>

class ROSThread : public QThread
{
    Q_OBJECT
public:
    explicit ROSThread(QObject *parent = 0);

signals:

public slots:
protected:
    void run();
};

#endif // ROSTHREAD_H
