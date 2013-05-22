#ifndef DEVICEWORKER_H
#define DEVICEWORKER_H

#include <QObject>
#include <QVector>
#include <QMutex>
#include <QWaitCondition>
#include "scope.h"

class DeviceWorker : public QObject
{
    Q_OBJECT

public:
    explicit DeviceWorker(QObject *parent = 0);

    void setDevMutex(QMutex *dev_mutex);
    void setFd(const int *dev_fd);
    void setSettings(const scope_settings *settings);
private:
    QMutex *dev_mutex;
    const int *dev_fd;
    const scope_settings *settings;
public slots:
    void process();
signals:
    void devFail(int error);
    void dataLoad(const QVector<unsigned char> *data, bool *done,
                  QMutex *data_mutex, QWaitCondition *data_read);
};

#endif // DEVICEWORKER_H
