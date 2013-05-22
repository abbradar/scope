#include <cerrno>
#include <unistd.h>
#include <sys/ioctl.h>
#include "deviceworker.h"

void DeviceWorker::setDevMutex(QMutex *dev_mutex)
{
    this->dev_mutex = dev_mutex;
}

void DeviceWorker::setFd(const int *dev_fd)
{
    this->dev_fd = dev_fd;
}

void DeviceWorker::setSettings(const scope_settings *settings)
{
    this->settings = settings;
}

DeviceWorker::DeviceWorker(QObject *parent) :
    QObject(parent), dev_mutex(NULL), dev_fd(NULL), settings(NULL)
{
}

void DeviceWorker::process()
{
    QMutex data_mutex;
    QWaitCondition data_read;
    QVector<unsigned char> data;
    bool done = true;
    while (true) {
        dev_mutex->lock();
        if (*dev_fd == -1) {
            dev_mutex->unlock();
            break;
        }
        int rv = ::ioctl(*dev_fd, SIO_TRIGGER, 0);
        if (rv) {
            dev_mutex->unlock();
            emit devFail(errno);
            break;
        }
        data_mutex.lock();
        if (!done)
            data_read.wait(&data_mutex);
        data_mutex.unlock();
        if (settings->time_div == SCOPE_TRANSIENT_TIME)
            data.resize(SCOPE_TRANSIENT_SIZE);
        else
            data.resize(SCOPE_DATA_SIZE);
        rv = ::read(*dev_fd, data.data(), data.size());
        dev_mutex->unlock();
        if (rv < 0) {
            emit devFail(errno);
            break;
        }
        Q_ASSERT(rv == data.size());

        done = false;
        emit dataLoad(&data, &done, &data_mutex, &data_read);
    }
    data_mutex.lock();
    if (!done)
        data_read.wait(&data_mutex);
    data_mutex.unlock();
}
