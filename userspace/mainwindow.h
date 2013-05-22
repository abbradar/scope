#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QVector>
#include <QMap>
#include "scope.h"
#include "scopeconst.h"
#include "deviceworker.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private slots:
    void dataLoaded(const QVector<unsigned char> *data, bool *done, QMutex
                    *data_mutex, QWaitCondition *data_read);
    void devFailed(int error);
    void on_genLoadPushButton_clicked();

    void on_yposSlider_1_valueChanged(int value);

    void on_voltComboBox_1_currentIndexChanged(int index);

    void on_couplingComboBox_1_currentIndexChanged(int index);

    void on_yposSlider_2_valueChanged(int value);

    void on_voltComboBox_2_currentIndexChanged(int index);

    void on_couplingComboBox_2_currentIndexChanged(int index);

    void on_triggerSlider_valueChanged(int value);

    void on_trgDirComboBox_currentIndexChanged(int index);

    void on_trgChComboBox_currentIndexChanged(int index);

    void on_freqComboBox_currentIndexChanged(int index);

    void on_ledComboBox_currentIndexChanged(int index);

    void on_digitalCheckBox_toggled(bool checked);

    void on_trgCheckBox_toggled(bool checked);

    void on_genEnableCheckBox_toggled(bool checked);

    void on_genOffsetSlider_valueChanged(int value);

    void on_genAmplSlider_valueChanged(int value);

    void on_genCorrSlider_valueChanged(int value);

    void on_startFreqDoubleSpinBox_valueChanged(double arg1);

    void on_endFreqDoubleSpinBox_valueChanged(double arg1);

    void on_sweepDoubleSpinBox_valueChanged(double arg1);

    void on_sweepTypeComboBox_currentIndexChanged(int index);

    void on_waveComboBox_currentIndexChanged(int index);

signals:
    void startProcess();
    
private:
    void failOpen(int error, const QString &msg);
    void showGenLoadError(const QString &text);
    void updateScope();
    void updateGenerator();
    void updateFreq();
    void updateFilter();
    void devLock();

    const static int num_packets;
    Ui::MainWindow *ui;
    int dev_fd;
    QMutex dev_mutex;

    QThread *thread;
    DeviceWorker *worker;
    scope_settings scope_set;
    generator_settings gen_set;
    freq_settings freq_set;
    QVector<unsigned> time_div, volt_div;
    QVector<unsigned char> wave;
    QVector<double> wave_x, wave_y;
    QVector<double> x, y[2], yb[2];
    bool update_settings;
    filter_type f_type;
};

#endif // MAINWINDOW_H
