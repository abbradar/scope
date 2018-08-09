#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QtGlobal>
#include "mainwindow.h"
#include "ui_mainwindow.h"

const int MainWindow::num_packets = 2;

struct ChannelControls {
    QComboBox *voltDiv;
    QComboBox *coupling;
    QAbstractSlider *yPos;
};

int coupling_to_int(const coupling_type t) {
    switch (t) {
        case SCOPE_COUPLING_AC: return 0;
        case SCOPE_COUPLING_DC: return 1;
        case SCOPE_COUPLING_GND: return 2;
        default: Q_ASSERT(false);
    }
    return -1;
}

int trg_dir_to_int(const trigger_dir t) {
    switch (t) {
        case TRIGGER_UP: return 0;
        case TRIGGER_DOWN: return 1;
        default: Q_ASSERT(false);
    }
    return -1;
}

int power_led_to_int(const power_led t) {
    switch (t) {
        case POWER_LED_OFF: return 0;
        case POWER_LED_DIM: return 1;
        case POWER_LED_BRIGHT: return 2;
        default: Q_ASSERT(false);
    }
    return -1;
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    update_settings(false),
    f_type(OTHER_WAVE)
{
    ui->setupUi(this);

    QString fname = QFileDialog::getOpenFileName(this, "Open device file - make sure you have read permissions", "/dev", "All files (*)");
    if (fname.isNull()) exit(EXIT_SUCCESS);
    this->dev_fd = ::open(fname.toUtf8().constData(), O_RDONLY);
    if (this->dev_fd < 0) {
        failOpen(errno, "Error opening file.");
        exit(EXIT_FAILURE);
    }
    size_t size;
    int rv = ::ioctl(dev_fd, SIO_TIMEDIV_SIZE, &size);
    if (rv) {
        failOpen(errno, "Error while querying the size of the frequency table.");
        exit(EXIT_FAILURE);
    }
    time_div.resize(size);
    rv = ::ioctl(dev_fd, SIO_TIMEDIV, time_div.data());
    if (rv) {
        failOpen(errno, "Error reading the frequency table.");
        exit(EXIT_FAILURE);
    }
    rv = ::ioctl(dev_fd, SIO_VOLTDIV_SIZE, &size);
    if (rv) {
        failOpen(errno, "Error while requesting the size of the permission table.");
        exit(EXIT_FAILURE);
    }
    volt_div.resize(size);
    rv = ::ioctl(dev_fd, SIO_VOLTDIV, volt_div.data());
    if (rv) {
        failOpen(errno, "Error reading the permission table.");
        exit(EXIT_FAILURE);
    }
    wave.resize(GENERATOR_WAVE_SIZE);
    rv = ::ioctl(dev_fd, SIO_WAVEFORM_GET, wave.data());
    if (rv) {
        failOpen(errno, "Error reading current waveform.");
        exit(EXIT_FAILURE);
    }
    rv = ::ioctl(dev_fd, SIO_SCOPE_GET, &scope_set);
    if (rv) {
        failOpen(errno, "Error reading the oscilloscope settings.");
        exit(EXIT_FAILURE);
    }
    rv = ::ioctl(dev_fd, SIO_GENERATOR_GET, &gen_set);
    if (rv) {
        failOpen(errno, "Error reading generator settings.");
        exit(EXIT_FAILURE);
    }
    rv = ::ioctl(dev_fd, SIO_FREQ_GET, &freq_set);
    if (rv) {
        failOpen(errno, "Error reading the generator frequency settings.");
        exit(EXIT_FAILURE);
    }

    for (size_t ch = 0; ch < 2; ++ch)
        ui->graphWidget->addGraph();
    ui->graphWidget->xAxis->setLabel("time");
    ui->graphWidget->yAxis->setLabel("level");
    size = ((scope_set.time_div == SCOPE_TRANSIENT_TIME) ?
                SCOPE_TRANSIENT_SIZE : SCOPE_DATA_SIZE) / 2;
    ui->graphWidget->xAxis->setRange(0, size * num_packets);
    ui->graphWidget->yAxis->setRange(0, qMax(scope_set.channel[0].volt_div,
                                     scope_set.channel[1].volt_div));
    ui->graphWidget->graph(0)->setPen(QPen(Qt::blue));
    ui->graphWidget->graph(1)->setPen(QPen(Qt::red));
    ui->graphWidget->replot();
    wave_x.resize(GENERATOR_WAVE_SIZE);
    for (size_t i = 0; i < GENERATOR_WAVE_SIZE; ++i) wave_x[i] = i;
    ui->genGraphWidget->addGraph();
    ui->genGraphWidget->xAxis->setLabel("time");
    ui->genGraphWidget->yAxis->setLabel("level");
    ui->genGraphWidget->xAxis->setRange(0, GENERATOR_WAVE_SIZE);
    ui->genGraphWidget->yAxis->setRange(0, 256);
    wave_y.resize(GENERATOR_WAVE_SIZE);
    for (size_t i = 0; i < GENERATOR_WAVE_SIZE; ++i)
        wave_y[i] = wave[i];
    ui->genGraphWidget->graph(0)->setData(wave_x, wave_y);
    ui->genGraphWidget->replot();
    QStringList voltDivStr;
    voltDivStr.reserve(volt_div.size());
    foreach (const int &a, volt_div) {
        voltDivStr.append(QString::number(a));
    }
    QStringList timeDivStr;
    timeDivStr.reserve(time_div.size());
    foreach (const int &a, time_div) {
        timeDivStr.append(QString::number((double)a / 1000));
    }
    timeDivStr.append("Unsteady");
    ui->voltComboBox_1->addItems(voltDivStr);
    ui->voltComboBox_1->setCurrentIndex(volt_div.indexOf(scope_set.channel[0].volt_div));
    ui->couplingComboBox_1->setCurrentIndex(coupling_to_int(scope_set.channel[0].coupling));
    ui->yposSlider_1->setValue(scope_set.channel[0].y_pos);
    ui->voltComboBox_2->addItems(voltDivStr);
    ui->voltComboBox_2->setCurrentIndex(volt_div.indexOf(scope_set.channel[1].volt_div));
    ui->couplingComboBox_2->setCurrentIndex(coupling_to_int(scope_set.channel[1].coupling));
    ui->yposSlider_2->setValue(scope_set.channel[1].y_pos);
    ui->triggerSlider->setValue(scope_set.trigg_level);
    ui->freqComboBox->addItems(timeDivStr);
    if (scope_set.time_div == SCOPE_TRANSIENT_TIME)
        ui->freqComboBox->setCurrentIndex(timeDivStr.size() - 1);
    else
        ui->freqComboBox->setCurrentIndex(time_div.indexOf(scope_set.time_div));
    ui->trgCheckBox->setChecked(scope_set.enable_trg);
    ui->trgChComboBox->setCurrentIndex(scope_set.trg_ch);
    ui->trgDirComboBox->setCurrentIndex(trg_dir_to_int(scope_set.trg_dir));
    ui->digitalCheckBox->setChecked(scope_set.digi);
    
    ui->genOffsetSlider->setValue(gen_set.dc_offset);
    ui->genAmplSlider->setValue(gen_set.ampl);
    ui->genCorrSlider->setValue(gen_set.correction);
    ui->ledComboBox->setCurrentIndex(power_led_to_int(gen_set.power_led));
    ui->genEnableCheckBox->setChecked(gen_set.enable);

    ui->startFreqDoubleSpinBox->setValue((double)freq_set.start_freq / 1000);
    ui->endFreqDoubleSpinBox->setValue((double)freq_set.end_freq / 1000);
    ui->sweepDoubleSpinBox->setValue((double)freq_set.sweep_time / 10000);
    ui->sweepTypeComboBox->setCurrentIndex(freq_set.log_sweep ? 1 : 0);

    ui->waveComboBox->setCurrentIndex(f_type);

    thread = new QThread(this);
    worker = new DeviceWorker();
    worker->setDevMutex(&dev_mutex);
    worker->setFd(&dev_fd);
    worker->setSettings(&scope_set);
    QObject::connect(worker, SIGNAL(dataLoad(const QVector<unsigned char>*,bool*,QMutex*,QWaitCondition*)),
                     this, SLOT(dataLoaded(const QVector<unsigned char>*,bool*,QMutex*,QWaitCondition*)));
    QObject::connect(worker, SIGNAL(devFail(int)), this, SLOT(devFailed(int)));
    QObject::connect(this, SIGNAL(startProcess()), worker, SLOT(process()));
    worker->moveToThread(thread);
    thread->start();
    emit startProcess();
    update_settings = true;
}

MainWindow::~MainWindow()
{
    devLock();
    ::close(this->dev_fd);
    this->dev_fd = -1;
    dev_mutex.unlock();
    delete thread;
    delete worker;
    delete ui;
}

void MainWindow::dataLoaded(const QVector<unsigned char> *data, bool *done,
                            QMutex* data_mutex, QWaitCondition *data_read)
{
    size_t size = ((scope_set.time_div == SCOPE_TRANSIENT_TIME) ?
                SCOPE_TRANSIENT_SIZE : SCOPE_DATA_SIZE) / 2;
    size_t max_size = size * num_packets;
    x.reserve(max_size);
    for (size_t ch = 0; ch < 2; ++ch) {
        y[ch].reserve(max_size);
        yb[ch].reserve(max_size);
    }
    size_t contained = x.size();
    size_t remained = max_size - contained;
    if (remained < size) {
        for (size_t ch = 0; ch < 2; ++ch)
            yb[ch].resize(max_size);
        size_t last = max_size - size;
        for (size_t ch = 0; ch < 2; ++ch)
            for (size_t i = 0; i < size; ++i)
                yb[ch][i + last] = ((int)(*data)[2 * i + 1 - ch] *
                        scope_set.channel[ch].volt_div) / (double)256;
        data_mutex->lock();
        *done = true;
        data_mutex->unlock();
        data_read->wakeAll();
        x.resize(max_size);
        for (size_t ch = 0; ch < 2; ++ch)
            memcpy(yb[ch].data(), y[ch].data() + contained - last, last * sizeof(double));
        for (size_t i = contained; i < max_size; ++i)
            x[i] = i;
        for (size_t ch = 0; ch < 2; ++ch)
            yb[ch].swap(y[ch]);
    } else {
        size_t new_size = contained + size;
        for (size_t ch = 0; ch < 2; ++ch)
            y[ch].resize(new_size);
        for (size_t ch = 0; ch < 2; ++ch)
            for (size_t i = 0; i < size; ++i)
                y[ch][i + contained] = ((int)(*data)[2 * i + 1 - ch] *
                        scope_set.channel[ch].volt_div) / (double)256;
        data_mutex->lock();
        *done = true;
        data_mutex->unlock();
        data_read->wakeAll();
        x.resize(new_size);
        for (size_t i = contained; i < new_size; ++i)
            x[i] = i;
    }
    for (size_t ch = 0; ch < 2; ++ch)
        ui->graphWidget->graph(ch)->setData(x, y[ch]);
    ui->graphWidget->replot();
}

void MainWindow::devFailed(int error)
{
    failOpen(error, "Error while working with the device.");
    QApplication::exit(EXIT_FAILURE);
}

void MainWindow::failOpen(int error, const QString &msg)
{
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Device error");
    msgBox.setText(msg);
    msgBox.setInformativeText(strerror(error));
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.exec();
}

void MainWindow::showGenLoadError(const QString &text) {
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Read error");
    msgBox.setText("Wave file read error");
    msgBox.setInformativeText(text);
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.exec();
}

void MainWindow::updateScope()
{
    devLock();
    int rv = ::ioctl(dev_fd, SIO_SCOPE_SET, &scope_set);
    dev_mutex.unlock();
    if (rv) {
        failOpen(errno, "Error setting oscilloscope parameters");
        QApplication::exit(EXIT_FAILURE);
    }
    size_t size = ((scope_set.time_div == SCOPE_TRANSIENT_TIME) ?
                SCOPE_TRANSIENT_SIZE : SCOPE_DATA_SIZE) / 2;
    ui->graphWidget->xAxis->setRange(0, size * num_packets);
    ui->graphWidget->yAxis->setRange(0, qMax(scope_set.channel[0].volt_div,
                                     scope_set.channel[1].volt_div));
    for (size_t ch = 0; ch < 2; ++ch)
        y[ch].resize(0);
    x.resize(0);
    ui->graphWidget->replot();
}

void MainWindow::updateGenerator()
{
    devLock();
    int rv = ::ioctl(dev_fd, SIO_GENERATOR_SET, &gen_set);
    dev_mutex.unlock();
    if (rv) {
        failOpen(errno, "Error setting generator parameters");
        QApplication::exit(EXIT_FAILURE);
    }
}

void MainWindow::updateFreq()
{
    devLock();
    int rv = ::ioctl(dev_fd, SIO_FREQ_SET, &freq_set);
    dev_mutex.unlock();
    if (rv) {
        failOpen(errno, "Error setting oscillator frequency parameters");
        QApplication::exit(EXIT_FAILURE);
    }
}

void MainWindow::on_genLoadPushButton_clicked()
{
    QString fname = QFileDialog::getOpenFileName(this, "Open wave", "",
                                                 "Waves (* .gen)");

    if (fname.isNull()) return;
    QFile file(fname);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        showGenLoadError(strerror(file.error()));
        return;
    }
    QTextStream in(&file);
    QStringList strs;
    while (!in.atEnd()) {
        strs += in.readLine().split(",");
    }
    if (strs.size() != GENERATOR_WAVE_SIZE) {
        showGenLoadError("Invalid format or amount of data.");
        return;
    }
    QVector<unsigned char> wave;
    wave.resize(GENERATOR_WAVE_SIZE);
    for (size_t i = 0; i < GENERATOR_WAVE_SIZE; ++i) {
        bool ok;
        int t = strs[i].toInt(&ok);
        if (!ok || t < 0 || t > 255) {
            showGenLoadError(QString("Error at position %1: invalid number").arg(i));
            return;
        }
        wave[i] = t;
    }
    this->wave = wave;
    for (size_t i = 0; i < GENERATOR_WAVE_SIZE; ++i)
        wave_y[i] = wave[i];
    ui->genGraphWidget->graph(0)->setData(wave_x, wave_y);
    ui->genGraphWidget->replot();
    devLock();
    int rv = ::ioctl(this->dev_fd, SIO_WAVEFORM_SET, wave.data());
    dev_mutex.unlock();
    if (rv) {
        failOpen(rv, "Error while setting the wave.");
        QApplication::exit(EXIT_FAILURE);
    }
}

void MainWindow::devLock()
{
    while (!dev_mutex.tryLock(1)) {
        QCoreApplication::processEvents();
    }
}

void MainWindow::updateFilter()
{
    const filter_row *row;
    for (row = kFilterTable; row->v; ++row) {
        if (row->k == f_type) break;
    }
    Q_ASSERT(row->v);
    const uint32_char *i;
    for (i = row->v; i->k; ++i) {
        if (i->k >= freq_set.end_freq) break;
    }
    if (!i->k) {
        --i;
        freq_set.start_freq = i->k;
        ui->endFreqDoubleSpinBox->setValue((double)i->k / 1000);
        // we will get called from there
        return;
    }
    if (gen_set.filter != i->v) {
        gen_set.filter = i->v;
        updateGenerator();
    }
}

coupling_type int_to_coupling(const int type)
{
    switch (type) {
        case 0: return SCOPE_COUPLING_AC;
        case 1: return SCOPE_COUPLING_DC;
        case 2: return SCOPE_COUPLING_GND;
        default: Q_ASSERT(false);
    }
    return (coupling_type)-1;
}

trigger_dir int_to_trigger(const int type)
{
    switch (type) {
        case 0: return TRIGGER_UP;
        case 1: return TRIGGER_DOWN;
        default: Q_ASSERT(false);
    }
    return (trigger_dir)-1;
}

power_led power_led_to_trigger(const int type)
{
    switch (type) {
        case 0: return POWER_LED_OFF;
        case 1: return POWER_LED_DIM;
        case 2: return POWER_LED_BRIGHT;
        default: Q_ASSERT(false);
    }
    return (power_led)-1;
}

void MainWindow::on_yposSlider_1_valueChanged(int value)
{
    if (scope_set.channel[0].y_pos == value || !update_settings) return;
    scope_set.channel[0].y_pos = value;
    updateScope();
}

void MainWindow::on_voltComboBox_1_currentIndexChanged(int index)
{
    if (scope_set.channel[0].volt_div == volt_div[index] || !update_settings) return;
    scope_set.channel[0].volt_div = volt_div[index];
    updateScope();
}

void MainWindow::on_couplingComboBox_1_currentIndexChanged(int index)
{
    if (scope_set.channel[0].coupling == int_to_coupling(index) || !update_settings) return;
    scope_set.channel[0].coupling = int_to_coupling(index);
    updateScope();
}

void MainWindow::on_yposSlider_2_valueChanged(int value)
{
    if (scope_set.channel[1].y_pos == value || !update_settings) return;
    scope_set.channel[1].y_pos = value;
    updateScope();
}

void MainWindow::on_voltComboBox_2_currentIndexChanged(int index)
{
    if (scope_set.channel[1].volt_div == volt_div[index] || !update_settings) return;
    scope_set.channel[1].volt_div = volt_div[index];
    updateScope();
}

void MainWindow::on_couplingComboBox_2_currentIndexChanged(int index)
{
    if (scope_set.channel[1].coupling == int_to_coupling(index) || !update_settings) return;
    scope_set.channel[1].coupling = int_to_coupling(index);
    updateScope();
}

void MainWindow::on_triggerSlider_valueChanged(int value)
{
    if (scope_set.trigg_level == value || !update_settings) return;
    scope_set.trigg_level = value;
    updateScope();
}

void MainWindow::on_trgDirComboBox_currentIndexChanged(int index)
{
    if (scope_set.trg_dir == int_to_trigger(index) || !update_settings) return;
    scope_set.trg_dir = int_to_trigger(index);
    updateScope();
}

void MainWindow::on_trgChComboBox_currentIndexChanged(int index)
{
    if (scope_set.trg_ch == index || !update_settings) return;
    scope_set.trg_ch = index;
    updateScope();
}

void MainWindow::on_freqComboBox_currentIndexChanged(int index)
{
    if (!update_settings) return;
    if (index == time_div.size()) {
        if (scope_set.time_div == SCOPE_TRANSIENT_TIME) return;
        scope_set.time_div = SCOPE_TRANSIENT_TIME;
        updateScope();
    } else {
        if (scope_set.time_div == time_div[index]) return;
        scope_set.time_div = time_div[index];
        updateScope();
    }
}

void MainWindow::on_ledComboBox_currentIndexChanged(int index)
{
    if (gen_set.power_led == power_led_to_trigger(index) || !update_settings) return;
    gen_set.power_led = power_led_to_trigger(index);
    updateGenerator();
}

void MainWindow::on_digitalCheckBox_toggled(bool checked)
{
    if (scope_set.digi == checked || !update_settings) return;
    scope_set.digi = checked;
    updateScope();
}

void MainWindow::on_trgCheckBox_toggled(bool checked)
{
    if (scope_set.enable_trg == checked || !update_settings) return;
    scope_set.enable_trg = checked;
    updateScope();
}

void MainWindow::on_genEnableCheckBox_toggled(bool checked)
{
    if (gen_set.enable == checked || !update_settings) return;
    gen_set.enable = checked;
    updateGenerator();
}

void MainWindow::on_genOffsetSlider_valueChanged(int value)
{
    if (gen_set.dc_offset == value || !update_settings) return;
    gen_set.dc_offset = value;
    updateGenerator();
}

void MainWindow::on_genAmplSlider_valueChanged(int value)
{
    if (gen_set.ampl == value || !update_settings) return;
    gen_set.ampl = value;
    updateGenerator();
}

void MainWindow::on_genCorrSlider_valueChanged(int value)
{
    if (gen_set.correction == value || !update_settings) return;
    gen_set.correction = value;
    updateGenerator();
}

void MainWindow::on_startFreqDoubleSpinBox_valueChanged(double arg1)
{
    if (freq_set.start_freq == arg1 * 1000 || !update_settings) return;
    freq_set.start_freq = arg1 * 1000;
    updateFreq();
}

void MainWindow::on_endFreqDoubleSpinBox_valueChanged(double arg1)
{
    if (freq_set.end_freq == arg1 * 1000 || !update_settings) return;
    freq_set.end_freq = arg1 * 1000;
    updateFilter();
    updateFreq();
}

void MainWindow::on_sweepDoubleSpinBox_valueChanged(double arg1)
{
    if (freq_set.sweep_time == arg1 * 10000 || !update_settings) return;
    freq_set.sweep_time = arg1 * 10000;
    updateFreq();
}

void MainWindow::on_sweepTypeComboBox_currentIndexChanged(int index)
{
    if (freq_set.log_sweep == (index > 0) || !update_settings) return;
    freq_set.log_sweep = index > 0;
    updateFreq();
}

void MainWindow::on_waveComboBox_currentIndexChanged(int index)
{
    if (f_type == (filter_type)index || !update_settings) return;
    f_type = (filter_type)index;
    updateFilter();
}
