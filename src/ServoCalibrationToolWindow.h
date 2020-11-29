#pragma once

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class ServoCalibrationToolWindow; }
QT_END_NAMESPACE

class ServoCalibrationToolWindow : public QWidget
{
    Q_OBJECT

public:
    ServoCalibrationToolWindow(QWidget *parent = nullptr);
    ~ServoCalibrationToolWindow();

signals:
    void onSignalStrength(size_t channelIndex, double signalStrength);

private:
    Ui::ServoCalibrationToolWindow *ui;
};
