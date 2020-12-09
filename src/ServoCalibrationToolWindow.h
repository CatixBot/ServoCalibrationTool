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
    void onDropAll();
    void onSignalStrength(size_t channelIndex, double signalStrength);
    void onFirstPoint(size_t servoIndex, double signalStrength, double servoAngle);
    void onSecondPoint(size_t servoIndex, double signalStrength, double servoAngle);
    void onLowerLimit(size_t servoIndex, double signalStrength);
    void onUpperLimit(size_t servoIndex, double signalStrength);
    void onServoAngle(size_t servoIndex, double servoAngle);

private:
    void connectChannelGroup();
    void connectCalibrationGroup();
    void connectServoGroup();

private:
    Ui::ServoCalibrationToolWindow *ui;
};
