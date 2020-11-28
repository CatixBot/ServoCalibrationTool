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

private:
    Ui::ServoCalibrationToolWindow *ui;
};
