#include "ServoCalibrationToolWindow.h"
#include "ui_servocalibrationtoolwindow.h"

ServoCalibrationToolWindow::ServoCalibrationToolWindow(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ServoCalibrationToolWindow)
{
    ui->setupUi(this);
}

ServoCalibrationToolWindow::~ServoCalibrationToolWindow()
{
    delete ui;
}
