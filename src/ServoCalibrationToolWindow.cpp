#include "ServoCalibrationToolWindow.h"
#include "ui_servocalibrationtoolwindow.h"

double convertDegreesToRadians(double angleInDegrees)
{
    return angleInDegrees * M_PI / 180.0;
}

ServoCalibrationToolWindow::ServoCalibrationToolWindow(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ServoCalibrationToolWindow)
{
    ui->setupUi(this);

    this->connectChannelGroup();
    this->connectCalibrationGroup();
    this->connectServoGroup();
}

ServoCalibrationToolWindow::~ServoCalibrationToolWindow()
{
    delete ui;
}

void ServoCalibrationToolWindow::connectChannelGroup()
{
    QObject::connect(ui->pushButtonDropAll, &QPushButton::pressed, [&]()
    {
        ui->signalingChannelCheckBox->setChecked(false);
        ui->signalStrengthDial->setValue(0);

        emit onDropAll();
    });

    QObject::connect(ui->signalStrengthDial, &QDial::valueChanged, [&](int dialValue)
    {
        if (!ui->signalingChannelCheckBox->isChecked())
        {
            return;
        }

        const size_t channelIndex = ui->channelIndexComboBox->currentIndex();
        const double signalStrength = static_cast<double>(dialValue) / 100;
        emit onSignalStrength(channelIndex, signalStrength);
    });

    QObject::connect(ui->signalStrengthDial, &QDial::valueChanged, [&](int dialValue)
    {
        const double signalStrength = static_cast<double>(dialValue) / 100;
        this->ui->showSignalStrengthSpinBox->setValue(signalStrength);
    });

    QObject::connect(ui->signalingChannelCheckBox, &QCheckBox::toggled, [&](bool isChecked)
    {
        if (!isChecked)
        {
            return;
        }

        const size_t channelIndex = ui->channelIndexComboBox->currentIndex();
        const double signalStrength = static_cast<double>(ui->signalStrengthDial->value()) / 100;
        emit onSignalStrength(channelIndex, signalStrength);
    });
}

void ServoCalibrationToolWindow::connectCalibrationGroup()
{
    QObject::connect(ui->firstPointButton, &QPushButton::released, [&]()
    {
        const size_t servoIndex = ui->servoIndexComboBox_2->currentIndex();

        const auto signalStrength = ui->signalStrengthSpinBox->value();
        const auto servoAngle = ui->servoAngleSpinBox->value();

        emit onFirstPoint(servoIndex, signalStrength, servoAngle);
    });

    QObject::connect(ui->firstPointButton, &QPushButton::released, [&]()
    {
        ui->calibrationTab->setCurrentIndex(1);
        ui->secondPointPage->setEnabled(true);
    });

    QObject::connect(ui->secondPointButton, &QPushButton::released, [&]()
    {
        const size_t servoIndex = ui->servoIndexComboBox_2->currentIndex();

        const auto signalStrength = ui->signalStrengthSpinBox_2->value();
        const auto servoAngle = ui->servoAngleSpinBox_2->value();

        emit onSecondPoint(servoIndex, signalStrength, servoAngle);
    });

    QObject::connect(ui->secondPointButton, &QPushButton::released, [&]()
    {
        ui->calibrationTab->setCurrentIndex(2);
        ui->lowerLimitPage->setEnabled(true);
    });

    QObject::connect(ui->lowerLimitButton, &QPushButton::released, [&]()
    {
        const size_t servoIndex = ui->servoIndexComboBox_2->currentIndex();
        const auto signalStrength = ui->signalStrengthSpinBox_3->value();

        emit onLowerLimit(servoIndex, signalStrength);
    });

    QObject::connect(ui->lowerLimitButton, &QPushButton::released, [&]()
    {
        ui->calibrationTab->setCurrentIndex(3);
        ui->upperLimitPage->setEnabled(true);
    });

    QObject::connect(ui->upperLimitButton, &QPushButton::released, [&]()
    {
        const size_t servoIndex = ui->servoIndexComboBox_2->currentIndex();
        const auto signalStrength = ui->signalStrengthSpinBox_4->value();

        emit onUpperLimit(servoIndex, signalStrength);
    });

    QObject::connect(ui->upperLimitButton, &QPushButton::released, [&]()
    {
        ui->calibrationTab->setCurrentIndex(0);
        ui->secondPointPage->setEnabled(false);
        ui->lowerLimitPage->setEnabled(false);
        ui->upperLimitPage->setEnabled(false);
    });
}

void ServoCalibrationToolWindow::connectServoGroup()
{
    QObject::connect(ui->pushButtonDropAll, &QPushButton::pressed, [&]()
    {
        ui->servoAngeCheckBox->setChecked(false);
        ui->servoAngleDial->setValue(0);
    });

    QObject::connect(ui->servoAngleDial, &QDial::valueChanged, [&](int dialValue)
    {
        if (!ui->servoAngeCheckBox->isChecked())
        {
            return;
        }

        const size_t servoIndex = ui->servoIndexComboBox->currentIndex();
        const double servoAngle = convertDegreesToRadians(dialValue);
        emit onServoAngle(servoIndex, servoAngle);
    });

    QObject::connect(ui->servoAngleDial, &QDial::valueChanged, [&](int dialValue)
    {
        this->ui->showServoAngleSpinBox->setValue(dialValue);
    });

    QObject::connect(ui->servoAngeCheckBox, &QCheckBox::toggled, [&](bool isChecked)
    {
        if (!isChecked)
        {
            return;
        }

        const size_t servoIndex = ui->servoIndexComboBox->currentIndex();
        const double servoAngle = convertDegreesToRadians(ui->servoAngleDial->value());
        emit onServoAngle(servoIndex, servoAngle);
    });
}
