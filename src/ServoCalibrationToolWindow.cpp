#include "ServoCalibrationToolWindow.h"
#include "ui_servocalibrationtoolwindow.h"

ServoCalibrationToolWindow::ServoCalibrationToolWindow(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ServoCalibrationToolWindow)
{
    ui->setupUi(this);

    QObject::connect(ui->signalStrengthDial, &QDial::valueChanged, [&](int dialValue)
    {
        if (!ui->signalingChannelCheckBox->isChecked())
        {
            return;
        }

        const size_t channelIndex = ui->channelIndexComboBox->currentIndex();
        const double signalStrength = static_cast<double>(dialValue);
        emit onSignalStrength(channelIndex, signalStrength);
    });

    QObject::connect(ui->signalingChannelCheckBox, &QCheckBox::toggled, [&](bool isChecked)
    {
        if (!isChecked)
        {
            return;
        }

        const size_t channelIndex = ui->channelIndexComboBox->currentIndex();
        const double signalStrength = static_cast<double>(ui->signalStrengthDial->value());
        emit onSignalStrength(channelIndex, signalStrength);
    });
}

ServoCalibrationToolWindow::~ServoCalibrationToolWindow()
{
    delete ui;
}
