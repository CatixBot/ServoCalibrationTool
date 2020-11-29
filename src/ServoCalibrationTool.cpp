#include "ServoCalibrationTool.h"

#include <catix_messages/CalibrationLimitValue.h>
#include <catix_messages/CalibrationPointValue.h>
#include <catix_messages/SignalingChannelState.h>
#include <catix_messages/ServoState.h>

//---------------------------------------------------------------------------

ServoCalibrationTool::ServoCalibrationTool()
{
    ROS_INFO("Create calibration table publishers");
    this->publisherCalibrationFirstPointValue = node.advertise<catix_messages::CalibrationPointValue>("Catix/CalibrationFirstPoint", 1);
    this->publisherCalibrationSecondPointValue = node.advertise<catix_messages::CalibrationPointValue>("Catix/CalibrationSecondPoint", 1);
    this->publisherCalibrationLowerLimitValue = node.advertise<catix_messages::CalibrationLimitValue>("Catix/CalibrationLowerLimit", 1);
    this->publisherCalibrationUpperLimitValue = node.advertise<catix_messages::CalibrationLimitValue>("Catix/CalibrationUpperLimit", 1);
    
    ROS_INFO("Create signaling channel state publisher");
    this->publisherSignalingChannel = node.advertise<catix_messages::SignalingChannelState>("Catix/SignalingChannel", 1);

    ROS_INFO("Create servo state publisher");
    this->publisherServoState = node.advertise<catix_messages::ServoState>("Catix/Servo", 1);

    QObject::connect(&this->window, &ServoCalibrationToolWindow::onSignalStrength, [this](size_t signalIndex, double signalStrength)
    {
        catix_messages::SignalingChannelState channelStateMessage;
        channelStateMessage.signaling_channel_index = static_cast<uint8_t>(signalIndex);
        channelStateMessage.signal_strength_percentage = signalStrength;

        this->publisherSignalingChannel.publish(channelStateMessage);
        ROS_INFO("Signaling channel %d: [%f%%]", signalIndex, signalStrength);
    });

    window.show();
}
