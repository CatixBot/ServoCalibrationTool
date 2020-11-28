#pragma once

#include "ServoCalibrationToolWindow.h"

#include <ros/ros.h>

#include <catix_messages/CalibrationLimitValue.h>
#include <catix_messages/CalibrationPointValue.h>
#include <catix_messages/SignalingChannelState.h>
#include <catix_messages/ServoState.h>

//------------------------------------------------------------------------

class ServoCalibrationTool 
{
    public:
        ServoCalibrationTool();

    private:
        ros::NodeHandle node;
        ros::Publisher publisherCalibrationLimitValue;
        ros::Publisher publisherCalibrationPointValue;
        ros::Publisher publisherServoState;
        ros::Publisher publisherSignalingChannel;

    private:

        ServoCalibrationToolWindow window;
};
