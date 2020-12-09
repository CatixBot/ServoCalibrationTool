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
        void connectWindow();

    private:
        ros::NodeHandle node;
        ros::Publisher publisherSignalingDrop;
        ros::Publisher publisherCalibrationLowerLimitValue;
        ros::Publisher publisherCalibrationUpperLimitValue;
        ros::Publisher publisherCalibrationFirstPointValue;
        ros::Publisher publisherCalibrationSecondPointValue;
        ros::Publisher publisherServoState;
        ros::Publisher publisherSignalingChannel;

    private:

        ServoCalibrationToolWindow window;
};
