#include "ServoCalibrationTool.h"

#include <catix_messages/CalibrationLimitValue.h>
#include <catix_messages/CalibrationPointValue.h>
#include <catix_messages/SignalingChannelState.h>
#include <catix_messages/ServoState.h>

//---------------------------------------------------------------------------

ServoCalibrationTool::ServoCalibrationTool()
{
    window.show();
}
