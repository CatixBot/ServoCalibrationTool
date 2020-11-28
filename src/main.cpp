#include "ServoCalibrationTool.h"

#include <QApplication>

int main(int argc, char** argv) 
{
    QApplication app(argc, argv);

    ros::init(argc, argv, "ServoCalibrationTool");
    if (!ros::master::check())
    {
        std::cerr << "ROS Master is not running" << std::endl;
        return 1;
    }

    ServoCalibrationTool servoCalibrationTool;

	return app.exec();
}

