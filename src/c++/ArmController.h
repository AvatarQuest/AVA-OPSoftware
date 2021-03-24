#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <unordered_map>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "DynamixelMotor.h"
#include "DynamixelHelper.h"

class ArmController {
    private: 
        DynamixelHelper helper;
        double a1, a2, a3;

        double thetaToPos(double theta) {
            return (theta / 360) * 4096;
        }

    public:
        ArmController(DynamixelHelper helper, double a1, double a2, double a3) {
            this->helper = helper;
            this->a1 = a1;
            this->a2 = a2;
            this->a3 = a3;
        }

        void setAngle(MotorIdentifier id, double angle) {
            DynamixelMotor motor = helper.getByMotorIdentifier(id);
            motor.setGoal(thetaToPos(angle));
        }

        DynamixelHelper getHelper() {
            return helper;
        }

        DynamixelMotor getMotor(MotorIdentifier id) {
            return helper.getByMotorIdentifier(id);
        }

};