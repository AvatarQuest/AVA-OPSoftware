#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "dynamixel_sdk.h"  
#include "ArmController.h"

int main() {
    const double a1 = 13;
    const double a2 = 12.3;
    const double a3 = 8.5;
    double offsets[] = {0, -14.25, 14.25, 0, 0};

    const std::string port = "/dev/tty.usbserial-FT4TCRQV";
    const uint baudrate = 57600;
    const AddressTable table = XM430W350T_TABLE();
    std::unordered_map<MotorIdentifier, AddressTable, MotorIndentifierHasher> motors;

    MotorIdentifier rotation_motor = MotorIdentifier(11, baudrate, port);
    MotorIdentifier shoulder =  MotorIdentifier(12, baudrate, port);
    MotorIdentifier elbow = MotorIdentifier(13, baudrate, port);
    MotorIdentifier wrist = MotorIdentifier(14, baudrate, port);
    MotorIdentifier claw = MotorIdentifier(15, baudrate, port);
    
    ArmController controller = ArmController(rotation_motor, shoulder, elbow, wrist, claw, table, a1, a2, a3);
    controller.setOffsets(offsets);
    controller.setDebug(true);

    controller.setAllTorque(true);
    // controller.setWristAngle(300);
    controller.moveArm(0, 0);
//160
//225
    // motor12.setGoal(2635);
    // motor14.setGoal(920);
    // helper.writePositionAsync(id12, 2635, 10);
    // helper.writePositionAsync(id14, 920, 10);

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    
    // std::cout << motorid12 << std::endl;

    // helper.printAll();
}