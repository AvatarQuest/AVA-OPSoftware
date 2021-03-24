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
    const std::string port = "/dev/tty.usbserial-FT4TCRQV";
    const uint baudrate = 57600;
    const AddressTable table = XM430W350T_TABLE();
    std::unordered_map<MotorIdentifier, AddressTable, MotorIndentifierHasher> motors;

    motors.insert(std::make_pair(MotorIdentifier(12, baudrate, port), table));
    motors.insert(std::make_pair(MotorIdentifier(13, baudrate, port), table));
    motors.insert(std::make_pair(MotorIdentifier(14, baudrate, port), table));
    motors.insert(std::make_pair(MotorIdentifier(15, baudrate, port), table));
    // DynamixelMotor motor11 = DynamixelMotor(11, baudrate, port, table);
    // DynamixelMotor motor12 = DynamixelMotor(12, baudrate, port, table);
    // DynamixelMotor motor13 = DynamixelMotor(13, baudrate, port, table);
    // DynamixelMotor motor14 = DynamixelMotor(14, baudrate, port, table);
    // DynamixelMotor motor15 = DynamixelMotor(15, baudrate, port, table);

    // std::cout << motor11 << std::endl;

    // std::vector<DynamixelMotor> motors = {motor11, motor12, motor13, motor14, motor15}; 

    DynamixelHelper helper =  DynamixelHelper(motors);
    MotorIdentifier id14 = MotorIdentifier(14, baudrate, port);
    MotorIdentifier id13 = MotorIdentifier(13, baudrate, port);
    MotorIdentifier id12 = MotorIdentifier(12, baudrate, port);

    DynamixelMotor motor14 = helper.getByMotorIdentifier(id14);
    DynamixelMotor motor13 = helper.getByMotorIdentifier(id13);
    DynamixelMotor motor12 = helper.getByMotorIdentifier(id12);

    motor12.setTorque(true);
    motor13.setTorque(true);
    motor14.setTorque(true);

    motor12.setGoal(2635);
    motor14.setGoal(920);
    // helper.writePositionAsync(id12, 2635, 10);
    // helper.writePositionAsync(id14, 920, 10);

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    
    // std::cout << motorid12 << std::endl;

    // helper.printAll();
}