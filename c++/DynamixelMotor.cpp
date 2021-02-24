#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <iostream>
#include <string>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          11                   // Dynamixel ID: 1
#define BAUDRATE                        1000000 
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

class DynamixelMotor {
    private:
        uint id, baudrate;
        std::string port;

    public:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        DynamixelMotor(int id, int baudrate, std::string port) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;

            this->portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
            this->packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
        }

        DynamixelMotor(int id, int baudrate, std::string port, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;

            this->portHandler = portHandler;
            this->packetHandler = packetHandler;
        }

        bool init() {
            if (this->portHandler->openPort()) {
                std::cout << "Opened port " << this->port << " succesfully" << std::endl;
            } else {
                std::cout << "Failed to open port " << this->port << std::endl;
                std::cout << "Did you run \"sudo chmod a+rw " << this->port << "\"?" << std::endl;
                return false;
            }
 
            if (this->portHandler->setBaudRate(this->baudrate)) {
                std::cout << "Set baudrate to " << this->baudrate << " succesfully" << std::endl;
            } else {
                std::cout << "Failed to set baudrate to " << this->baudrate << std::endl;
                return false;
            }

            return true;
        }

        void close() {
            this->portHandler->closePort();
        }
};

int main() {
    DynamixelMotor motor = DynamixelMotor(DXL_ID, BAUDRATE, DEVICENAME);
    if (!motor.init()) {
        exit(1);
    } 
}