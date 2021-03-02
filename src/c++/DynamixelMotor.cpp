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
#define BAUDRATE                        57600 
#define DEVICENAME                      "/dev/tty.usbserial-FT4TCRQV"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
/**
 * @brief A class to easily control a single dynamixel motor
 */
class DynamixelMotor {
    private:
        uint id, baudrate;
        std::string port;

    public:
        /**
         * The port handler to the corrosponding port
         */
        dynamixel::PortHandler *portHandler;
        /**
         * The packet handler for the corrspoding baudrate
         */
        dynamixel::PacketHandler *packetHandler;
        
        /**
         * @brief Construct a new Dynamixel Motor object
         * 
         * @param id The integer id of the dynamixel
         * @param baudrate The integer baudrate of the dynamixel
         * @param port The port path as a string
         */
        DynamixelMotor(uint id, uint baudrate, std::string port) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;

            this->portHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
            this->packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
        }

        /**
         * @brief Construct a new Dynamixel Motor object with packet and port handler injection
         * 
         * @param id The integer id of the dynamixel
         * @param baudrate The integer baudrate of the dynamixel
         * @param port The port path as a string 
         * @param portHandler The dynamixel port handler pointer
         * @param packetHandler The dynamixel packet handler pointer
         */
        DynamixelMotor(uint id, uint baudrate, std::string port, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;

            this->portHandler = portHandler;
            this->packetHandler = packetHandler;
        }

        /**
         * @brief A method to set up the inital port and packet handler. No need to run this if port and packet handler pointers were already given in the constructor
         * 
         * @return true The initialization succeeded
         * @return false The initialization failed
         */
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
                this->close();
                std::cout << "Failed to set baudrate to " << this->baudrate << std::endl;
                return false;
            }

            return true;
        }

        /**
         * @brief Closes the port for the corrosponsong motor
         */
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