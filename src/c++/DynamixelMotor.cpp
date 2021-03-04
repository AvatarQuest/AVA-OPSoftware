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
 * @brief A structure to deal with address table values
 * 
 */
typedef struct AddressTable {
    typedef uint address;
    address TORQUE_ENABLE;
    address GOAL_POSITION;
    address PRESENT_POSITION;

    AddressTable() {};
} AddressTable;

typedef struct XM430W350T_TABLE: AddressTable {
    XM430W350T_TABLE() {
        TORQUE_ENABLE = 64;
        GOAL_POSITION = 116;
        PRESENT_POSITION = 132;
    }
} XM430W350T_TABLE;

/**
 * @brief A class to easily control a single dynamixel motor
 */
class DynamixelMotor {
    private:
        uint id, baudrate;
        std::string port;

    public:

        /**
         * The address table with memory location values
         */
        AddressTable addressTable;
        /**
         * The port handler to the corresponding port
         */
        dynamixel::PortHandler *portHandler;
        /**
         * The packet handler for the corresponding baudrate
         */
        dynamixel::PacketHandler *packetHandler;
        
        /**
         * @brief Construct a new Dynamixel Motor object
         * 
         * @param id The integer id of the dynamixel
         * @param baudrate The integer baudrate of the dynamixel
         * @param port The port path as a string
         */
        DynamixelMotor(uint id, uint baudrate, std::string port, AddressTable addressTable) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;
            this->addressTable = addressTable;

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
        DynamixelMotor(uint id, uint baudrate, std::string port, AddressTable addressTable, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
            this->id = id;
            this->baudrate = baudrate;
            this->port = port;
            this->addressTable = addressTable;

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
                std::cout << "Did you run \"sudo chmod a+rw " << this->port << "\"? Check if the port is being used by another program" << std::endl;
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
         * @brief Pings the dynamixel
         * 
         * @return true The ping succeeded
         * @return false The ping failed
         */
        bool ping() {
            uint8_t error = 0;                          
            uint16_t model_number;

            int result = this->packetHandler->ping(this->portHandler, this->id, &model_number, &error);
            bool success = this->checkCommResult(result, error);
            if (success) {
                std::cout << "[ID:" << this->id <<"] ping Succeeded" << std::endl;
                return true;
            }
            return false;
        }
        
        /**
         * @brief Reads the current position of the dynamixel
         * 
         * @return double The value in dxl units of the present posiiton of the servo
         */
        double readPosition() {
            int32_t position;
            uint8_t error = 0;

            int result = this->packetHandler->read4ByteTxRx(this->portHandler, this->id, this->addressTable.PRESENT_POSITION, (uint32_t*)&position, &error);
            bool success = this->checkCommResult(result, error);

            if (success) {
                return position;
            }
            return 0.0;
        }    

        bool checkCommResult(int result, uint8_t error) {
            if (result != COMM_SUCCESS) {
                std::cout << this->packetHandler->getTxRxResult(result) << std::endl;
                return false;
            } else if (error != 0) {
                std::cout << this->packetHandler->getRxPacketError(error) << std::endl;
                return false;
            } else {
                return true;
            }
        }

        /**
         * @brief Closes the port for the corrosponsong motor
         */
        void close() {
            this->portHandler->closePort();
        }
};

int main() {
    AddressTable table = XM430W350T_TABLE();

    DynamixelMotor motor = DynamixelMotor(DXL_ID, BAUDRATE, DEVICENAME, table);
    if (!motor.init()) {
        exit(1);
    }

    motor.ping();

    // while (true) {
        std::cout << "position: " << motor.readPosition() << std::endl;
    // }
}