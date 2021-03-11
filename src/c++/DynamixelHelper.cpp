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

/**
 * @brief A type to handle the location indentifiers of a motor: id, baudrate, and port
 * 
 */
typedef struct MotorIdentifier {
    uint id;
    uint baudrate;
    std::string port;

    MotorIdentifier() {}

    MotorIdentifier(int id, int baudrate, std::string port) {
        this->id = id;
        this->baudrate = baudrate;
        this->port = port;
    }

    /**
     * @brief Checks if two indetifiers point to the same motor
     * 
     * @param other The other motor
     * @return true The other identifier points to the same objects
     * @return false The other identifier does not point to the same objects
     */
    bool operator==(const MotorIdentifier& other) const { 
        return id == other.id && baudrate == other.baudrate && port == other.port; 
    } 
} MotorIdentifier;


/**
 * @brief A class to create unqiue hashes for a MotorIdentifier
 * 
 */
class MotorIndentifierHasher { 
    public: 
        /**
         * @brief The method to create a hash based on id, baudrate, and port
         * 
         * @param m The indentifier to hash
         * @return size_t The hash
         */
        size_t operator()(const MotorIdentifier& m) const
        { 
            return ((std::hash<int>()(m.id)
            ^ (std::hash<int>()(m.baudrate) << 1)) >> 1)
            ^ (std::hash<std::string>()(m.port) << 1);
        } 
};

class DynamixelHelper {
    private:
        /**
         * @brief A hash map to deal with each motor with they key as a MotorIdentifier and the value as the DynamixelMotor object
         * 
         */
        std::unordered_map<MotorIdentifier, DynamixelMotor, MotorIndentifierHasher> motors;

    public:
        /**
         * @brief Construct a new Dynamixel Helper object
         * 
         * @param dynamixel_motors A vector of dynamixel motors that need to be controlled
         */
        DynamixelHelper(std::vector<DynamixelMotor> dynamixel_motors) {
            for (DynamixelMotor &motor : dynamixel_motors) {
                MotorIdentifier identifier;
                identifier.id = motor.getId();
                identifier.baudrate = motor.getBaudrate();
                identifier.port = motor.getPort();

                motors.insert(std::make_pair(identifier, motor));
            }
        }

        void printAll() {
            for (auto& motor: motors) {
                std::cout << motor.second;
            }
        }

        DynamixelMotor selectByMotorIdentifier(MotorIdentifier identifier) {
            return motors[identifier];
        }

};

int main() {
    const std::string port = "/dev/tty.usbserial-FT4TCRQV";
    const uint baudrate = 57600;
    const AddressTable table = XM430W350T_TABLE();

    DynamixelMotor motor11 = DynamixelMotor(11, baudrate, port, table);
    DynamixelMotor motor12 = DynamixelMotor(12, baudrate, port, table);
    DynamixelMotor motor13 = DynamixelMotor(13, baudrate, port, table);
    DynamixelMotor motor14 = DynamixelMotor(14, baudrate, port, table);
    DynamixelMotor motor15 = DynamixelMotor(15, baudrate, port, table);

    std::cout << motor11 << std::endl;

    std::vector<DynamixelMotor> motors = {motor11, motor12, motor13, motor14, motor15}; 
    DynamixelHelper helper =  DynamixelHelper(motors);

    MotorIdentifier id = MotorIdentifier(12, baudrate, port);
    DynamixelMotor motor_by_id = helper.selectByMotorIdentifier(id);
    std::cout << "should be id 12: ";
    std::cout << motor_by_id << std::endl;

    // helper.printAll();
}