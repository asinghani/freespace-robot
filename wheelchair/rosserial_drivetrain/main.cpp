#include "mbed.h"
#include "consts.h"

#include <ros.h>
#include <std_msgs/Float32.h>

Serial can(PA_11, PA_12);

void sendSACUGeneralInfo();
int getPacketType(int byte);
void initConnection();
void sendStartup();
void sendData(float speed, float direction);

float _lastSpeed = 0.0;
float _lastSteer = 0.0;

void speedCb(const std_msgs::Float32& msg){ _lastSpeed = msg.data; }
void steerCb(const std_msgs::Float32& msg){ _lastSteer = msg.data; }

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32> speedSub("/wc_controller/speed", &speedCb);
ros::Subscriber<std_msgs::Float32> steerSub("/wc_controller/steer", &steerCb);

int main() {
    nh.initNode();
    nh.subscribe(speedSub);
    nh.subscribe(steerSub);

    can.baud(38400);
    can.format (8, SerialBase::None, 2);
    can.set_blocking(true);

    int lastChar = 0;

    initConnection();

    bool s = false;
    int t = 0;

    int currentPacketType = -1;
    
    while(1) {
        int c = can.getc();

        // Save packet type
        int packetType = getPacketType(c);
        if(packetType != MSG_PACKETEND && packetType != -1) {
            currentPacketType = packetType;
        }

        // TODO: add parsing of SPM general info such as faults and battery (potentially add digital voltage meter to battery?)
        
        // If SPM general info ended, send SACU general info
        if(c == PACKET_END && currentPacketType == MSG_SPM_GENERAL_INFO) {
            sendSACUGeneralInfo();
        }
        
        nh.spinOnce();
    }
}

void sendSACUGeneralInfo() {
    sendData(_lastSpeed, _lastSteer);
}

int getPacketType(int byte) {
    if((byte & MSB_HIGH) == MSB_HIGH) {
        return -1; // Not a header packet
    }

    return byte & 0b1111;
}

// Setup connection (wait for SPM and SR to power up)
void initConnection() {
    int lastByte = 0;
    while(1) {
        int byte = can.getc();

        // SPM_END = Checksum of SPM power-up info
        if(lastByte == SPM_END && byte == PACKET_END) {
            sendStartup();
            break;
        }
        lastByte = byte;
    }
}

/**
 * Send general information packet. Speed in range (-1.0, 1.0). Direction in range (-1.0, 1.0).
 */
void sendData(float speed, float direction) {
    // Convert data to proper format
    int speed10bit = (int) ((speed * 512.0) + 512.0);
    speed10bit = constrain(speed10bit, 1, 1023);

    int direction10bit = (int) ((direction * 512.0) + 512.0);
    direction10bit = constrain(direction10bit, 1, 1023);

    int speedPot8bit = 128; // 128 / 256 = 50%

    int speedMSB = (speed10bit & 0b1111111000) >> 3;
    int speedLSB = speed10bit & 0b111;

    int directionMSB = (direction10bit & 0b1111111000) >> 3;
    int directionLSB = direction10bit & 0b111;

    int speedPotMSB = (speedPot8bit & 0b11111110) >> 1;
    int speedPotLSB = (speedPot8bit & 0b1);

    int status = 0b0000100; // Drive mode = SACU drive
    int opMode = 0x00; // Operating mode = Drive mode

    int header = 0x58;                                  // Packet length 6, Packet type 08 (SACU General Information)
    int byte0 = speedMSB | MSB_HIGH;                    // Speed reading 7 MSBs
    int byte1 = directionMSB | MSB_HIGH;                // Direction reading 7 MSBs
    int byte2 = speedPotMSB | MSB_HIGH;                 // Speed pot reading 7 MSBs
    int byte3 = (speedPotLSB << 6) | (speedLSB << 3) |
        (directionLSB) | MSB_HIGH;                      // Speed pot, speed, direction LSBs
    int byte4 = status | MSB_HIGH;                      // Status flags
    int byte5 = opMode | MSB_HIGH;                      // Operation mode

    // Checksum byte
    int checksum = (0x7F - ((header + byte0 + byte1 + byte2 + byte3 + byte4 + byte5) & 0b1111111)) | MSB_HIGH;
    
    can.putc(header);
    can.putc(byte0);
    can.putc(byte1);
    can.putc(byte2);
    can.putc(byte3);
    can.putc(byte4);
    can.putc(byte5);
    can.putc(checksum);

    can.putc(PACKET_END);
    can.fsync();
}

void sendStartup() {
    can.putc(0x69); // Packet length 7, Packet type 09 (SACU power-up information)

    // Manufacture info + serial number
    can.putc(0x8A);
    can.putc(0x89);
    can.putc(0xDE);
    can.putc(0xAD);
    can.putc(0xC5);
    
    // Software version number
    can.putc(0xA5);

    can.putc(0x81); // Supports speed control

    // Checksum (hardcoded)
    can.putc(0x8D);

    // End packet
    can.putc(PACKET_END);
    can.fsync();
}

