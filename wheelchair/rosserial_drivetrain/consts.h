#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define MSB_HIGH 0b10000000

#define SPM_END 0xDA
#define PACKET_END 0x0F

// Message types
#define MSG_SR_GENERAL_INFO 0x00
#define MSG_SPM_GENERAL_INFO 0x01
#define MSG_SR_HHP 0x02
#define MSG_SPM_HHP 0x03
#define MSG_SR_POWERUP 0x04
#define MSG_SPM_POWERUP 0x05
#define MSG_SR_CALIBRATION 0x06
#define MSG_SR_FACTORYTEST 0x07
#define MSG_SACU_GENERALINFO 0x08
#define MSG_SACU_POWERUP 0x09
#define MSG_SPM_SETTINGS 0x0A
#define MSG_UNUSED_1 0x0B
#define MSG_UNUSED_2 0x0C
#define MSG_UNUSED_3 0x0D
#define MSG_UNUSED_4 0x0E
#define MSG_PACKETEND 0x0F
