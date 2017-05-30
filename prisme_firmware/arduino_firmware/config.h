/* Comment this line in order to use the Bluetooth Serial */
#define USE_USBCON
/* Unomment in order to use associated sensor */
#define USE_IR_FRONT
#define USE_IR_UNDER
#define USE_LC
//#define USE_IMU
//#define USE_MOT
//#define USE_ENC       // TODO: not implemented

#define SERIAL_BAUD     57600

/* Publishing rates in Hz */
#define IR_PUB_RATE     5
#define LC_PUB_RATE     2
#define IMU_PUB_RATE    2
#define MOT_RATE        2 // TODO: not used yet

/* ROS Tf parameters */
#define NAMESPACE       "/prisme/"
#define BASE_LINK       "/base"

/* IR Range sensors */
#define NB_IR_FRONT     4
#define IR_FRONT_PINS   uint8_t pins_ir_front[] = {0,1,2,3}; // left to right
#define IR_FRONT_NAMES  const char * const  names_ir_front[NB_IR_FRONT] PROGMEM = { \
    NAMESPACE "ir_fl", \
    NAMESPACE "ir_flc", \
    NAMESPACE "ir_frc", \
    NAMESPACE "ir_fr"};
#define IR_FOV          10      // [rad]
#define IR_MIN_R        0       // [m]
#define IR_MAX_R        0.01    // [m]

/* IR as light sensors */
#define NB_IR_UNDER     2
#define IR_UNDER_PINS   uint8_t pins_ir_under[] = {4,5}; // left to right
#define IR_UNDER_NAMES  const char * const  names_ir_under[NB_IR_FRONT] PROGMEM = { \
    NAMESPACE "ir_ul", \
    NAMESPACE "ir_ur" };

/* Linear camera */
#define LC_NAME         const char name_lc[] PROGMEM = { NAMESPACE "lc" };

/* IMU MPU-6050 */
#define IMU_NAME        const char name_imu[] PROGMEM = { NAMESPACE "imu" };
#define IMU_ADDR        0x68
#define IMU_WAKEUP      0x6B
#define IMU_READ        0x3B
#define ACC_NORMALIZER  16348   // 2^14
#define GYR_NORMALIZER  131     // ?

/* Joint speed controller */
#define MAX_SPEED       2       // [m.s-1]
#define MOT_NAME        const char name_mot[] PROGMEM = { NAMESPACE "mot" };

