/* Arduino ROS node for the PRisme Simulation environment.
 * Configuration of sensors and communication to be performed in config.h
 * NOTE: it is necessary to change the buffer size in roslib/ros.h from 512 to 280 :
 * --> append `|| defined(__AVR_ATmega32U4__)` to `#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__)` */


#include "config.h"
#include <ros.h>
#include <ros/time.h>
#include <prismino.h>

#ifndef USE_USBCON
#include <Bluetooth.h>
#endif

#ifdef USE_IR_FRONT
#include <sensor_msgs/Range.h>
#endif

#ifdef USE_IR_UNDER
#include <sensor_msgs/Illuminance.h>
#endif

#ifdef USE_LC
#include <LinearCamera.h>
#include <sensor_msgs/Image.h>
#endif

#ifdef USE_IMU
#include <Wire.h>
#include <sensor_msgs/Imu.h>
#endif

#ifdef USE_MOT
#include <std_msgs/Float32MultiArray.h>
#endif


ros::NodeHandle nh;
ros::Time stamp;
unsigned long timer_ir, timer_lc, timer_imu;
int8_t i;
const char base_link[] PROGMEM = { BASE_LINK };

#ifdef USE_IR_FRONT
IR_FRONT_PINS
IR_FRONT_NAMES
sensor_msgs::Range msg_ir_front[NB_IR_FRONT];
ros::Publisher* pub_ir_front[NB_IR_FRONT];
#endif

#ifdef USE_IR_UNDER
IR_UNDER_PINS
IR_UNDER_NAMES
sensor_msgs::Illuminance msg_ir_under[NB_IR_UNDER];
ros::Publisher* pub_ir_under[NB_IR_UNDER];
#endif

#ifdef USE_LC
LC_NAME
LinearCamera lc = LinearCamera(5);
uint8_t *lc_data = NULL;
sensor_msgs::Image msg_lc;
ros::Publisher pub_lc(name_lc, &msg_lc);
#endif

#ifdef USE_IMU
IMU_NAME
int16_t raw_value;
sensor_msgs::Imu msg_imu;
ros::Publisher pub_imu(name_imu, &msg_imu);
#endif

#ifdef USE_MOT
void motor_cb( const std_msgs::Float32MultiArray& cmd)
{
    int v_r, v_l;
    v_l = 100*(cmd.data[0]/MAX_SPEED);
    v_r = 100*(cmd.data[1]/MAX_SPEED);

    if(v_l > 100)
        v_l = 100;
    if(v_l < -100)
        v_l = -100;
    if(v_r > 100)
        v_r = 100;
    if(v_r < -100)
        v_r = -100;

    setSpeed(v_l, v_r);
}
MOT_NAME
ros::Subscriber<std_msgs::Float32MultiArray> sub(name_mot, motor_cb); // TODO: this type uses dynamic allocation...
#endif

void setup()
{
    nh.initNode();

#ifdef USE_USBCON
    Serial.begin(SERIAL_BAUD);
#else
    Bluetooth.begin(SERIAL_BAUD);
#endif


#ifdef USE_IR_FRONT
    for(i=0; i<NB_IR_FRONT; i++) {
        pub_ir_front[i] = new ros::Publisher(names_ir_front[i], &msg_ir_front[i]);
        nh.advertise(*pub_ir_front[i]);
        msg_ir_front[i].header.frame_id =  base_link;
        msg_ir_front[i].radiation_type = sensor_msgs::Range::INFRARED;
        msg_ir_front[i].field_of_view = IR_FOV;
        msg_ir_front[i].min_range = IR_MIN_R;
        msg_ir_front[i].max_range = IR_MAX_R;
    }
#endif

#ifdef USE_IR_UNDER
    for(i=0; i<NB_IR_UNDER; i++) {
        pub_ir_under[i] = new ros::Publisher(names_ir_under[i], &msg_ir_under[i]);
        nh.advertise(*pub_ir_under[i]);
        msg_ir_under[i].header.frame_id = base_link;
        msg_ir_under[i].variance = 0;
    }
#endif

#ifdef USE_LC
    nh.advertise(pub_lc);
    msg_lc.header.frame_id = base_link;
    msg_lc.height = 1;
    msg_lc.width = LinearCamera::PIXELS;
    msg_lc.encoding= "mono8"; // TODO: best would be to use F(), but can't cast to <const char*>
    msg_lc.step = LinearCamera::PIXELS*8; // TODO: not sure here
#endif

#ifdef USE_IMU
    nh.advertise(pub_imu);
    msg_imu.header.frame_id = base_link;
    for(i=0; i < 9; i++)
        msg_imu.orientation_covariance[i] = -1; // no magnetometer in the 6050

    Wire.begin();
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(IMU_WAKEUP); Wire.write(0); // Wake up IMU
    Wire.endTransmission(true);
#endif

    timer_ir = 0;
    timer_lc = 0;
    timer_imu = 0;
}

void loop()
{
#if defined(USE_IR_FRONT) || defined(USE_IR_UNDER)
    if( (millis() - timer_ir) > (unsigned)(1000./IR_PUB_RATE) ) {
        stamp = nh.now();
#ifdef USE_IR_FRONT
        for(i=0; i<NB_IR_FRONT; i++) {
            // from 0-1023 to MIN-MAX (m)
            msg_ir_front[i].range = IR_MIN_R + analogRead(pins_ir_front[i])*(IR_MAX_R - IR_MIN_R)/1023.;
            msg_ir_front[i].header.stamp = stamp;
            pub_ir_front[i]->publish(&msg_ir_front[i]);
        }
#endif
#ifdef USE_IR_UNDER
        for(i=0; i<NB_IR_UNDER; i++) {
            msg_ir_under[i].illuminance = analogRead(pins_ir_under[i])/4; // from 1023 to 255
            msg_ir_under[i].header.stamp = stamp;
            pub_ir_under[i]->publish(&msg_ir_under[i]);
        }
#endif
        timer_ir =  millis();
    }
#endif

#ifdef USE_LC
    if( (millis() - timer_lc) > (unsigned)(1000./LC_PUB_RATE) ) {
        lc_data = lc.getPixels();
        msg_lc.data = lc_data;
        msg_lc.header.stamp = nh.now();
        pub_lc.publish(&msg_lc);
        timer_lc =  millis();
    }
#endif

#ifdef USE_IMU
    if( (millis() - timer_imu) > (unsigned)(1000./IMU_PUB_RATE) ) {
        Wire.beginTransmission(IMU_ADDR);
        Wire.write(IMU_READ);  // start with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(IMU_ADDR, 14, true); // request a total of 14 registers

        // Note: unrolling the loop manually avoids the need for additional variables
        raw_value = Wire.read() << 8 | Wire.read();
        msg_imu.linear_acceleration.x = raw_value/(float)ACC_NORMALIZER;
        raw_value = Wire.read() << 8 | Wire.read();
        msg_imu.linear_acceleration.y = raw_value/(float)ACC_NORMALIZER;
        raw_value = Wire.read() << 8 | Wire.read();
        msg_imu.linear_acceleration.z = raw_value/(float)ACC_NORMALIZER;

        Wire.read(); Wire.read(); // discard temperature values

        raw_value = Wire.read() << 8 | Wire.read();
        msg_imu.angular_velocity.x = raw_value/(float)GYR_NORMALIZER;
        raw_value = Wire.read() << 8 | Wire.read();
        msg_imu.angular_velocity.y = raw_value/(float)GYR_NORMALIZER;
        raw_value = Wire.read() << 8 | Wire.read();
        msg_imu.angular_velocity.z = raw_value/(float)GYR_NORMALIZER;

        msg_imu.header.stamp = nh.now();
        pub_imu.publish(&msg_imu);
        timer_imu = millis();
    }
#endif

    nh.spinOnce();
}
