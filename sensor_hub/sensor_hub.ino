/*
 * rosserial Sensor Hub

 * created 16 June 2017
 * by Eliot Lim (github: @eliotlim)
 *
 * This code is released into the MIT License.
 */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <Wire.h>
#include "UltrasoundSensor.h"
#include "SRFRangeSensor.h"

/*****************************************************************
 * ROS Runtime
 *****************************************************************/

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range("ultrasound/range", &range_msg);

/*****************************************************************
 * Ultrasonic
 *****************************************************************/

RangeSensor* ultrasounds[] = {new UltrasoundSensor(14),
                              new UltrasoundSensor(6, 7),
                              new SRFRangeSensor(0xE4),
                              new UltrasoundSensor(10, 11),
                              new UltrasoundSensor(16)};
String frames[] = {"sensor0",
                   "sensor1",
                   "sensor2",
                   "sensor3",
                   "sensor4"};
const int ULTRASOUND_COUNT = 5;

/*****************************************************************
 * Setup
 *****************************************************************/

void setup() {
    nh.initNode();

    nh.advertise(pub_range);

    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = frames[0].c_str();
    range_msg.field_of_view = 0.1;  // fake
    range_msg.min_range = 0.01;
    range_msg.max_range = 6.47;

    ultrasounds[0]->setTimeout(20000UL);
    ultrasounds[1]->setTimeout(1000UL);
    ultrasounds[2]->setTimeout(20000UL);
    ultrasounds[3]->setTimeout(1000UL);
    ultrasounds[4]->setTimeout(20000UL);

    Wire.begin();
}

/*****************************************************************
 * State Variables
 *****************************************************************/

long range_time;
int count = 0;

/*****************************************************************
 * Main Loop
 *****************************************************************/

void loop() {

    // Publish with rate-limiting
    if ( millis() >= range_time ){
        range_time =  millis() + 20;
        ultrasounds[count]->doRange();
        range_msg.range = ultrasounds[count]->getRange()/100.0f;
        range_msg.header.stamp = nh.now();
        range_msg.header.frame_id = frames[count].c_str();
        pub_range.publish(&range_msg);
        count = ++count % ULTRASOUND_COUNT;
   }

   nh.spinOnce();
}
