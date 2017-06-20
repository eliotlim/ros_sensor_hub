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
#include <sensor_hub/AttachSRF05.h>
#include <sensor_hub/AttachSRF10.h>

/*****************************************************************
 * ROS Runtime
 *****************************************************************/

ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range("sensor_hub/range", &range_msg);

const char* ROS_PREFIX = "sensor_hub_fw";
const char* VERSION_STRING = "0.2.0";

/*****************************************************************
 * Ultrasonic
 *****************************************************************/

const int SENSOR_MAX = 5;
RangeSensor* range_sensors[SENSOR_MAX];
String frames[SENSOR_MAX];
int sensor_count = 0;

/*****************************************************************
 * Service Methods
 *****************************************************************/

bool attachSRF05(sensor_hub::AttachSRF05::Request &req,
                 sensor_hub::AttachSRF05::Response &res) {
    // Bounds check on range_sensors array
    if (sensor_count >= SENSOR_MAX) { res.error = 1; return false; }
    // Setup sensors
    range_sensors[sensor_count] = new UltrasoundSensor(req.pin);
    range_sensors[sensor_count]->setTimeout(req.timeout);
    frames[sensor_count] = req.frame;
    res.error = 0;
    ++sensor_count;
    return true;
}

bool attachSRF10(sensor_hub::AttachSRF10::Request &req,
                 sensor_hub::AttachSRF10::Response &res) {
    // Bounds check on range_sensors array
    if (sensor_count >= SENSOR_MAX) { res.error = 1; return false; }
    // Setup sensors
    range_sensors[sensor_count] = new SRFRangeSensor(req.addr);
    range_sensors[sensor_count]->setTimeout(req.timeout);
    frames[sensor_count] = req.frame;
    res.error = 0;
    ++sensor_count;
    return true;
}

ros::ServiceServer<sensor_hub::AttachSRF05::Request, sensor_hub::AttachSRF05::Response> srf05Service("AttachSRF05", attachSRF05);
ros::ServiceServer<sensor_hub::AttachSRF10::Request, sensor_hub::AttachSRF10::Response> srf10Service("AttachSRF10", attachSRF10);

 /*****************************************************************
  * Setup
  *****************************************************************/

void setup() {
    Wire.begin();

    nh.initNode();

    nh.loginfo("Intializing ");
    nh.loginfo(ROS_PREFIX);
    nh.loginfo(" version ");
    nh.loginfo(VERSION_STRING);

    for (int c = 0; c < SENSOR_MAX; c++) {
        range_sensors[c] = NULL;
        frames[c] = "";
    }
    nh.loginfo("Intialized arrays");

    // Default range_msg data fields
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = "test";
    range_msg.field_of_view = 0.1; // fake
    range_msg.min_range = 0.01;
    range_msg.max_range = 6.47;
    nh.advertise(pub_range);
    nh.loginfo("Setup Publisher");

    // Advertise Services
    nh.advertiseService(srf05Service);
    nh.loginfo("Setup SRF05 Service");
    nh.advertiseService(srf10Service);
    nh.loginfo("Setup SRF10 Service");

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
    if (sensor_count != 0 && millis() >= range_time ){
        // NULL pointer check
        if (range_sensors[count] != NULL) {
            // Schedule next Ranging with interval >= 20ms
            range_time = millis() + 20;
            range_sensors[count]->doRange();
            // Populate range_msg data fields and publish
            range_msg.range = range_sensors[count]->getRange()/100.0f;
            range_msg.header.stamp = nh.now();
            range_msg.header.frame_id = frames[count].c_str();
            pub_range.publish(&range_msg);
        }
        // Select the next sensor (wrapping around SENSOR_COUNT)
        count = ++count % sensor_count;
   }

   nh.spinOnce();
}
