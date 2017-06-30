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
#include <sensor_hub/SetupSensorHub.h>
#include <sensor_hub/AttachSRF05.h>
#include <sensor_hub/AttachSRF10.h>

/*****************************************************************
 * ROS Runtime
 *****************************************************************/

ros::NodeHandle nh;

const char* ROS_PREFIX = "sensor_hub_fw";
const char* VERSION_STRING = "0.4.2";
const int SENSOR_MAX = 20;

/*****************************************************************
 * Sensor Variables
 *****************************************************************/

int sensor_count = 0;
RangeSensor* range_sensors[SENSOR_MAX];
String frames[SENSOR_MAX];
sensor_msgs::Range range_msgs[SENSOR_MAX];
ros::Publisher range_pub("sensor_hub/range", range_msgs);

/*****************************************************************
 * State Variables
 *****************************************************************/

long range_time;
int count = 0;
bool published = false, scheduleReset = false;

/*****************************************************************
 * Service Methods
 *****************************************************************/

bool attachSRF05(sensor_hub::AttachSRF05::Request &req,
                 sensor_hub::AttachSRF05::Response &res) {
    // Bounds check on range_sensors array - return no space left on device (28)
    if (sensor_count >= SENSOR_MAX) { res.error = 28; return false; }
    // Setup sensor
    range_sensors[sensor_count] = new UltrasoundSensor(req.pin);
    range_sensors[sensor_count]->setTimeout(req.timeout);
    // Populate published fields
    frames[sensor_count] = req.frame;
    range_msgs[sensor_count].radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msgs[sensor_count].field_of_view = 0.52;
    range_msgs[sensor_count].min_range = 0.03;
    range_msgs[sensor_count].max_range = 4.0;
    // No errors encountered
    res.error = 0;
    ++sensor_count;
    return true;
}

bool attachSRF10(sensor_hub::AttachSRF10::Request &req,
                 sensor_hub::AttachSRF10::Response &res) {
    // Bounds check on range_sensors array - return no space left on device (28)
    if (sensor_count >= SENSOR_MAX) { res.error = 28; return false; }
    // Setup sensor
    range_sensors[sensor_count] = new SRFRangeSensor(req.addr);
    range_sensors[sensor_count]->setTimeout(req.timeout);
    // Populate published fields
    frames[sensor_count] = req.frame;
    range_msgs[sensor_count].radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msgs[sensor_count].field_of_view = 1.57;
    range_msgs[sensor_count].min_range = 0.05;
    range_msgs[sensor_count].max_range = 6.0;
    // No errors encountered
    res.error = 0;
    ++sensor_count;
    return true;
}

ros::ServiceServer<sensor_hub::AttachSRF05::Request, sensor_hub::AttachSRF05::Response> srf05Service("AttachSRF05", attachSRF05);
ros::ServiceServer<sensor_hub::AttachSRF10::Request, sensor_hub::AttachSRF10::Response> srf10Service("AttachSRF10", attachSRF10);

void resetHub() {
    // Reset and reinitialize global variables
    sensor_count = 0;
    for (int c = 0; c < SENSOR_MAX; c++) {
        range_sensors[c] = NULL;
        frames[c] = "";
        // Default range_msg data fields
        range_msgs[c].radiation_type = sensor_msgs::Range::ULTRASOUND;
        range_msgs[c].header.frame_id = "map";
        range_msgs[c].field_of_view = 1.57;
        range_msgs[c].min_range = 0.1;
        range_msgs[c].max_range = 5.0;
    }
}

void (*softReset)(void) = NULL;

bool setupSensorHub(sensor_hub::SetupSensorHub::Request &req,
                   sensor_hub::SetupSensorHub::Response &res) {
    // Provision for reset
    if (req.reset) { res.error = 1; scheduleReset = true; return; }
    // SetupSensorHub
    resetHub();
    // Disable ranging for one second
    range_time = millis() + 1000;

}

ros::ServiceServer<sensor_hub::SetupSensorHub::Request, sensor_hub::SetupSensorHub::Response> setupService("SetupSensorHub", setupSensorHub);

/*****************************************************************
* Setup
*****************************************************************/

void setup() {
    // Required for SRF08/10 Support
    Wire.begin();

    nh.initNode();
    resetHub();

    // Advertise Setup Service
    nh.advertiseService(setupService);

    // Advertise Sensor Services
    nh.advertiseService(srf05Service);
    nh.advertiseService(srf10Service);

    // Advertise Data Topic
    nh.advertise(range_pub);

}

/*****************************************************************
 * Main Loop
 *****************************************************************/

void loop() {
    // Reset hook
    if (scheduleReset) {
        nh.spinOnce();
        softReset();
    }

    // Publish with rate-limiting
    if (sensor_count != 0 && millis() >= range_time ){
        // NULL pointer check
        if (range_sensors[count] != NULL) {
            // Schedule next Ranging with interval >= 20ms
            range_time = millis() + 20;
            range_sensors[count]->doRange();
            // Populate range_msg data fields and publish
            range_msgs[count].range = range_sensors[count]->getRange()/100.0f;
            range_msgs[count].header.stamp = nh.now();
            range_msgs[count].header.frame_id = frames[count].c_str();
            range_pub.publish(&(range_msgs[count]));
        }
        // Select the next sensor (wrapping around SENSOR_COUNT)
        count = ++count % sensor_count;
    }

    nh.spinOnce();
}
