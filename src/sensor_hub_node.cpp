/**
    sensor_hub_node
    sensor_hub_node.cpp
    Purpose: ROS Node for initializing remote rosserial node sensors

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (19/6/17)
*/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_hub/SetupSensorHub.h>
#include <sensor_hub/AttachSRF05.h>
#include <sensor_hub/AttachSRF10.h>

const std::string ROS_PREFIX = "sensor_hub_node";
const std::string VERSION_STRING = "0.4.2";

std::set<std::string> frameSet;
std::map<std::string, int> frameBucket;

/**
    Callback for handling ROS sensor messages

    @param msg sensor_msgs::Range by reference
    @return nothing
*/

void topicMonitor(const sensor_msgs::Range& msg) {
    std::string frame_id = msg.header.frame_id;

    // If frame is monitored, increment respective bucket
    if (frameSet.find(frame_id) != frameSet.end()) {
        // TODO: Increment bucket for real (requires check)
        if (msg.min_range <= msg.range && msg.range <= msg.max_range) {
            frameBucket[frame_id] = 1;
        } else {
            ROS_DEBUG("Received invalid message from %s", frame_id.c_str());
        }
    }
}

void topicTimer(const ros::TimerEvent& event) {
    // Tell everyone who has been a BAD BOY
    std::vector<std::string> frameMissing;

    // For each monitored frame, check for missing data
    for (std::set<std::string>::iterator it = frameSet.begin(); it != frameSet.end(); it++) {
        if (frameBucket.find(*it) != frameBucket.end() && frameBucket[*it] >= 1) {
            // Has received at least <threshold> packets since last run
        } else {
            // Has not received anything
            frameMissing.push_back(*it);
        }
    }

    // For each frame with missing data, log an error
    for (std::vector<std::string>::iterator it = frameMissing.begin(); it != frameMissing.end(); it++) {
        ROS_ERROR("Missing data from sensor frame: %s", it->c_str());
    }

    // If frameMissing is Empty
    if (frameMissing.size() == 0) {
        ROS_INFO("No missing frames detected.");
    }

    // Empty the frameBucket
    while (!frameBucket.empty()) {
        frameBucket.erase(frameBucket.begin());
    }
}

/**
    Entry point for ROS Node Execution

    @param argc number of arguments
    @param argv argument list
    @return exit status error code, or zero.
*/

int main(int argc, char** argv) {
    // ros::init must be the first call in main()
    ros::init(argc, argv, ROS_PREFIX);
    ros::NodeHandle nh, nhp("~");

    ROS_INFO("%s: v%s", ROS_PREFIX.c_str(), VERSION_STRING.c_str());

    // Wait for SensorHub to be ready
    while (true) {
        ros::service::waitForService("SetupSensorHub");
        sensor_hub::SetupSensorHub setupHub;
        if (!ros::service::call("SetupSensorHub", setupHub) || setupHub.response.error) {
            // Setup Hub Failed
            // ROS_WARN("SensorHub Error");
        } else {
            ROS_INFO("Connected to SensorHub");
            break;
        }
    }

    // Read sensor list
    std::vector<std::string> sensors;
    std::map<std::string, ros::Subscriber> subscriberMap;
    nhp.getParam("sensors", sensors);
    if (sensors.size() == 0) ROS_WARN("No sensors configured");

    // Initialize each sensor by service call
    for (std::vector<std::string>::iterator sensorNameIt = sensors.begin(); sensorNameIt != sensors.end(); ++sensorNameIt) {
        std::string sensorTopic, sensorFrame, deviceType;
        nhp.getParam(*sensorNameIt + "/topic", sensorTopic);
        nhp.getParam(*sensorNameIt + "/transform/frame", sensorFrame);
        nhp.getParam(*sensorNameIt + "/device/type", deviceType);

        if (subscriberMap.find(sensorTopic) == subscriberMap.end()) {
            subscriberMap[sensorTopic] = nh.subscribe(sensorTopic, 100, topicMonitor);
        }
        if (subscriberMap.find(sensorTopic) != subscriberMap.end()) {
            frameSet.insert(sensorFrame);
        }

        errno = 0;

        if (deviceType.compare("SRF05") == 0) {
            // Retrieve the sensor parameters for SRF05
            uint8_t pin = 0; int pinInt = 0, timeout = 20000;
            nhp.getParam(*sensorNameIt + "/device/pin", pinInt);
            nhp.getParam(*sensorNameIt + "/device/timeout", timeout);
            // Cast and assign service call fields
            pin = (uint8_t) pinInt;
            sensor_hub::AttachSRF05 attachSRF05;
            attachSRF05.request.pin = pin;
            attachSRF05.request.frame = sensorFrame;
            attachSRF05.request.timeout = timeout;
            // Call SRF05 initialization service
            if (ros::service::call("AttachSRF05", attachSRF05)) {
                errno = attachSRF05.response.error;
            } else {
                errno = 38;
            }
            if (errno) {
                ROS_ERROR("Sensor %s : %s - on pin %d", sensorNameIt->c_str(), strerror(errno), pin);
            }
        } else if (deviceType.compare("SRF10") == 0) {
            // Retrieve the sensor parameters for SRF10
            uint8_t address = 0; int addressInt = 0, timeout = 20000;
            nhp.getParam(*sensorNameIt + "/device/addr", addressInt);
            nhp.getParam(*sensorNameIt + "/device/timeout", timeout);
            // Cast and assign service call fields
            address = (uint8_t) addressInt;
            sensor_hub::AttachSRF10 attachSRF10;
            attachSRF10.request.addr = address;
            attachSRF10.request.frame = sensorFrame;
            attachSRF10.request.timeout = timeout;
            // Call SRF10 initialization service
            if (ros::service::call("AttachSRF10", attachSRF10)) {
                errno = attachSRF10.response.error;
            } else {
                errno = 38;
            }
            if (errno) {
                ROS_WARN("Sensor %s : %s - on address 0x%x", sensorNameIt->c_str(), strerror(errno), address);
            }
        } else if (deviceType.compare("") == 0) {
            // Ignore Undefined Sensor
            ROS_INFO("Ignoring sensor %s", sensorNameIt->c_str());
        } else {
            // Unrecognized sensor type
            ROS_WARN("Unknown sensor %s of type: %s", sensorNameIt->c_str(), deviceType.c_str());
        }

        ROS_DEBUG("Sensor Parameters Loaded - Topic: %s Frame: %s", sensorTopic.c_str(), sensorFrame.c_str());
    }

    ROS_INFO("Sensors added");

    // Add topic monitor hook
    ros::Timer timer = nh.createTimer(ros::Duration(5), topicTimer);

    // Process all event callbacks
    ros::spin();
}
