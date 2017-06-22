/**
    sensor_hub_node
    sensor_hub_node.cpp
    Purpose: ROS Node for initializing remote rosserial node sensors

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (19/6/17)
*/

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_hub/AttachSRF05.h>
#include <sensor_hub/AttachSRF10.h>

const std::string ROS_PREFIX = "sensor_hub_node";
const std::string VERSION_STRING = "0.3.0";

std::set<std::string> frameSet;
std::map<std::string, int> frameBucket;
int timer = 0;

/**
    Callback for handling ROS sensor messages

    @param msg sensor_msgs::Range by reference
    @return nothing
*/

void topicMonitor(const sensor_msgs::Range& msg) {
    timer = (timer + 1) % 100;
    std::string frame_id = msg.header.frame_id;

    ROS_DEBUG("Received message! %d", timer);

    // If frame is monitored, increment respective bucket
    if (frameSet.find(frame_id) != frameSet.end()) {
        // TODO: Increment bucket for real (requires check)
        if (msg.min_range <= msg.range && msg.range <= msg.max_range) {
            frameBucket[frame_id] = 1;
            ROS_DEBUG("Received valid message from: %s", frame_id.c_str());
        }
    }

    // Tell everyone who has been a BAD BOY
    if (timer == 0) {
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
        ROS_DEBUG("frameMissing: %ul", frameMissing.size());

        // Empty the frameBucket
        while (!frameBucket.empty()) {
            frameBucket.erase(frameBucket.begin());
        }
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
    ros::NodeHandle nh;

    ROS_INFO("%s: v%s", ROS_PREFIX.c_str(), VERSION_STRING.c_str());

    // Read sensor list
    std::vector<std::string> sensors;
    std::map<std::string, ros::Subscriber> subscriberMap;
    nh.getParam("sensors", sensors);
    if (sensors.size() == 0) ROS_WARN("No sensors configured");

    // Initialize each sensor by service call
    for (std::vector<std::string>::iterator sensorNameIt = sensors.begin(); sensorNameIt != sensors.end(); ++sensorNameIt) {
        std::string sensorTopic, sensorFrame, deviceType;
        nh.getParam(*sensorNameIt + "/topic", sensorTopic);
        nh.getParam(*sensorNameIt + "/transform/frame", sensorFrame);
        nh.getParam(*sensorNameIt + "/device/type", deviceType);

        if (subscriberMap.find(sensorTopic) == subscriberMap.end()) {
            subscriberMap[sensorTopic] = nh.subscribe(sensorTopic, 100, topicMonitor);
        }
        if (subscriberMap.find(sensorTopic) != subscriberMap.end()) {
            frameSet.insert(sensorFrame);
        }

        uint8_t error = 0;

        if (deviceType.compare("SRF05") == 0) {
            // Retrieve the sensor parameters for SRF05
            uint8_t pin = 0; int pinInt = 0, timeout = 20000;
            nh.getParam(*sensorNameIt + "/device/pin", pinInt);
            nh.getParam(*sensorNameIt + "/device/timeout", timeout);
            // Cast and assign service call fields
            pin = (uint8_t) pinInt;
            sensor_hub::AttachSRF05 attachSRF05;
            attachSRF05.request.pin = pin;
            attachSRF05.request.frame = sensorFrame;
            attachSRF05.request.timeout = timeout;
            // Call SRF05 initialization service
            if (ros::service::call("AttachSRF05", attachSRF05)) {
                error = attachSRF05.response.error;
            } else {
                error = 2;
            }
            if (error) {
                ROS_ERROR("Sensor %s : Error %d - on pin %d", sensorNameIt->c_str(), error, pin);
            }
        } else if (deviceType.compare("SRF10") == 0) {
            // Retrieve the sensor parameters for SRF10
            uint8_t address = 0; int addressInt = 0, timeout = 20000;
            nh.getParam(*sensorNameIt + "/device/addr", addressInt);
            nh.getParam(*sensorNameIt + "/device/timeout", timeout);
            // Cast and assign service call fields
            address = (uint8_t) addressInt;
            sensor_hub::AttachSRF10 attachSRF10;
            attachSRF10.request.addr = address;
            attachSRF10.request.frame = sensorFrame;
            attachSRF10.request.timeout = timeout;
            // Call SRF10 initialization service
            if (ros::service::call("AttachSRF10", attachSRF10)) {
                error = attachSRF10.response.error;
            } else {
                error = 2;
            }
            if (error) {
                ROS_ERROR("Sensor %s : Error %d - on address 0x%x", sensorNameIt->c_str(), error, address);
            }
        } else if (deviceType.compare("") == 0) {
            // Ignore Undefined Sensor
            ROS_INFO("Ignoring sensor %s", sensorNameIt->c_str());
        } else {
            // Unrecognized sensor type
            ROS_ERROR("Unknown sensor %s of type: %s", sensorNameIt->c_str(), deviceType.c_str());
        }

        ROS_INFO("Sensor Parameters Loaded - Topic: %s Frame: %s", sensorTopic.c_str(), sensorFrame.c_str());
    }

    ROS_INFO("Sensors added");

    // Process all event callbacks
    ros::spin();
}
