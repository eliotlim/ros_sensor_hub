/**
    sensor_hub_node
    sensor_hub_node.cpp
    Purpose: ROS Node for initializing remote rosserial node sensors

    @author Eliot Lim (github: @eliotlim)
    @version 1.0 (19/6/17)
*/

#include <ros/ros.h>
#include <sensor_hub/AttachSRF05.h>
#include <sensor_hub/AttachSRF10.h>

const std::string ROS_PREFIX = "sensor_hub_node";
const std::string VERSION_STRING = "0.1.0";

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
    nh.getParam("sensors", sensors);
    if (sensors.size() == 0) ROS_WARN("No sensors configured");

    // Initialize each sensor by service call
    for (std::vector<std::string>::iterator sensorNameIt = sensors.begin(); sensorNameIt != sensors.end(); ++sensorNameIt) {
        std::string sensorTopic, sensorType, sensorFrame;
        nh.getParam(*sensorNameIt + "/topic", sensorTopic);
        nh.getParam(*sensorNameIt + "/transform/frame", sensorFrame);
        nh.getParam(*sensorNameIt + "/device/type", sensorType);

        if (sensorType.compare("SRF05") == 0) {
            // Retrieve the sensor parameters for SRF05
            uint8_t pin = 0; int pinInt = 0;
            nh.getParam(*sensorNameIt + "/device/pin", pinInt);
            // Cast and assign service call fields
            pin = (uint8_t) pinInt;
            sensor_hub::AttachSRF05 attachSRF05;
            attachSRF05.request.pin = pin;
            attachSRF05.request.frame = sensorFrame;
            // Call SRF05 initialization service
            if (ros::service::call("AttachSRF05", attachSRF05)) {
                // Success
            } else {
                ROS_ERROR("Failed to initialize sensor: %s on pin %d, raw pin: %d", sensorFrame.c_str(), pin, pinInt);
            }
        } else if (sensorType.compare("SRF10") == 0) {
            // Retrieve the sensor parameters for SRF10
            uint8_t address = 0; int addressInt = 0;
            nh.getParam(*sensorNameIt + "/device/addr", addressInt);
            // Cast and assign service call fields
            address = (uint8_t) addressInt;
            sensor_hub::AttachSRF10 attachSRF10;
            attachSRF10.request.addr = address;
            attachSRF10.request.frame = sensorFrame;
            // Call SRF10 initialization service
            if (ros::service::call("AttachSRF10", attachSRF10)) {
                // Success
            } else {
                ROS_ERROR("Failed to initialize sensor: %s on address %x, raw address: %x", sensorFrame.c_str(), address, addressInt);
            }
        } else if (sensorType.compare("") == 0) {
            // Ignore Undefined Sensor
            ROS_INFO("Ignoring sensor: %s", sensorFrame.c_str());
        } else {
            // Unrecognized sensor type
            ROS_ERROR("Unknown sensor type: %s on frame: %s", sensorType.c_str(), sensorFrame.c_str());
        }

        ROS_INFO("Sensor Parameters Loaded - Topic: %s Frame: %s", sensorTopic.c_str(), sensorFrame.c_str());
    }

    ROS_INFO("Sensors added");

    // Process all event callbacks
    ros::spin();
}
