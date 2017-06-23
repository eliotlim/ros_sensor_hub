# sensor_hub
Dynamically configure ultrasonic range sensors on an Arduino running the sensor_hub firmware

#### sensor_hub firmware
The sensor_hub firmware implements ROS Service Calls that configure the various sensors and other parameters for the publication of data.
#### sensor_hub_node driver
The sensor_hub_node acts as a driver, initializing and configuring the sensors by calling the services exposed by the counterpart firmware.

---

## Installation
#### Firmware
1. Open the `sensor_hub.ino` sketch in the Arduino IDE
- Install the required dependencies:
  - [Rosserial](https://github.com/frankjoshua/rosserial_arduino_lib)
  - [Ultrasonic](https://github.com/eliotlim/Ultrasonic) <- Modified!
- Select the platform and amend the `SENSOR_MAX` variable
- Upload the firmware

#### ROS Node
1. Change directory your catkin workspace `cd ~/catkin_ws/src`
- Clone the sensor_hub repository into your catkin workspace `git clone https://github.com/eliotlim/sensor_hub`
- Build the package from source by performing `catkin_make`

#### ROS dependencies
- rosserial_client
- rosserial_python

## Configuration
#### Shared `sensorConfig.yaml` with sensor_pointcloud
Add a `<sensorName>/device` key to each `sensor_hub` sensor, and specify the
- type:
  - SRF05 # For SRF05/HY-SRF05/Ping)))
  - SRF10 # For SRF08/10
- connection point:
  - pin: 14 # For SRF05 use a pin number
  - addr: 0xE0 # For SRF08/10 use an 8-bit I2C address (as per datasheet)
- timeout:
  - timeout: 60000 # Wait 60 ms for ping return

```yaml
sensor0:
    topic: ultrasound/range
    device:
        type: SRF05
        pin: 14
        timeout: 60000
    transform:
        frame: sensor0
...
sensor1:
    topic: sensor_hub/range
    device:
        type: SRF10
        addr: 0xE2
        timeout: 60000
    transform:
        frame: sensor1
```
See the sensorConfig.yaml [guide](https://github.com/eliotlim/sensor_pointcloud/blob/master/sensorConfig.md) for more details

---

## Usage
#### Connecting using the rosserial node
1. Connect the device and run: `rosrun rosserial_python serial_node.py <device>`

  For Example:
  ```
  rosrun rosserial_python serial_node.py /dev/ttyACM0
  ```
- Run the sensor_hub_node `rosrun sensor_hub sensor_hub_node` to load the sensor configuration onto the Arduino

## Errors
#### Unable to find Service Provider
When sensor_hub is not connected by rosserial, the service call cannot be resolved and will fail. Check the power, data cable, serial port, and any collision with other sensor_hub_node / Arduino IDE instances.
#### Initialization Failure
If the sensor_hub has run out of slots (exceeding `SENSOR_MAX`), any sensor attachment service call will fail.
#### Invalid Sensor Data
The sensor_hub_node will monitor initialized sensors and highlight signs of sensor errors:
- Measured value exceeds defined range
- Timeout resulting in no reading


---
## Miscellaneous
## License
This library is licensed under the MIT License.
