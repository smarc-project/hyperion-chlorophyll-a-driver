# valeport_hyperion_chlorophyll_driver

ROS 2 driver for Valeport Hyperion Chlorophyll a Fluorometer

## Overview

This project provides a ROS 2 driver for the Valeport Hyperion Chlorophyll a Fluorometer sensor, used for underwater chlorophyll a measurement. The driver allows for communication between the Hyperion Chlorophyll a sensor and a ROS 2-based system, enabling the integration of chlorophyll a data into robotic applications.

Link to sensor datasheet: [Valeport Hyperion Chlorophyll a](https://www.valeport.co.uk/products/hyperion-chlorophyll-a/)

## Installation

To install the driver, follow these steps:

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/valeport_hyperion_chlorophyll_driver.git
    ```
2. Navigate to the project directory:
    ```sh
    cd valeport_hyperion_chlorophyll_driver
    ```
3. Install dependencies:
    ```sh
    rosdep install --from-paths src --ignore-src -r -y
    ```
4. Build the package:
    ```sh
    colcon build
    ```
5. Source the setup file:
    ```sh
    source install/setup.bash
    ```

## Usage

To run the driver, use the following command:
```sh
ros2 launch valeport_hyperion_chlorophyll_driver hyperion_chlorophyll.launch.py
```

This will start the driver and begin publishing chlorophyll a data to the appropriate ROS 2 topics.

To run the chlorophyll decoder, use the following command:
```sh
ros2 run valeport_hyperion_chlorophyll_driver chlorophyll_decoder
```

This will start the decoder and begin processing the raw chlorophyll a data.

## Configuration

The driver settings can be configured using the `config.yaml` file located in the `config/` directory. Example configuration:

```yaml
port: /dev/ttyUSB0
baudrate: 115200
beam_width: 30
multibeam: true
```

## File Structure

The project directory is structured as follows:

```
valeport_hyperion_chlorophyll_driver/
├── CMakeLists.txt
├── README.md
├── package.xml
├── config/
│   └── config.yaml
├── data/
│   └── test_data.json
│   └── raw.txt
├── launch/
│   └── hyperion_chlorophyll.launch.py
├── src/
│   ├── reader/
│   │   ├── hyperion_chlorophyll_driver.py
│   │   ├── hyperion_chlorophyll_node.py
│   ├── decoder/
│   │   ├── chlorophyll_decoder.py
│   ├── tests/
│   │   ├── test_driver.py
├── msg/
│   └── ChlorophyllData.msg
│   └── test_data.json
└── docs/
    ├── datasheet.pdf
    └── user_manual.pdf
```

- `CMakeLists.txt`: The CMake build configuration file.
- `README.md`: This readme file.
- `package.xml`: The ROS 2 package configuration file.
- `config/`: Directory containing configuration files.
  - `config.yaml`: Configuration file for the driver settings.
- `launch/`: Directory containing launch files.
  - `hyperion_chlorophyll.launch.py`: Launch file to start the Hyperion Chlorophyll a driver.
- `src/`: Source code directory.
  - `serial_reader/`: Directory containing scripts needed to read data from the sensor.
    - `serial_reader_node.py`: Script for low-level communication with the Hyperion Chlorophyll a sensor.
  - `data_processor/`: Directory containing scripts needed for processing the raw data.
    - `data_processor_node.py`: Script for processing the raw data and publishing it to a new topic.
  - `tests/`: Directory containing test scripts.
    - `test_driver.py`: Test script to check that both nodes are up and running in the intended manner.
- `msg/`: Directory containing message files.
  - `ChlorophyllData.msg`: Message definition for the chlorophyll a data.
- `docs/`: Directory containing documentation files.
  - `datasheet.pdf`: Datasheet for the Valeport Hyperion Chlorophyll a sensor.
  - `user_manual.pdf`: User manual for the Valeport Hyperion Chlorophyll a sensor.
- `data/`: Directory containing data files.
  - `test_data.json`: Mock data for testing the driver.
  - `raw.txt`: Raw data from the sensor.

## Topics

The driver publishes the following ROS 2 topics:

- `/chlorophyll/raw_data`: Contains the raw chlorophyll a data from the Hyperion Chlorophyll a sensor.
- `/chlorophyll/processed_data`: Contains the processed chlorophyll a data from the Hyperion Chlorophyll a sensor.

## Services

The driver provides the following ROS 2 services:

- `/chlorophyll/reset`: Resets the Hyperion Chlorophyll a device.

## Parameters

The driver uses the following ROS 2 parameters:

- `~port`: The serial port to which the Hyperion Chlorophyll a is connected (default: `/dev/ttyUSB0`).
- `~baudrate`: The baud rate for the serial communication (default: `115200`).

## Roles of the various Python Scripts

### driver/hyperion_chlorophyll_driver.py
This script handles the low-level communication with the Valeport Hyperion Chlorophyll a sensor. It reads raw data from the sensor via a serial connection and processes it.

### driver/hyperion_chlorophyll_node.py
This script is a ROS 2 node that uses the `hyperion_chlorophyll_driver.py` to read data from the sensor and publish it to ROS 2 topics. It publishes both raw and processed data.

### decoder/chlorophyll_decoder.py
This script is a ROS 2 node that subscribes to the raw data topic published by `hyperion_chlorophyll_node.py` and processes the data into a more structured format, which it then publishes to another topic.

### tests/test_driver.py
This script is a test script that checks if the driver is up and running. It is used to verify that the driver is functioning correctly using mock data stored inside `msg/test_data.json`.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.