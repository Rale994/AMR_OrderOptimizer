# amr_ex_wts

AMR Example for Job Interview at Knapp AG

Author: Raphael Watschinger

## Installation
This project relies on some libraries that have to be installed before compilation. The libraries (and the commands to install them on a PC using Ubuntu 20.04 are)

- yaml-cpp: used to parse yaml files;   `$ sudo apt-get install libyaml-cpp-dev`
- libmosquitto: library for mqtt interface;    `$ sudo apt-get install libmosquitto-dev`)
- gtest: Google Test used for unit tests;    `$ sudo apt-get install libgtest-dev`)
- cmake: To build the project;  `$sudo apt-get install cmake`
   
Optional:
- mosquitto clients and broker: can be used to test the mqtt interface of the main application; `sudo apt install mosquitto mosquitto-clients`

Alternatively, one can use a docker container for testing and running the application. A Dockerfile (and docker-compose file) is included in the project.

## Build
The project can be built using cmake. To build the project please follow the steps below starting at the root directory of the project.
- `$ mkdir -p build`
- `$ cd build`
- `$ cmake ..`
- `$ make`

## Run
Two executables are built in the build directory:
- `RunAmrTests`: Executes all unit tests. No input arguments required or expected.
- `OrderOptimizer`: The main application. It takes one path `data_dir` to the data directory as input argument. It includes an MQTT client that subscribes to several topics (see *Assumptions* below for a list of topics), and executes operations based on received messages. (See Hints for Testing below)

## Assumptions
The following assumptions were made:
- The MQTT client connects to `localhost` at port `1883`.
- The MQTT client subscribes to the following topics:
  - `/AmrUnit/currentPosition`
  - `/AmrUnit/nextOrder`
  - `/AmrUnit/shutdown`
- Received messages for all topics are strings in yaml format:
  - Topic `/AmrUnit/currentPosition`: Message `{x: <x>, y: <y>, yaw: <yaw>}`
  - Topic `/AmrUnit/nextOrder`: Message `{order_id: <id>, description: <string>}`
  - Topic `/AmrUnit/shutdown`: Message arbitrary
- The directory specified by the user contains the subdirectories `configuration` and `orders`. The files contained in these subdirectories are assumed to be those provided with the candidate evaluation task (i.e. `orders` contains five yaml files named `orders_20201201.yaml` - `orders_20201205.yaml` and `configuration` a single file called `products.yaml`).
- It is assumed that the number of different product part locations is small. In fact, the computation of the shortest path relies on a simple brute force solution with non-polynomial run-time.

## Features
- Received messages are stored internally in a queue. The corresponding tasks are processed in the order in which they were received.
- When receiving an order via the `nextOrder` topic the application executes the desired steps (parse orders, determine shortest path, print). Messages received via the other 2 topics are handled as follows:
  - Topic `/AmrUnit/currentPosition`: The current position of the AMR Unit is changed and a message is printed to console.
  - Topic `/AmrUnit/shutdown`: The application terminates after finishing the remaining tasks in its queue.

## Hints for Testing
- Messages can be sent to the MQTT client of the main program by using the command: `mosquitto_pub -h localhost -t <topic> -m <message>`
- In case of connection errors in the main application, calling `$ sudo service mosquitto start` or `$ sudo systemctl mosquitto start` to start the mosquitto broker might help (if the client which is marked as optional above is installed).
