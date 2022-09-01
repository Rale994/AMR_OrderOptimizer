#include "amr_interface.hpp"

#include <yaml-cpp/yaml.h>

#include "amr_task_executors.hpp"

namespace AMR {
MqttInterface::MqttInterface(const std::string host, const int port,
                             const std::string client_id,
                             AMR::TaskQueue *const task_queue) {
  _mosquitto_client =
      mosquitto_new(client_id.data(), true, static_cast<void *>(task_queue));
  if (!_mosquitto_client) {
    std::cerr << "Error in MqttInterface: Out of memory" << std::endl;
    return;
  }
  mosquitto_connect_callback_set(_mosquitto_client, AMR::mqttConnectCallback);
  mosquitto_message_callback_set(_mosquitto_client, AMR::mqttMessageCallback);
  mosquitto_subscribe_callback_set(_mosquitto_client,
                                   AMR::mqttSubscribeCallback);

  if (mosquitto_connect(_mosquitto_client, host.data(), port, _keep_alive)) {
    std::cerr << "Error in MqttInterface: Unable to connect." << std::endl;
  }
}

MqttInterface::~MqttInterface() { mosquitto_destroy(_mosquitto_client); }

void MqttInterface::run() { mosquitto_loop_start(_mosquitto_client); }

void mqttConnectCallback(struct mosquitto *mosq,
                         [[maybe_unused]] void *userdata, int result) {
  if (!result) {
    std::cout << "Connect successful: Subscribing to AmrUnit topics"
              << std::endl;
    // Subscribe to 3 broker information topics relevant for the AMR unit
    int n_subs = 3;
    char *subscriptions[3];
    char sub_0[25] = "/AmrUnit/currentPosition";
    char sub_1[19] = "/AmrUnit/nextOrder";
    char sub_2[18] = "/AmrUnit/shutdown";
    subscriptions[0] = sub_0;
    subscriptions[1] = sub_1;
    subscriptions[2] = sub_2;

    mosquitto_subscribe_multiple(mosq, NULL, n_subs, subscriptions, 2, 0, NULL);
  } else {
    std::cerr << "MqttInterface Error: Connection failed" << std::endl;
  }
}

void mqttMessageCallback([[maybe_unused]] struct mosquitto *mosq,
                         void *userdata,
                         const struct mosquitto_message *message) {
  TaskQueue *task_queue = static_cast<TaskQueue *>(userdata);
  // check if the message is not empty
  std::string msg_topic = std::string(static_cast<char *>(message->topic));
  if (msg_topic == "/AmrUnit/shutdown") {
    task_queue->_mutex.lock();
    task_queue->_shutdown = true;
    task_queue->_mutex.unlock();
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, false);
  } else if (message->payloadlen) {
    // the following variable will be set to false in case of errors
    bool create_new_task = true;
    // message is not empty
    if (msg_topic == "/AmrUnit/nextOrder") {
      // get the order id and description and create a new entry in the task
      // queue
      try {
        YAML::Node msg_yaml = YAML::Load(static_cast<char *>(message->payload));
        uint32_t order_id = 0;
        if (msg_yaml["order_id"]) {
          order_id = msg_yaml["order_id"].as<uint32_t>();
        } else {
          std::cout << "Error: Key 'order_id' in message for topic "
                       "/AmrUnit/nextOrder is missing"
                    << std::endl;
          create_new_task = false;
        }
        std::string description;
        if (msg_yaml["description"]) {
          description = msg_yaml["description"].as<std::string>();
        } else {
          std::cout << "Error: Key 'description' in message for topic "
                       "/AmrUnit/nextOrder is missing"
                    << std::endl;
          create_new_task = false;
        }
        // check if unexpected key is included and print a warning
        for (auto it = msg_yaml.begin(); it != msg_yaml.end(); ++it) {
          std::string key = it->first.as<std::string>();
          if (key != "order_id" && key != "description") {
            std::cout << "Warning: Received message in /AmrUnit/nextOrder with "
                         "unexpected key: "
                      << key << std::endl;
          }
        }

        if (create_new_task) {
          // add the received order as new task to the queue
          OrderExecutor *newOrderExecutor =
              new OrderExecutor(order_id, description);
          task_queue->_mutex.lock();
          task_queue->_queue.push(newOrderExecutor);
          task_queue->_mutex.unlock();
        }
      } catch (const YAML::ParserException &e) {
        std::cout << "Error: Could not interpret message as Map. Please retry "
                     "using exactly the following format:"
                  << std::endl;
        std::cout << "\"{order_id: <order_id>, description: <description>}\""
                  << std::endl;
        std::cout << e.what() << std::endl;
      }
    } else if (msg_topic == "/AmrUnit/currentPosition") {
      try {
        // get the position and create a new entry in the task queue
        YAML::Node msg_yaml = YAML::Load(static_cast<char *>(message->payload));
        double x{0.0}, y{0.0}, yaw{0.0};
        if (msg_yaml["x"]) {
          x = msg_yaml["x"].as<double>();
        } else {
          std::cout << "Error: Key 'x' in message for topic "
                       "/AmrUnit/currentPosition is missing"
                    << std::endl;
          create_new_task = false;
        }
        if (msg_yaml["y"]) {
          y = msg_yaml["y"].as<double>();
        } else {
          std::cout << "Error: Key 'y' in message for topic "
                       "/AmrUnit/currentPosition is missing"
                    << std::endl;
          create_new_task = false;
        }
        if (msg_yaml["yaw"]) {
          yaw = msg_yaml["yaw"].as<double>();
        } else {
          std::cout << "Warning: Key 'yaw' in message for topic "
                       "/AmrUnit/currentPosition is missing"
                    << std::endl;
        }
        // check if unexpected key is included and print a warning
        for (auto it = msg_yaml.begin(); it != msg_yaml.end(); ++it) {
          std::string key = it->first.as<std::string>();
          if (key != "x" && key != "y" && key != "yaw") {
            std::cout << "Warning: Received message in /AmrUnit/nextOrder with "
                         "unexpected key: "
                      << key << std::endl;
          }
        }
        // add the received order as new task to the queue
        if (create_new_task) {
          MoveExecutor *newMoveExecutor = new MoveExecutor(Position(x, y, yaw));
          task_queue->_mutex.lock();
          task_queue->_queue.push(newMoveExecutor);
          task_queue->_mutex.unlock();
        }
      } catch (const YAML::ParserException &e) {
        std::cout << "Error: Could not interpret message as Map. Please retry "
                     "using exactly the following format:"
                  << std::endl;
        std::cout << "\"{x: <x>, y: <y>, yaw: <yaw>}\"" << std::endl;
        std::cout << e.what() << std::endl;
      }
    }
  } else {
    std::cout << "Error: Message in " << msg_topic << " with empty payload"
              << std::endl;
  }
}

void mqttSubscribeCallback([[maybe_unused]] struct mosquitto *mosq,
                           [[maybe_unused]] void *userdata,
                           [[maybe_unused]] int mid, int qos_count,
                           const int *granted_qos) {
  std::cout << "Subscribe callback:" << std::endl;
  std::cout << "  Number of granted subs: " << qos_count
            << ", granted quos:" << std::endl;
  if (qos_count == 3) {
    std::cout << "  /AmrUnit/currentPosition: " << granted_qos[0] << std::endl;
    std::cout << "  /AmrUnit/nextOrder: " << granted_qos[1] << std::endl;
    std::cout << "  /AmrUnit/shutdown: " << granted_qos[2] << std::endl;
  } else {
    std::cout << "Warning: quos_count is unexpectedly not 3" << std::endl;
  }
  std::cout << std::endl;
}

}  // namespace AMR
