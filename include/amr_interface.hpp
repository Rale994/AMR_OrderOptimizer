/** @file amr_interface.hpp
 * @brief Defines interfaces for the communication with an AMR unit.
 */

#ifndef INCLUDE_AMR_INTERFACE_HPP_
#define INCLUDE_AMR_INTERFACE_HPP_

#include <mosquitto.h>

#include <string>

namespace AMR {
// forward declaration:
class TaskQueue;

/**
 * @brief Abstract base class for interfaces of AMR units.
 *
 */
class Interface {
 public:
  /**
   * @brief Destroy the Interface object.
   *
   */
  virtual ~Interface(){};

  /**
   * @brief Virtual start function.
   *
   */
  virtual void run() = 0;
};

/**
 * @brief MQTT Interface for AMR Unit.
 *
 */
class MqttInterface : public Interface {
 public:
  /**
   * @brief Construct a new Mqtt Interface object.
   *
   * @param[in] host  Host for the mqtt client.
   * @param[in] port  Port for the mqtt client.
   * @param[in] client_id Id that will identify the mqtt client
   * @param[in] task_queue  Pointer to task queue in which received messages are
   * stored.
   *
   * @warning Use mosquitto_lib_init() before calling this constructor, and
   * mosquitto_lib_cleanup() after all instances are no longer needed (and have
   * terminated after receiving the corresponding message)
   */
  MqttInterface(const std::string host, const int port,
                const std::string client_id, AMR::TaskQueue *const task_queue);
  /**
   * @brief Destroy the Mqtt Interface object
   *
   */
  ~MqttInterface();

  /**
   * @brief Start the loop of the mqtt client in a separate thread.
   *
   */
  virtual void run();

 private:
  struct mosquitto
      *_mosquitto_client;  //!< Mosquitto client used for receiving messages.
  static constexpr int _keep_alive =
      60;  //!< Keep alive parameter for mosquitto client.
};

/**
 * @brief Connect callback function used by the mosquitto client of the
 * MqttInterface
 *
 * @param[in] mosq  Pointer to mosquitto client that executes the callback.
 * @param[in] userdata  Unused here.
 * @param[in] result  Indicates whether the connection was successful (0) or not
 * (1)
 */
void mqttConnectCallback(struct mosquitto *mosq,
                         [[maybe_unused]] void *userdata, int result);

/**
 * @brief Message callback function used by the mosquitto client of the
 * MqttInterface.
 *
 * It processes the incoming messages and updates the task queue which is
 * provided via @p userdata.
 * The access to the provided task queue is thread safe.
 *
 * @param[in] mosq Pointer to mosquitto client that executes the callback.
 * @param[in] userdata  Pointer to the task queue where the processed messages
 * (tasks)are inserted.
 * @param[in] message Pointer to the received message.
 */
void mqttMessageCallback([[maybe_unused]] struct mosquitto *mosq,
                         void *userdata,
                         const struct mosquitto_message *message);

/**
 * @brief Subscribe callback function used by the mosquitto client of the
 * MqttInterface
 *
 * @param[in] mosq  Pointer to mosquitto client that executes the callback.
 * @param[in] userdata Unused here.
 * @param[in] mid Message here. In general important, but unused here.
 * @param[in] qos_count Counter of the subscriptions of the callback.
 * @param[in] granted_qos Array containing the granted qos for the
 * subscriptions.
 */
void mqttSubscribeCallback([[maybe_unused]] struct mosquitto *mosq,
                           [[maybe_unused]] void *userdata,
                           [[maybe_unused]] int mid, int qos_count,
                           const int *granted_qos);

}  // namespace AMR

#endif  // define INCLUDE_AMR_INTERFACE_HPP_
