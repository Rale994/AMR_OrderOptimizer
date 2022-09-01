#include "amr_unit.hpp"

#include <chrono>
#include <thread>

#include "basic_routines.hpp"

namespace AMR {
AmrUnit::AmrUnit(std::string working_directory,
                 const std::string mqtt_client_id, const std::string host,
                 const int port, AMR::Position starting_position)
    : _current_position(starting_position),
      _working_directory(working_directory) {
  _task_queue = new TaskQueue();
  _task_queue->_shutdown = false;
  _interface = new MqttInterface(host, port, mqtt_client_id, _task_queue);
}

AmrUnit::~AmrUnit() {
  delete _task_queue;
  delete _interface;
}

void AmrUnit::run() {
  // first, parse all products in the appropriate file
  parseConfigurationFiles(_working_directory + "/configuration", _all_products,
                          _all_product_parts);
  _interface->run();

  _task_queue->_mutex.lock();
  bool shutdown = _task_queue->_shutdown;
  size_t queue_size = _task_queue->_queue.size();
  _task_queue->_mutex.unlock();
  TaskExecutor* nextTask = nullptr;
  while (!shutdown || queue_size > 0) {
    // check if task queue was updated
    _task_queue->_mutex.lock();
    shutdown = _task_queue->_shutdown;
    queue_size = _task_queue->_queue.size();
    if (queue_size > 0) {
      nextTask = _task_queue->_queue.front();
      _task_queue->_queue.pop();
    }
    _task_queue->_mutex.unlock();
    // execute a task if one is available
    if (nextTask != nullptr) {
      nextTask->execute(*this);
      // delete the task and set the point to null again.
      delete nextTask;
      nextTask = nullptr;
    }
    // let this thread sleep for 20ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "Received signal to shut down. Terminating." << std::endl;
}

}  // namespace AMR