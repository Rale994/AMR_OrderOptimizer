/** @file amr_unit.hpp
 * @brief Defines the class amr_unit, which represent a single autonomous mobile
 * robot unit.
 */

#ifndef INCLUDE_AMR_UNIT_HPP_
#define INCLUDE_AMR_UNIT_HPP_

#include <queue>
#include <string>
#include <vector>

#include "amr_interface.hpp"
#include "amr_task_executors.hpp"
#include "basic_structs.hpp"

namespace AMR {

/**
 * @brief Basic AMR Unit. This is the main class of the project.
 *
 */
class AmrUnit {
 public:
  /**
   * @brief Construct a new Amr Unit.
   *
   * @param[in] working_directory Directory where the orders and configuration
   * subdirectory resides.
   * @param[in] mqtt_client_id  Id assigned to the MQTT client assigned to the
   * unit.
   * @param[in] host  Host of the MQTT network.
   * @param[in] port  Port used for MQTT communication.
   * @param[in] starting_position Starting position of the unit.
   */
  AmrUnit(std::string working_directory,
          const std::string mqtt_client_id = "AmrUnitMqttClient",
          const std::string host = "localhost", const int port = 1883,
          AMR::Position starting_position = AMR::Position(0.0, 0.0, 0.0));

  /**
   * @brief Destroy the Amr Unit object
   *
   */
  ~AmrUnit();

  /**
   * @brief Set the Position of the current AmrUnit
   *
   * @param[in] new_position New position for the AmrUnit.
   */
  void setCurrentPosition(const AMR::Position& new_position) {
    _current_position._coords_2d = new_position._coords_2d;
    _current_position._yaw = new_position._yaw;
  }

  /**
   * @brief Get the Working Directory of the AmrUnit.
   *
   * @return @ref _working_directory.
   */
  const std::string& getWorkingDirectory() const { return _working_directory; }

  /**
   * @brief Get the current position of the AmrUnit.
   *
   * @return @ref _current_position.
   */
  const AMR::Position& getCurrentPosition() const { return _current_position; }

  /**
   * @brief Get all products the unit found in the configuration directory.
   *
   * @return @ref _all_products.
   */
  const std::vector<AMR::Product>& get_all_products() const {
    return _all_products;
  }

  /**
   * @brief Get all product parts needed for products.
   *
   * @return const std::vector<AMR::ProductPart>&
   */
  const std::vector<AMR::ProductPart>& get_all_product_parts() const {
    return _all_product_parts;
  }

  /**
   * @brief Lets the AmrUnit run.
   *
   * When calling this routine the AmrInterface starts to listen for messages
   * and fills the task queue with tasks, which are then executed.
   *
   * The AmrUnit can be turned off by sending a message to the
   * topic "/AmrUnit/shutdown".
   *
   */
  void run();

 private:
  AMR::Interface* _interface;   //!< Interface handling incoming tasks;
  AMR::TaskQueue* _task_queue;  //!< Incoming tasks are added into this queue in
                                //!< a thread safe manner.
  Position _current_position;   //!< Current position of the AMR Unit
  std::string
      _working_directory;  //!< Working directory containing the subdirectories
                           //!< for orders and configurations.
  std::vector<AMR::Product>
      _all_products;  //!< List storing all the products given in the
                      //!< configuration subdirectoy. It is filled by the
                      //!< routine @ref AMR::parseConfigurationFiles.
                      //!< NOTE: One could store this outside of the
                      //!< unit, to allow several units to access the same
                      //!< vector (if desired).

  std::vector<AMR::ProductPart>
      _all_product_parts;  //!< List storing all the product parts given in the
                           //!< configuration subdirectory. It is filled by the
                           //!< routine @ref AMR::parseConfigurationFiles.
                           //!< NOTE: One could store this outside of the
                           //!< unit, to allow several units to access the same
                           //!< vector (if desired).
};
}  // namespace AMR

#endif  // define INCLUDE_AMR_UNIT_HPP_
