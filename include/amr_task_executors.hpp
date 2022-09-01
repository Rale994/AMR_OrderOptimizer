/** @file amr_task_executors.hpp
 * @brief Defines the tasks which the AMR unit can execute
 */

#ifndef INCLUDE_AMR_TASK_EXECUTORS_HPP_
#define INCLUDE_AMR_TASK_EXECUTORS_HPP_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "basic_structs.hpp"

namespace AMR {
// forward declaration
class AmrUnit;

/**
 * @brief Abstract basis class for task executors.
 *
 */
class TaskExecutor {
 public:
  /**
   * @brief Virtual destructor for abstract basis class.
   *
   */
  virtual ~TaskExecutor(){};

  /**
   * @brief Virtual execute function for abstract basis class.
   *
   * @param target_unit AMR Unit that should execute the task.
   * @param stream A message regarding the movement is printed into this
   * stream.
   * @note This is a purely abstract method.
   */
  virtual void execute(AMR::AmrUnit& target_unit,
                       std::ostream& stream = std::cout) const = 0;
};

/**
 * @brief Task executor that is used to move an AMR unit.
 *
 */
class MoveExecutor : public TaskExecutor {
 public:
  /**
   * @brief Construct a new Move Executor object
   *
   * @param[in] target_position Position to which the Move Executor moves an
   * AmrUnit.
   */
  MoveExecutor(AMR::Position target_position)
      : _target_position(target_position){};

  /**
   * @brief Lets an AMR unit execute a move operation.
   *
   * @param[in] target_unit AMR unit that is moved.
   * @param[in] stream  A message regarding the movement is printed into this
   * stream.
   */
  virtual void execute(AMR::AmrUnit& target_unit,
                       std::ostream& stream = std::cout) const;

 private:
  AMR::Position _target_position;  //!< Target position for an AMR unit.
};

/**
 * @brief Task executor that is used to let an AMR unit process an order.
 *
 */
class OrderExecutor : public TaskExecutor {
 public:
  /**
   * @brief Construct a new Order Executor object
   *
   * @param[in] order_id  Id of the order that is executed.
   * @param[in] order_description   Description of the order that is executed.
   */
  OrderExecutor(const uint32_t order_id, const std::string& order_description)
      : _order_id(order_id), _order_description(order_description){};

  /**
   * @brief Lets a given AMR unit execute the operation corresponding to the
   * given order
   *
   * @param target_unit AMR unit that executes the operation.
   * @param stream A message regarding the executed order is printed into this
   * stream.
   */
  virtual void execute(AMR::AmrUnit& target_unit,
                       std::ostream& stream = std::cout) const;

 private:
  /**
   * @brief Prints the delivery path of the given order.
   *
   * @param[in] starting_point Starting point of the path.
   * @param[in] delivery_point Delivery point of the path.
   * @param[in] pickup_order Order in which the product parts are picked up.
   * @param[in] processed_product_parts Structure containing the product parts
   * and corresponding information.
   * @param[in] processed_product_parts_position_to_key Auxiliary structure to
   * access @p processed_product_parts via an index, not the part id.
   * @param[in] all_product_parts Vector containing all available product parts.
   * @param[in] stream Stream to which the result is printed.
   */
  void printDeliveryPath(
      const Coordinates2D& starting_point, const Coordinates2D& delivery_point,
      const std::vector<int>& pickup_order,
      const AMR::ProcessedProductParts& processed_product_parts,
      const std::vector<long long int>& processed_product_parts_position_to_key,
      const std::vector<AMR::ProductPart>& all_product_parts,
      std::ostream& stream) const;

  /**
   * @brief Auxiliary function used to process ordered products.
   *
   * @param[in] ordered_products Vector containing the ids of the products of
   * the given order.
   * @param[in] all_products  Vector containing all available products.
   * @param[out] processed_product_parts Properly processed product parts. (See
   * the definition of the struct for clarification)
   */
  void processOrderedProducts(
      const std::vector<long long int>& ordered_products,
      const std::vector<AMR::Product>& all_products,
      AMR::ProcessedProductParts& processed_product_parts) const;

  uint32_t _order_id;  //!< Id of the order that is executed.
  std::string
      _order_description;  //!< Description of the order that is executed.
};

/**
 * @brief Simple task queue structure that enables thread safe handling of the
 * queue.
 *
 */
struct TaskQueue {
  std::mutex
      _mutex;  //!< Mutex to ensure thread safe access of queue and abort flag.
  std::queue<AMR::TaskExecutor*>
      _queue;      //!< Queue to store incoming task messages.
  bool _shutdown;  //!< Boolean that signals that a shutdown is desired.
};

}  // namespace AMR

#endif  // define INCLUDE_AMR_TASK_EXECUTORS_HPP_