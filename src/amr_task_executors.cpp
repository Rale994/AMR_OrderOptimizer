#include "amr_task_executors.hpp"

#include "amr_unit.hpp"
#include "basic_routines.hpp"

namespace AMR {
void MoveExecutor::execute(AMR::AmrUnit& target_unit,
                           std::ostream& stream) const {
  target_unit.setCurrentPosition(_target_position);
  stream << "Moved to position x: " << _target_position._coords_2d._x
         << ", y: " << _target_position._coords_2d._y << std::endl;
}

void OrderExecutor::execute(AMR::AmrUnit& target_unit,
                            std::ostream& stream) const {
  stream << "Working on order " << _order_id << "(" << _order_description << ")"
         << std::endl;
  // initialize data
  AMR::Coordinates2D delivery_point;
  std::vector<long long int> ordered_products;
  // get the information about the order by parsing all files
  
  bool found_order =
      parseAllFilesToFindOrder(target_unit.getWorkingDirectory() + "/orders",
                               _order_id, delivery_point, ordered_products);

  if (found_order) {
    AMR::ProcessedProductParts processed_product_parts;
    // determine all the product parts and their quantities (different products
    // can require the same parts)
    processOrderedProducts(ordered_products, target_unit.get_all_products(),
                           processed_product_parts);
    // get the coordinates of the processed product parts and an auxiliary
    // structure to access the map entries according to their position (this is
    // not clever. it would probably be better to use a vector instead of a map
    // for processed product parts (maybe just use a map for construction and
    // then copy to a vector))
    std::vector<long long int> processed_product_parts_position_to_key(
        processed_product_parts.size());
    std::vector<AMR::Coordinates2D> parts_positions(
        processed_product_parts.size());
    const std::vector<AMR::ProductPart>& all_product_parts =
        target_unit.get_all_product_parts();
    int counter = 0;
    for (auto iter = processed_product_parts.begin();
         iter != processed_product_parts.end(); ++iter) {
      processed_product_parts_position_to_key[counter] = iter->first;
      parts_positions[counter] = all_product_parts[iter->first]._coords;
      ++counter;
    }

    // determine the pickup order (geometrically shortest path!)
    std::vector<int> pickup_order;
    AMR::Coordinates2D starting_point =
        target_unit.getCurrentPosition()._coords_2d;
    determineShortestPath(starting_point, parts_positions, delivery_point,
                          pickup_order);

    // reposition the AmrUnit and print the result
    target_unit.setCurrentPosition(AMR::Position(delivery_point, 0.0));
    printDeliveryPath(
        starting_point, delivery_point, pickup_order, processed_product_parts,
        processed_product_parts_position_to_key, all_product_parts, stream);
  } else {
    stream << "Error: Order " << _order_id << " not found " << std::endl;
  }
}

void OrderExecutor::printDeliveryPath(
    const Coordinates2D& starting_point, const Coordinates2D& delivery_point,
    const std::vector<int>& pickup_order,
    const AMR::ProcessedProductParts& processed_product_parts,
    const std::vector<long long int>& processed_product_parts_position_to_key,
    const std::vector<AMR::ProductPart>& all_product_parts,
    std::ostream& stream) const {
  stream << "Starting from position x: " << starting_point._x
         << ", y: " << starting_point._y << std::endl;
  for (size_t i = 0; i < pickup_order.size(); ++i) {
    long long int part_id =
        processed_product_parts_position_to_key[pickup_order[i]];
    auto product_part_information = processed_product_parts.at(part_id);
    for (size_t j = 0; j < product_part_information.size(); ++j) {
      std::string product_name = all_product_parts[part_id]._name;
      AMR::Coordinates2D product_coords = all_product_parts[part_id]._coords;
      for (int k = 0; k < product_part_information[j].second; ++k) {
        stream << "Fetching '" << product_name << "' for product '"
               << product_part_information[j].first
               << "' at x: " << product_coords._x
               << ", y: " << product_coords._y << std::endl;
      }
    }
  }
  stream << "Delivering to destination x: " << delivery_point._x
         << ", y: " << delivery_point._y << std::endl;
}

void OrderExecutor::processOrderedProducts(
    const std::vector<long long int>& ordered_products,
    const std::vector<AMR::Product>& all_products,
    AMR::ProcessedProductParts& processed_product_parts) const {
  processed_product_parts.clear();
  for (size_t i = 0; i < ordered_products.size(); ++i) {
    long long int current_product_id = ordered_products[i];
    AMR::Product current_product = all_products[current_product_id];
    for (auto it = current_product._parts.begin();
         it != current_product._parts.end(); ++it) {
      long long int part_id = it->first;
      int quantity = it->second;
      // check if the part is already in the map of processed product parts
      // if yes, append the corresponding "value vector" appropriately,
      // otherwise create a new entry
      auto part_iter = processed_product_parts.find(part_id);
      if (part_iter != processed_product_parts.end()) {
        part_iter->second.push_back(
            std::make_pair(current_product_id, quantity));
      } else {
        processed_product_parts.emplace(
            std::make_pair(part_id, std::vector<std::pair<long long int, int>>{
                                        {current_product_id, quantity}}));
      }
    }
  }
}
}  // namespace AMR