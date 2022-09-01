/** @file basic_routines.hpp
 * @brief Contains basic routines, like parsing routines and one to determine
 * the geometrically shortest path in a 2D plane.
 */

#ifndef INCLUDE_BASIC_ROUTINES_HPP_
#define INCLUDE_BASIC_ROUTINES_HPP_

#include <iostream>
#include <vector>

#include "basic_structs.hpp"

namespace AMR {
/**
 * @brief Determines the length of a given path
 *
 * The path is specified by a starting point, an end point (delivery point) and
 * locations of parts in between that are visited according to a specified
 * order.
 *
 * @param starting_point  Starting point of the path.
 * @param part_locations  Points on the path between the starting and the end
 * point.
 * @param delivery_point  End point of the path.
 * @param pickup_order  Specifies the order in which the part locations are
 * visited. It is a vector of pairwise different integers in the set
 * {0, ..., n-1} where n is the number of parts given.
 * @return double
 */
double determinePathLength(const AMR::Coordinates2D &starting_point,
                           const std::vector<Coordinates2D> &part_locations,
                           const AMR::Coordinates2D &delivery_point,
                           const std::vector<int> &pickup_order);

/**
 * @brief Determines the geometrically shortest path connecting a given starting
 * and delivery point while collecting several parts on the way.
 *
 * @param[in] starting_point  Starting point of the path.
 * @param[in] part_locations  Vector containing the locations of all parts which
 * have to be collected.
 * @param[in] delivery_point  Delivery coordinates of the order.
 * @param[in,out] pickup_order  Order in which the products have to be picked
 * up.
 */
void determineShortestPath(const AMR::Coordinates2D &starting_point,
                           const std::vector<Coordinates2D> &part_locations,
                           const AMR::Coordinates2D &delivery_point,
                           std::vector<int> &pickup_order);

/**
 * @brief Parses the configuration file in the proper subdirectory and
 * fills a given vector with the products in this file.
 *
 * @param[in] dir_path  Path to the directory containing the configuration
 * files.
 * @param[in,out] all_products  Vector filled with all products in the
 * configuration file.
 * @param[in,out] all_product_parts Vector filled with all the possible parts
 * required for the products.
 * @note This function is used to initialize @ref AMR::AmrUnit::_all_products.
 */
void parseConfigurationFiles(const std::string &dir_path,
                             std::vector<AMR::Product> &all_products,
                             std::vector<AMR::ProductPart> &all_product_parts);

/**
 * @brief Parses all order files in the proper subdirectory searching for
 * information about an order whose id is known.
 *
 * If the order is found, the return value is set to true and the output
 * variables delivery_point and ordered_products are set appropriately.
 * Otherwise, the return value is set to false and the output variables are
 * not changed.
 *
 * @param[in] dir_path  Path to the directory containing the order
 * files.
 * @param[in] order_id  Id of the order whose information is wanted.
 * @param[in,out] delivery_point Delivery point of the order.
 * @param[in,out] ordered_products Products of the order.
 * @return true The order was found.
 * @return false  The order was not found.
 */
bool parseAllFilesToFindOrder(const std::string &dir_path,
                              const uint32_t order_id,
                              AMR::Coordinates2D &delivery_point,
                              std::vector<long long int> &ordered_products);

/**
 * @brief Parses one order file and looks in each order(item) if the order_id 
 * is the one searched for.
 *
 * If the order is found, found_ is set to true and variables 
 * del_point_ and ordered_products_ are set appropriately.
 *
 * @param[in] file_name_  Path to the order file.
 * @param[in] order_id_  Id of the order whose information is wanted.
 * @param[in,out] del_point_ Delivery point of the order.
 * @param[in,out] ordered_products_ Products of the order.
 * @note This function is used in threads to parse files in paralel.
 */
void* parseFile(void* args);

/**
 * @brief Creates a n x n Matrix with distances betwen nodes in coord
 * vector with n-2 nods and start and end nodes
 *
 * @param[in] file_name_  Path to the order file.
 * @param[in] order_id_  Id of the order whose information is wanted.
 * @param[in,out] del_point_ Delivery point of the order.
 * @param[in,out] ordered_products_ Products of the order.
 * @return nxn Matrix with distances
 */
std::vector<std::vector<int>> makeMatrixFromVector(std::vector<Coordinates2D> coord, 
    AMR::Coordinates2D start_c, AMR::Coordinates2D end_c);
} // namespace AMR
#endif  //#ifndef INCLUDE_BASIC_ROUTINES_HPP_