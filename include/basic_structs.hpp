/** @file basic_structs.hpp
 * @brief Defines basic structures used in the whole project.
 */

#ifndef INCLUDE_BASIC_STRUCTS_HPP_
#define INCLUDE_BASIC_STRUCTS_HPP_

#include <map>
#include <queue>
#include <string>
#include <vector>

namespace AMR {
/**
 * @brief Struct representing 2D coordinates.
 *
 */
struct Coordinates2D {
  /**
   * @brief Default constructor for coordinates structure.
   */
  Coordinates2D() : _x(0.0), _y(0.0){};

  /**
   * @brief Construct a new coordinates structure.
   *
   * @param[in] x x coordinate.
   * @param[in] y y coordinate.
   */
  Coordinates2D(const double x, const double y) : _x(x), _y(y){};
  double _x;  //!< x coordinate.
  double _y;  //!< y coordinate.
};

/**
 * @brief Struct representing the position of an AMR Unit in 2D.
 *
 */
struct Position {
  /**
   * @brief Default constructor for a position structure.
   */
  Position() : _coords_2d(), _yaw(0.0){};

  /**
   * @brief Construct a new position structure by providing x,y and yaw.
   *
   * @param[in] x x coordinate.
   * @param[in] y y coordinat.
   * @param[in] yaw yaw.
   */
  Position(const double x, const double y, const double yaw)
      : _coords_2d(x, y), _yaw(yaw){};
  /**
   * @brief Construct a new position structure by providing coordinates and yaw.
   *
   * @param[in] coords_2d 2d coordinates.
   * @param[in] yaw yaw.
   */
  Position(const Coordinates2D coords_2d, const double yaw)
      : _coords_2d(coords_2d._x, coords_2d._y), _yaw(yaw){};
  Coordinates2D _coords_2d;  //!< Coordinates in 2D space.
  double _yaw;               //!< Yaw.
};

/**
 * @brief Struct representing a Product Part.
 *
 */
struct ProductPart {
  /**
   * @brief Construct a new product part
   *
   * @param[in] name Name of the product part.
   * @param[in] x x coordinate of the pickup position.
   * @param[in] y y cooridnate of the pickup position.
   */
  ProductPart(const std::string& name, const double x, const double y)
      : _name(name), _coords(x, y){};
  std::string _name;      //!< Name of the product part.
  Coordinates2D _coords;  //!< Coordinates where the part is located.
};

/**
 * @brief Struct representing a Product.
 *
 */
struct Product {
  std::string _name;  //!< Name of the product.
  std::map<long long int, int>
      _parts;  //!< Map of required parts; The key is the internal part_id, the
               //!< value the required quantity
};

/**
 * @brief Struct representing arguments for ParseFilArgs Thread.
 *
 */
struct ParseFilArgs {
  std::string file_name_;
  const uint32_t order_id_;
  bool &found_; 
  AMR::Coordinates2D &del_point_;
  std::vector<long long int> &ordered_products_;
};

/**
 * @brief Alias for processed product parts.
 *
 */
typedef std::map<long long int, std::vector<std::pair<long long int, int> > >
    ProcessedProductParts;  //!< Each key corresponds to a product part id and
                            //!< each value to the product information. Product
                            //!< information is a vector of pairs of product ids
                            //!< and required quantities.

}  // namespace AMR

#endif  // INCLUDE_BASIC_STRUCTS_HPP_
