/** @file unit_tests.hpp
 * @brief Unit tests for the AMR project.
 */

#ifndef INCLUDE_AMR_UNIT_TESTS_HPP_
#define INCLUDE_AMR_UNIT_TESTS_HPP_

#include <string>

#include "amr.hpp"

namespace AMR {
namespace tests {
TEST(ParseOrder, missingOrderDetected) {
  constexpr uint32_t order_id = 66;
  const std::string dir_path = "./../tests/test_orders";
  AMR::Coordinates2D delivery_point;
  std::vector<long long int> ordered_products;
  bool order_found = parseAllFilesToFindOrder(dir_path, order_id,
                                              delivery_point, ordered_products);
  EXPECT_FALSE(order_found);
}

TEST(ParseOrder, OrderProcessedCorrectly) {
  constexpr uint32_t order_id = 1000001;
  const std::string dir_path = "./../tests/test_orders";
  AMR::Coordinates2D delivery_point;
  std::vector<long long int> ordered_products;
  bool order_found = parseAllFilesToFindOrder(dir_path, order_id,
                                              delivery_point, ordered_products);
  EXPECT_TRUE(order_found);
  const std::vector<long long int> reference_products{902, 293, 142, 56, 894};
  EXPECT_DOUBLE_EQ(delivery_point._x, 748.944);
  EXPECT_DOUBLE_EQ(delivery_point._y, 474.71707);
  EXPECT_EQ(ordered_products, reference_products);
}

TEST(ParseConfiguration, ProductsParsedCorrectly) {
  const std::string dir_path = "./../tests/test_configuration";
  std::vector<AMR::Product> products;
  std::vector<AMR::ProductPart> product_parts;
  parseConfigurationFiles(dir_path, products, product_parts);
  // Check products
  ASSERT_EQ(products.size(), 4);
  // check product 1
  EXPECT_EQ(products[1]._parts.at(0), 2);
  EXPECT_EQ(products[1]._parts.at(1), 2);
  // check product 2
  EXPECT_EQ(products[2]._parts.at(0), 3);
  // check product 3
  EXPECT_EQ(products[3]._parts.at(0), 2);
  EXPECT_EQ(products[3]._parts.at(2), 1);

  // Check product parts
  ASSERT_EQ(product_parts.size(), 3);

  // Check product part 0
  EXPECT_EQ(product_parts[0]._name, "Part A");
  EXPECT_DOUBLE_EQ(product_parts.at(0)._coords._x, 791.86304);
  EXPECT_DOUBLE_EQ(product_parts.at(0)._coords._y, 732.23236);

  // Check product part 1
  EXPECT_EQ(product_parts[1]._name, "Part B");
  EXPECT_DOUBLE_EQ(product_parts.at(1)._coords._x, 550.09924);
  EXPECT_DOUBLE_EQ(product_parts.at(1)._coords._y, 655.423);
  // Check product part 2
  EXPECT_EQ(product_parts[2]._name, "Part C");
  EXPECT_DOUBLE_EQ(product_parts.at(2)._coords._x, 281.39413);
  EXPECT_DOUBLE_EQ(product_parts.at(2)._coords._y, 68.39627);
}

TEST(ShortestPath, DeterminePathCorrectly) {
  const Coordinates2D starting_point(0.0, 0.0);
  const std::vector<Coordinates2D> part_locations_a{
      {281.39413, 68.39627}, {550.09924, 655.423}, {791.86304, 732.23236}};
  const Coordinates2D delivery_point(800.0, 800.0);
  std::vector<int> pickup_order;
  determineShortestPath(starting_point, part_locations_a, delivery_point,
                        pickup_order);
  ASSERT_EQ(pickup_order.size(), part_locations_a.size());
  EXPECT_EQ(pickup_order, std::vector<int>({0, 1, 2}));

  // another test with the same points, but a different order in the original
  // vector
  const std::vector<Coordinates2D> part_locations_b{
      {550.09924, 655.423}, {281.39413, 68.39627}, {791.86304, 732.23236}};
  determineShortestPath(starting_point, part_locations_b, delivery_point,
                        pickup_order);
  ASSERT_EQ(pickup_order.size(), part_locations_a.size());
  EXPECT_EQ(pickup_order, std::vector<int>({1, 0, 2}));
}

}  // namespace tests
}  // namespace AMR

#endif  // define INCLUDE_AMR_UNIT_TESTS_HPP_
