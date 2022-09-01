#include "basic_routines.hpp"

#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <pthread.h>
#include "yaml-cpp/yaml.h"

double AMR::determinePathLength(
    const AMR::Coordinates2D &starting_point,
    const std::vector<Coordinates2D> &part_locations,
    const AMR::Coordinates2D &delivery_point,
    const std::vector<int> &pickup_order)
{
  // compute the distance between the starting point and the first part location
  double x_diff = part_locations[pickup_order[0]]._x - starting_point._x;
  double y_diff = part_locations[pickup_order[0]]._y - starting_point._y;
  double path_length = sqrt(x_diff * x_diff + y_diff * y_diff);
  // add the distances between all the parts in the specified order
  for (size_t i = 0; i < pickup_order.size() - 1; ++i)
  {
    x_diff = part_locations[pickup_order[i + 1]]._x -
             part_locations[pickup_order[i]]._x;
    y_diff = part_locations[pickup_order[i + 1]]._y -
             part_locations[pickup_order[i]]._y;
    path_length += sqrt(x_diff * x_diff + y_diff * y_diff);
  }
  // add the distance between the last part and the delivery point
  x_diff = delivery_point._x - part_locations.back()._x;
  y_diff = delivery_point._y - part_locations.back()._y;
  path_length += sqrt(x_diff * x_diff + y_diff * y_diff);
  return path_length;
}

std::vector<std::vector<int>> AMR::makeMatrixFromVector(
    std::vector<Coordinates2D> coord,
    AMR::Coordinates2D start_c, AMR::Coordinates2D end_c)
{
  // std::cout << "Corrd Before adding st and end:\n"; 
  // for(int i = 0; i < coord.size(); i++) {
  //   std::cout << "Corrd[" << i << "] : x=" << coord[i]._x << " y=" << coord[i]._y << "\n"; 
  // }
  // std::cout << "Start: x="<< start_c._x << " y=" << start_c._y << "\n";
  // std::cout << "End: x="<< end_c._x << " y=" << end_c._y << "\n";

  std::vector<Coordinates2D> coord_st_end(coord);
  coord_st_end.push_back(end_c);
  coord_st_end.insert(coord_st_end.begin(), start_c);
  int n = coord_st_end.size();
  // std::cout << "Corrd AFTER adding st and end:\n"; 
  // for(int i = 0; i < coord_st_end.size(); i++) {
  //   std::cout << "Corrd[" << i << "] : x=" << coord_st_end[i]._x << " y=" << coord_st_end[i]._y << "\n"; 
  // }

  std::vector<std::vector<int>> matrix_nxn(n, std::vector<int>(n));

  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      matrix_nxn[i][j] = sqrt(pow(coord_st_end[i]._x - coord_st_end[j]._x, 2) +
                              pow(coord_st_end[i]._y - coord_st_end[j]._y, 2));
    }
  }
  // std::cout << "Matrix: \n";
  // for (int i = 0; i < n; i++)
  // {
  //   std::cout << "Row[" << i << "] : ";
  //   for (int j = 0; j < n; j++)
  //   {
  //     std::cout << matrix_nxn[i][j] << ", ";
  //   }
  //   std::cout << "\n";
  // }
  return matrix_nxn;
}

void AMR::determineShortestPath(
    const AMR::Coordinates2D &starting_point,
    const std::vector<Coordinates2D> &part_locations,
    const AMR::Coordinates2D &delivery_point, std::vector<int> &pickup_order)
{
  // first, prepare the output variable pickup_order
  pickup_order.resize(part_locations.size());
  
  // PLEASE ADD YOUR IMPLEMENTATION HERE
  // EXPLANATION: The vector pickup_order should be filled with integers that indicate
  // in which order the parts in the vector part_locations should be picked up. For example,
  // if part_locations contains 3 locations, a possible output would be {1,0,2}. This would
  // mean that the shortest path is:
  // starting_point, part_locations[1], part_locations[0], part_locations[2], delivery_point

  std::vector<std::vector<int>> matrix_ = makeMatrixFromVector(
    part_locations, starting_point, delivery_point);
  
  std::vector<int> temp_pickup_order; 
  
	std::vector<int> location_indexes; 

	for (int i = 1; i <= (int)part_locations.size(); i++) {
    location_indexes.push_back(i); 
  }
			
	double shortest_path = MAXFLOAT; 

	do { 

		double current_path = 0; 

    temp_pickup_order.clear();

    int current_point = 0;
		// compute current path weight 
		for (int i = 0; i < (int)location_indexes.size(); i++) { 
			current_path += matrix_[current_point][location_indexes[i]]; 
			current_point = location_indexes[i];
      temp_pickup_order.push_back(current_point);
		} 

    int delivery_point = part_locations.size() + 1;
	  current_path += matrix_[current_point][delivery_point]; 

    if (current_path < shortest_path){
      pickup_order.clear();
      for (int i = 0; i < (int)part_locations.size(); i++) {
        pickup_order.push_back(temp_pickup_order[i] - 1);
      }
      shortest_path = current_path;
    }

	} while (std::next_permutation(location_indexes.begin(), location_indexes.end())); 
}

void AMR::parseConfigurationFiles(
    const std::string &dir_path, std::vector<AMR::Product> &all_products,
    std::vector<AMR::ProductPart> &all_product_parts)
{
  std::string configuration_file = dir_path + "/products.yaml";
  std::ifstream fin(configuration_file);
  if (fin.is_open())
  {
    YAML::Node configuration_doc = YAML::Load(fin);
    long long int n_products = configuration_doc.size();
    // product ids might start at 1, so we reserve an additional position in
    // the vector to account for that.
    all_products.resize(n_products + 1);
    all_products[0]._name = "placeholder";
    for (auto product_iter = configuration_doc.begin();
         product_iter != configuration_doc.end(); ++product_iter)
    {
      // get the data of the current product
      long long int product_id = (*product_iter)["id"].as<long long int>();
      std::string product_name = (*product_iter)["product"].as<std::string>();
      // Add all the parts of the current product to a proper map (of part ids
      // and quantities); Update the vector of all product parts if a new part
      // is encountered.
      YAML::Node current_product_parts = (*product_iter)["parts"];
      std::map<long long int, int> parts_and_quantities;
      for (auto part_iter = current_product_parts.begin();
           part_iter != current_product_parts.end(); ++part_iter)
      {
        std::string part_name = (*part_iter)["part"].as<std::string>();
        // check whether the current part is already
        auto find_part_iter =
            std::find_if(all_product_parts.begin(), all_product_parts.end(),
                         [&part_name](AMR::ProductPart &part)
                         {
                           return (part._name == part_name);
                         });
        long long part_id = 0;
        if (find_part_iter == all_product_parts.end())
        {
          // new part; update the vector of all product parts
          double x_coord = (*part_iter)["cx"].as<double>();
          double y_coord = (*part_iter)["cy"].as<double>();
          all_product_parts.push_back(ProductPart(part_name, x_coord, y_coord));
          // the id of the current part is its position in the vector
          part_id = all_product_parts.size() - 1;
        }
        else
        {
          part_id = distance(all_product_parts.begin(), find_part_iter);
        }
        // add the part to the map parts_and_quantities. If the id of the part
        // is already included, update the quantity counter.
        auto part_in_map_iter = parts_and_quantities.find(part_id);
        if (part_in_map_iter == parts_and_quantities.end())
        {
          parts_and_quantities.emplace(std::make_pair(part_id, 1));
        }
        else
        {
          (*part_in_map_iter).second++;
        }
      }
      all_products[product_id]._name = product_name;
      all_products[product_id]._parts = std::move(parts_and_quantities);
    }
  }
  else
  {
    std::cout << "file: " << configuration_file << " not found " << std::endl;
  }
}

void* AMR::parseFile(void * args) {
  AMR::ParseFilArgs* parse_args = (AMR::ParseFilArgs*) args;
  //std::cout << parse_args->file_name_ << "\n";
  std::ifstream fin(parse_args->file_name_);
  if(fin.is_open())
  {
    YAML::Node order_file = YAML::Load(fin);
    for (auto order_iter = order_file.begin();
         order_iter != order_file.end(); ++order_iter) {
      if ((*order_iter)["order"].as<uint64_t>() == parse_args->order_id_)
      {
        //std::cout << "Order id:" << (*order_iter)["order"].as<uint64_t>() << parse_args->order_id_ << "\n";
        parse_args->found_ = true;
        parse_args->del_point_._x = (*order_iter)["cx"].as<double>();
        parse_args->del_point_._y = (*order_iter)["cy"].as<double>();
        YAML::Node products = (*order_iter)["products"];
        parse_args->ordered_products_.resize(products.size());
        int i = 0;
        for (auto product_iter = products.begin();
           product_iter != products.end(); ++product_iter)
        {
          parse_args->ordered_products_[i] = (*product_iter).as<long long int>();
          i++;
        }
      }
    }
  }
  else
  {
    std::cout << "\nFile not opened!\n";
  }
  delete parse_args;
  return NULL;
}

bool AMR::parseAllFilesToFindOrder(
    const std::string &dir_path, const uint32_t order_id,
    AMR::Coordinates2D &delivery_point,
    std::vector<long long int> &ordered_products)
{
  // the number of files and the names of the files is hardcoded here. It
  // could be retrieved by using std::filesystem routines
  int n_files = 5;
  std::vector<std::string> file_names = {
      dir_path + "/orders_20201201.yaml", dir_path + "/orders_20201202.yaml",
      dir_path + "/orders_20201203.yaml", dir_path + "/orders_20201204.yaml",
      dir_path + "/orders_20201205.yaml"};

  bool found = false;
 
  
  std::vector<pthread_t> threads;
  threads.resize(n_files);
  for (int i = 0; i < n_files; i++)
  {
    auto to_pass = file_names[i];
    AMR::ParseFilArgs* arguments = new AMR::ParseFilArgs{to_pass, order_id, found, delivery_point, ordered_products};
    //parseFile((void*) &arguments);
    pthread_create(&threads[i], NULL, &parseFile, (void*)arguments);
  }

  for (int i = 0; i < n_files; i++)
  {
    pthread_join(threads[i], NULL);
  }

  // for(auto part : ordered_products) {
  //   std::cout << "Part: " << part  << "\n";
  // }
  // std::cout << std::setprecision(10);
  // std::cout << "Delivery: x=" << (double)delivery_point._x << " y=" << (double)delivery_point._y << "\n";
  return found;
}
