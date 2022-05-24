//
// Created by sejego on 10/18/20.
// Last modified by sejego on 01/18/20.
//

#ifndef CATKIN_WS_PARSEDEWETRON_H
#define CATKIN_WS_PARSEDEWETRON_H

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

/*
 * Summary:
 *      ParseDewetron class provides a mechanism of converting .csv files separated by a ',' of the
 * format defined the following way:
 *
 *
 *       +--------------------+--------------------+--------------------+--------------------+
 *       | MeasuredParameter1 | MeasuredParameter2 | MeasuredParameter3 | MeasuredParameterX |
 *       +--------------------+--------------------+--------------------+--------------------+
 *       | Value1             | Value1             | Value1             | Value1             |
 *       | Value2             | Value2             | Value2             | Value2             |
 *       | ValueX             | ValueX             | ValueX             | ValueX             |
 *       +--------------------+--------------------+--------------------+--------------------+
 *
 *
 *      Where each column corresponds to a measured parameter, with rows being values measured per
 * parameter with a set frequency. The number of columns must be known beforehand. The class will
 * process the values into vectors (one vector per measured parameter) and return them via getter.
 * */
class ParseDewetron {
 public:
  /* File parser */
  ParseDewetron(std::string filename, int numberOfColumns);
  ~ParseDewetron();

  /* Getters */

  std::vector<std::vector<float>> get_processed_2D_vector();
  float get_time_step() const;
  float get_start_time() const;
  int get_number_of_columns();
  int get_number_of_rows();
  std::vector<std::string> get_list_of_value_names();
  std::vector<std::vector<float>> get_only_times();
  std::vector<std::vector<float>> get_only_values();

 private:
  void separate_to_vectors();
  std::ifstream dewetron_file_;
  float time_step_;
  float start_time_;
  int num_of_cols_;
  int num_of_rows_;
  std::string filename_;
  std::vector<std::vector<float>> parsed_data_;
  std::vector<std::string> list_of_value_names_;
  std::vector<std::vector<float>> time_vector_;
  std::vector<std::vector<float>> values_vector_;
  std::string line_;
  void parse_dewetron_file();
};

#endif  // CATKIN_WS_PARSEDEWETRON_H
