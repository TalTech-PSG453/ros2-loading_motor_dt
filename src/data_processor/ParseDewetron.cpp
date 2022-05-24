//
// Created by sejego on 10/18/20.
// This script parses the data from Dewetron measurement csv file
//

#include "ParseDewetron.h"

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

ParseDewetron::ParseDewetron(std::string filename, int numberOfColumns) {
  num_of_cols_ = numberOfColumns;
  dewetron_file_ = std::ifstream(filename);
  parse_dewetron_file();
}

// parse the Dewetron measurements file
void ParseDewetron::parse_dewetron_file() {
  int firstLineRead = 0;
  std::vector<float> row_of_values;
  list_of_value_names_.reserve(num_of_cols_);
  row_of_values.reserve(num_of_cols_);  // reserve num_of_cols_ number of columns of the csv file.
  bool is_first_line = true;
  std::stringstream ss(line_);
  std::string element;
  if (!dewetron_file_.is_open()) {
    throw std::runtime_error(
        "Could not open file. Check filename definitions in /config/params.yaml");
  }

  // if file is open and is OK continue

  else if (dewetron_file_.good()) {
    while (std::getline(dewetron_file_, line_)) {
      // Create a stringstream from line_

      std::stringstream ss(line_);
      if (is_first_line) {  // check for first line_, it contains headers

        while (std::getline(ss, element, ',')) {
          // Excluding the _time postfixed names
          if (firstLineRead % 2 == 0) list_of_value_names_.push_back(element);
          firstLineRead++;
        }

        is_first_line = false;
        continue;
      }
      // Create a stringstream from line_

      // start reading the file line_ by line_

      while (std::getline(ss, element, ',')) {
        row_of_values.push_back(std::stof(element));
      }

      parsed_data_.push_back(row_of_values);

      // empty the buffer vector
      row_of_values.clear();
    }
    dewetron_file_.close();
  }

  num_of_rows_ = parsed_data_.size();

  time_vector_.resize(num_of_rows_);
  values_vector_.resize(num_of_rows_);
  separate_to_vectors();
}

std::vector<std::vector<float>> ParseDewetron::get_processed_2D_vector() { return parsed_data_; }

void ParseDewetron::separate_to_vectors() {
  int colCt = 0;
  for (auto row : parsed_data_) {
    for (int i = 0; i < num_of_cols_ - 1; i += 2) {
      time_vector_[colCt].push_back(row[i + 1]);
      values_vector_[colCt].push_back(row[i]);
    }
    colCt++;
  }
}

int ParseDewetron::get_number_of_columns() { return num_of_cols_; }
int ParseDewetron::get_number_of_rows() { return num_of_rows_; }
std::vector<std::string> ParseDewetron::get_list_of_value_names() { return list_of_value_names_; }
float ParseDewetron::get_start_time() const { return parsed_data_[1][0]; }
float ParseDewetron::get_time_step() const { return (parsed_data_[1][1] - parsed_data_[1][0]); }
std::vector<std::vector<float>> ParseDewetron::get_only_times() { return time_vector_; }
std::vector<std::vector<float>> ParseDewetron::get_only_values() { return values_vector_; }
/* Destructor */
ParseDewetron::~ParseDewetron() {
  parsed_data_.clear();
  time_vector_.clear();
  values_vector_.clear();
}