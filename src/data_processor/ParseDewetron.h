//
// Created by sejego on 10/18/20.
// Last modified by sejego on 01/18/20.
//

#ifndef CATKIN_WS_PARSEDEWETRON_H
#define CATKIN_WS_PARSEDEWETRON_H

#include <iostream>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>

/*
 * Summary:
 *      ParseDewetron class provides a mechanism of converting .csv files separated by a ',' of the format defined the following way:
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
 *      Where each column corresponds to a measured parameter, with rows being values measured per parameter with a set
 *      frequency. The number of columns must be known beforehand. The class will process the values into vectors
 *      (one vector per measured parameter) and return them via getter.
 * */
class ParseDewetron
{
public:

    /* File parser */
    ParseDewetron(std::string filename, int numberOfColumns);
    ~ParseDewetron();

    /* Getters */

    std::vector<std::vector<float>> getProcessed2DVector();
    float getTimeStep() const;
    float getStartTime() const;
    int getNumberOfColumns();
    int getNumberOfRows();
    std::vector<std::string> getListOfValueNames();
    std::vector<std::vector<float>> getOnlyTimes();
    std::vector<std::vector<float>> getOnlyValues();

private:
    void separateToVectors();
    std::ifstream dewetronFile;
    float timeStep;
    float startTime;
    int numOfCols;
    int numOfRows;
    std::string filename;
    std::vector<std::vector<float>> parsedData;
    std::vector<std::string> listOfValueNames;
    std::vector<std::vector<float>> timeVector;
    std::vector<std::vector<float>> valuesVector;
    std::string line;
    void parseDewetronFile();
};

#endif //CATKIN_WS_PARSEDEWETRON_H
