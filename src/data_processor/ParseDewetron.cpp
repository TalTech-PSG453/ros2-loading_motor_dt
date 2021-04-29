//
// Created by sejego on 10/18/20.
// This script parses the data from Dewetron measurement csv file
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include "ParseDewetron.h"

ParseDewetron::ParseDewetron(std::string filename, int numberOfColumns)
{
    numOfCols = numberOfColumns;
    //filename = filename;
    dewetronFile = std::ifstream(filename);
    //usedFrequency = frequency;
    /* automatic parsing of the file */
    parseDewetronFile();
    //std::cout << "Dewetron got parsed\n";
}

// parse the Dewetron measurements file
void ParseDewetron::parseDewetronFile()
{
    int firstLineRead = 0;
    std::vector<float> rowOfValues;
    listOfValueNames.reserve(numOfCols);
    rowOfValues.reserve(numOfCols);                 // reserve numOfCols number of columns of the csv file.
    bool isFirstLine = true;
    std::stringstream ss(line);
    std::string element;
    if(!dewetronFile.is_open())
    {
        throw std::runtime_error("Could not open file:");
    }

        /* if file is open and is OK continue */

    else if(dewetronFile.good()) {
        while (std::getline(dewetronFile, line))
        {
            /* Create a stringstream from line */

            std::stringstream ss(line);
            if(isFirstLine)                 // check for first line, it contains headers
            {
                while(std::getline(ss, element, ','))
                {
                    /* Excluding the _time postfixed names */

                    if (firstLineRead % 2 == 0)
                        listOfValueNames.push_back(element);
                    firstLineRead++;
                }

                isFirstLine = false;
                continue;
            }
            /* Create a stringstream from line */

            /* start reading the file line by line */

            while (std::getline(ss, element, ','))
            {
                rowOfValues.push_back(std::stof(element));
            }

            parsedData.push_back(rowOfValues);

            /*empty the buffer vector*/
            rowOfValues.clear();
        }
        dewetronFile.close();
    }

    numOfRows = parsedData.size();
    //std::cout << "Number of rows is: " << numOfRows << "\n";
    //std::cout << "Number of columns is " << numOfCols << "\n" ;
    //std::cout << "Size of vector is: " << (sizeof(std::vector<float>) + (sizeof(float) * parsedData.size())) << "\n";

    timeVector.resize(numOfRows);
    valuesVector.resize(numOfRows);
    separateToVectors();
    //std::cout << "Separate vectors worked\n";
}
std::vector<std::vector<float>> ParseDewetron::getProcessed2DVector()
{
    return parsedData;
}

void ParseDewetron::separateToVectors()
{
    int colCt = 0;
    for(auto row : parsedData)
    {
        for(int i = 0; i < numOfCols-1; i+=2)
        {
            timeVector[colCt].push_back(row[i+1]);
            valuesVector[colCt].push_back(row[i]);
        }
        colCt++;
    }

    //std::cout << timeVector.size() << "The size of timeVec\n";
    //std::cout << valuesVector.size() << "The size of valVec\n";

}

int ParseDewetron::getNumberOfColumns()
{
    return numOfCols;
}

int ParseDewetron::getNumberOfRows()
{
    return numOfRows;
}
std::vector<std::string> ParseDewetron::getListOfValueNames()
{
    return listOfValueNames;
}
float ParseDewetron::getStartTime() const
{
    return parsedData[1][0];
}
float ParseDewetron::getTimeStep() const
{
    return (parsedData[1][1] - parsedData[1][0]);
}
std::vector<std::vector<float>> ParseDewetron::getOnlyTimes()
{
    return timeVector;
}
std::vector<std::vector<float>> ParseDewetron::getOnlyValues()
{
    return valuesVector;
}
/* Destructor */
ParseDewetron::~ParseDewetron()
{
    parsedData.clear();
    timeVector.clear();
    valuesVector.clear();
}