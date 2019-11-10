//
// Created by yash on 10/20/19.
//

#ifndef SRC_CSV_READER_H
#define SRC_CSV_READER_H

#include <fstream>
#include <utility>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

namespace f110
{

/// A Class to read csv data files
class CSVReader
{
    std::string fileName;
    std::string delimeter;

public:
    explicit CSVReader(std::string filename, std::string delm = ",") :
            fileName(std::move(filename)), delimeter(std::move(delm))
    {}

    ///
    /// Function to fetch data from a CSV File
    std::vector<std::array<double, 2>> getData()
    {
        std::ifstream file(fileName);
        if (!file)
        {
            throw std::runtime_error("Invalid Path for csv file.");
        }
        std::vector<std::array<double, 2>> dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while (getline(file, line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
            std::array<double, 2> trackpoint{};
            trackpoint[0] = std::stod(vec[0]);
            trackpoint[1] = std::stod(vec[1]);

            dataList.emplace_back(trackpoint);
        }
        // Close the File
        file.close();

        return dataList;
    }
};

} // namespace f110

#endif //SRC_CSV_READER_H
