#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>

void writeDataToFile(const std::string &Filename, const Eigen::VectorXd data,
                     const bool append, std::string delimiter = " ");
void writeStringToFile(const std::string &Filename, const std::string &ToWrite,
                       const bool Append);
std::string eigenToString(const Eigen::VectorXd data);
void createNewFile(const std::string &Filename);

/**
 * @brief Writes a matrix to a text file.
 */
void writeMatrixToFile(const Eigen::MatrixXd &matrix,
                       const std::string &filename);

/**
 * @brief Creates a new directory if it noes not already exist
 */
void createNewDirectory(const std::string &directory, bool overwrite = false);

/**
 * @brief Retrieves all files in a given directory with a specific extension.
 */
std::vector<std::string> getFilesInDirectory(const std::string &directory,
                                             const std::string &extension = "");
