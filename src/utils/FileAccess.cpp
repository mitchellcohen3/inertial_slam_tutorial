#include "utils/FileAccess.h"

#include <filesystem>
#include <glog/logging.h>
#include <fstream>
#include <sstream>

void writeDataToFile(const std::string &Filename, const Eigen::VectorXd data,
                     const bool Append, std::string delimiter) {
  std::ofstream File;

  if (Append) {
    File.open(Filename, std::ios::out | std::ios::app);
  } else {
    File.open(Filename, std::ios::out | std::ios::trunc);
  }

  if (!File.is_open()) {
    LOG(ERROR) << "Failed to open file: " << Filename;
    return;
  }

  // Write data to file
  File.precision(17);
  for (int i = 0; i < data.rows(); i++) {
    if (i == data.rows() - 1) {
      File << data(i) << std::endl;
    } else {
      File << data(i) << delimiter;
    }
  }
  File.close();
}

void writeStringToFile(const std::string &Filename, const std::string &ToWrite,
                       const bool Append) {
  std::ofstream File;

  if (Append) {
    File.open(Filename, std::ios::out | std::ios::app);
  } else {
    File.open(Filename, std::ios::out | std::ios::trunc);
  }

  File << ToWrite;
  File.close();
}

std::string eigenToString(const Eigen::VectorXd data) {
  std::stringstream ss;
  ss.precision(17);
  for (int i = 0; i < data.rows(); i++) {
    if (i == data.rows() - 1) {
      ss << data(i) << std::endl;
    } else {
      ss << data(i) << ", ";
    }
  }

  return ss.str();
}

void createNewFile(const std::string &Filename) {
  std::ofstream file(Filename, std::ios::out | std::ios::trunc);
  if (!file) {
    LOG(ERROR) << "Failed to create file: " << Filename;
    throw std::runtime_error("Failed to create file: " + Filename);
  }
}

void writeMatrixToFile(const Eigen::MatrixXd &matrix,
                       const std::string &filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    LOG(ERROR) << "Failed to open file: " << filename;
    return;
  }

  // Write the matrix data
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      file << matrix(i, j);
      if (j < matrix.cols() - 1) {
        file << ", ";
      }
    }
    file << "\n";
  }

  file.close();
}

void createNewDirectory(const std::string &directory, bool overwrite) {
  if (std::filesystem::exists(directory)) {
    if (overwrite) {
      try {
        LOG(INFO) << "Removing existing directory: " << directory;
        std::filesystem::remove_all(directory);
      } catch (const std::filesystem::filesystem_error &e) {
        LOG(ERROR) << "Failed to remove existing directory: " << directory
                   << " Error: " << e.what();
        throw std::runtime_error("Failed to remove existing directory: " +
                                 directory);
      }
    }
  }

  try {
    std::filesystem::create_directories(directory);
  } catch (const std::filesystem::filesystem_error &e) {
    LOG(ERROR) << "Failed to create directory: " << directory
               << " Error: " << e.what();
    throw std::runtime_error("Failed to create directory: " + directory);
  }
}

std::vector<std::string> getFilesInDirectory(const std::string &directory,
                                             const std::string &extension) {
  std::vector<std::string> files;
  try {
    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
      if (entry.is_regular_file() && entry.path().extension() == extension) {
        files.push_back(entry.path().string());
      }
    }
  } catch (const std::filesystem::filesystem_error &e) {
    LOG(ERROR) << "Failed to read directory: " << directory
               << " Error: " << e.what();
    throw std::runtime_error("Failed to read directory: " + directory);
  }

  return files;
}