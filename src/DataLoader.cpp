/// @file DataLoader.cpp
/// @brief Implements the DataLoader class for loading images and ground-truth poses.

#include "DataLoader.h"
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>

// Static member for supported image extensions
const std::vector<std::string> DataLoader::imageExtensions = {".jpg", ".jpeg", ".png"};

/**
 * @brief Retrieves a sorted list of image file paths from a given directory.
 * @param folderPath Path to the folder containing images.
 * @return A vector of image file paths.
 */
std::vector<std::string> DataLoader::getImages(const std::string &folderPath) const
{
    std::vector<std::string> images;

    try
    {
        for (const auto &entry : std::filesystem::directory_iterator(folderPath))
        {
            if (entry.is_regular_file())
            {
                std::string extension = entry.path().extension().string();
                for (const auto &ext : imageExtensions)
                {
                    if (extension == ext)
                    {
                        images.push_back(entry.path().string());
                        break;
                    }
                }
            }
        }
        std::sort(images.begin(), images.end()); // Ensure chronological order
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading directory: " << e.what() << std::endl;
    }

    return images;
}

/**
 * @brief Loads an image from a given file path.
 * @param imagePath Path to the image file.
 * @return The loaded image as an OpenCV matrix.
 */
cv::Mat DataLoader::loadImage(const std::string &imagePath) const
{
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
    if (image.empty())
    {
        std::cerr << "Error: Could not load image from " << imagePath << std::endl;
    }
    return image;
}

/**
 * @brief Loads ground-truth 3D poses from a KITTI-style file.
 * @param filePath Path to the ground-truth file.
 * @return A vector of 3D points representing the trajectory.
 */
std::vector<cv::Point3f> DataLoader::loadGroundPoses3D(const std::string &filePath) const
{
    std::vector<cv::Point3f> poses3D;
    std::ifstream infile(filePath);
    if (!infile.is_open())
    {
        std::cerr << "Error: Cannot open poses file " << filePath << std::endl;
        return poses3D;
    }

    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        double val, tx = 0, ty = 0, tz = 0;

        for (int i = 0; i < 12; i++)
        {
            iss >> val;
            if (i == 3)
                tx = val;
            if (i == 7)
                ty = val;
            if (i == 11)
                tz = val;
        }

        poses3D.emplace_back(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
    }

    return poses3D;
}
