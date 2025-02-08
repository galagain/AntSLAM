#include "DataLoader.h"
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <fstream> // For reading files
#include <sstream> // For parsing lines

// Static member for image extensions
const std::vector<std::string> DataLoader::imageExtensions = {
    ".jpg", ".jpeg", ".png"};

// Method to get the list of images
std::vector<std::string> DataLoader::getImages(const std::string &folderPath) const
{
    std::vector<std::string> images;

    try
    {
        // Iterate through the folder
        for (const auto &entry : std::filesystem::directory_iterator(folderPath))
        {
            // Check if it's a regular file
            if (entry.is_regular_file())
            {
                // Get the file extension
                std::string extension = entry.path().extension().string();
                for (const auto &ext : imageExtensions)
                {
                    // Check if it matches an image extension
                    if (extension == ext)
                    {
                        // Add the file path to the list
                        images.push_back(entry.path().string());
                        break;
                    }
                }
            }
        }

        // Sort the images alphabetically
        std::sort(images.begin(), images.end());
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading directory: " << e.what() << std::endl;
    }

    return images;
}

// Method to load an image
cv::Mat DataLoader::loadImage(const std::string &imagePath) const
{
    // Use OpenCV to load the image
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

    if (image.empty())
    {
        std::cerr << "Error: Could not load image from " << imagePath << std::endl;
    }

    return image;
}

// Method to load ground poses (KITTI style) in 3D
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
        double val;
        double tx = 0, ty = 0, tz = 0;

        // Each line has 12 values describing a 3x4 matrix: R(3x3) + t(3x1)
        // Indices 3, 7, and 11 correspond to tx, ty, tz.
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

        poses3D.push_back(cv::Point3f(static_cast<float>(tx),
                                      static_cast<float>(ty),
                                      static_cast<float>(tz)));
    }

    return poses3D;
}