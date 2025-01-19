#include "DataLoader.h"
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <opencv2/opencv.hpp>

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
