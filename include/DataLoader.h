#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

class DataLoader
{
public:
    // Constructor
    DataLoader() = default;

    // Method to get the list of images from a specified folder path
    std::vector<std::string> getImages(const std::string &folderPath) const;

    // Method to load an image from a path
    cv::Mat loadImage(const std::string &imagePath) const;

private:
    // Valid image extensions
    static const std::vector<std::string> imageExtensions;
};

#endif // DATA_LOADER_H