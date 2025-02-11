/// @file DataLoader.h
/// @brief Defines the DataLoader class for loading images and ground-truth poses.

#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

/**
 * @class DataLoader
 * @brief Handles loading images and ground-truth data from disk.
 */
class DataLoader
{
public:
    /// @brief Default constructor.
    DataLoader() = default;

    /**
     * @brief Retrieves a list of image file paths from a specified directory.
     * @param folderPath Directory containing images.
     * @return A vector of strings representing image file paths.
     */
    std::vector<std::string> getImages(const std::string &folderPath) const;

    /**
     * @brief Loads an image from a given file path.
     * @param imagePath Path to the image file.
     * @return The loaded image as an OpenCV matrix.
     */
    cv::Mat loadImage(const std::string &imagePath) const;

    /**
     * @brief Loads 3D ground-truth poses from a file in KITTI format.
     * @param filePath Path to the ground-truth file.
     * @return A vector of 3D points representing the poses.
     */
    std::vector<cv::Point3f> loadGroundPoses3D(const std::string &filePath) const;

private:
    /// List of valid image file extensions.
    static const std::vector<std::string> imageExtensions;
};

#endif // DATA_LOADER_H
