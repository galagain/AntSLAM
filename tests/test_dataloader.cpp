#include "DataLoader.h"
#include <iostream>

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <folder_path>" << std::endl;
        return 1;
    }

    std::string folderPath = argv[1];
    DataLoader dataLoader;

    // Test getImages function
    std::vector<std::string> images = dataLoader.getImages(folderPath);
    if (images.empty())
    {
        std::cout << "No images found in the folder: " << folderPath << std::endl;
    }
    else
    {
        std::cout << "Number of images found in folder: " << images.size() << std::endl;
    }

    // Test loadImage function
    if (!images.empty())
    {
        cv::Mat image = dataLoader.loadImage(images[0]);
        if (!image.empty())
        {
            std::cout << "Successfully loaded image: " << images[0] << std::endl;
        }
        else
        {
            std::cout << "Failed to load image: " << images[0] << std::endl;
        }
    }

    return 0;
}
