#include "Visualization.h"
#include <iostream>

Visualizer::Visualizer()
    : isWindowCreated_(false)
{
    // By default, the window is not created here.
    // It will be created when the first image is shown.
}

void Visualizer::showImage(const cv::Mat &image)
{
    // If the provided image is empty, log a warning and do nothing.
    if (image.empty())
    {
        std::cerr << "[Warning] showImage() received an empty image." << std::endl;
        return;
    }

    // Create the window only once, at the size of the first image received.
    if (!isWindowCreated_)
    {
        cv::namedWindow("Image Visualizer", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image Visualizer", image.cols, image.rows);
        isWindowCreated_ = true;
    }

    // Display the image
    cv::imshow("Image Visualizer", image);
    // Wait briefly for the window to update
    cv::waitKey(1);
}
