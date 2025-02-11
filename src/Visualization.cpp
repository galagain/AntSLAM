/// @file Visualization.cpp
/// @brief Implements the Visualizer class for displaying images with optional FPS overlay.

#include "Visualization.h"
#include <iostream>

/**
 * @brief Constructs the Visualizer.
 *
 * By default, it does not display FPS unless `showFPS` is set to `true`.
 * The visualization window is not created immediately; it will be initialized
 * when the first image is displayed.
 *
 * @param showFPS If `true`, overlays the FPS text on the displayed image.
 */
Visualizer::Visualizer(bool showFPS)
    : isWindowCreated_(false),
      showFPS_(showFPS),
      lastTick_(0)
{
}

/**
 * @brief Displays an image in the visualization window.
 *
 * If the window does not exist, it is created with the size of the first received image.
 * If FPS overlay is enabled, it calculates and displays the FPS on top of the image.
 *
 * @param image The input image to be displayed.
 */
void Visualizer::showImage(const cv::Mat &image)
{
    // Check if the input image is empty
    if (image.empty())
    {
        std::cerr << "[Warning] showImage() received an empty image." << std::endl;
        return;
    }

    // Create the window on the first call
    if (!isWindowCreated_)
    {
        cv::namedWindow("Image Visualizer", cv::WINDOW_NORMAL);
        cv::resizeWindow("Image Visualizer", image.cols, image.rows);
        isWindowCreated_ = true;

        // Initialize lastTick_ for FPS calculation
        lastTick_ = cv::getTickCount();
    }

    // Create a copy of the input image if necessary
    cv::Mat displayImage = image; // If modifications are needed, use `image.clone()`

    // Overlay FPS if enabled
    if (showFPS_)
    {
        int64 currentTick = cv::getTickCount();
        double freq = cv::getTickFrequency();

        // Compute elapsed time between frames
        double elapsedSec = static_cast<double>(currentTick - lastTick_) / freq;
        lastTick_ = currentTick; // Update timestamp for next frame

        // Compute FPS, ensuring no division by zero
        double fps = (elapsedSec > 0.0) ? (1.0 / elapsedSec) : 0.0;

        // Format FPS display text
        std::string fpsText = cv::format("FPS: %.2f", fps);

        // Draw FPS text on the top-left corner
        cv::putText(
            displayImage, fpsText,
            cv::Point(20, 30),        // Position (x=20, y=30)
            cv::FONT_HERSHEY_SIMPLEX, // Font style
            1.0,                      // Font scale
            cv::Scalar(0, 255, 0),    // Green text color
            2);                       // Thickness
    }

    // Show the image in the visualization window
    cv::imshow("Image Visualizer", displayImage);

    // Wait a short time for the window to update
    cv::waitKey(1);
}
