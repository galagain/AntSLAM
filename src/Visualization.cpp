#include "Visualization.h"
#include <iostream>

/**
 * @brief Constructs the Visualizer.
 * By default, it does not show FPS unless 'showFPS' is set to true.
 *
 * @param showFPS If true, overlays the FPS text on the displayed image.
 */
Visualizer::Visualizer(bool showFPS)
    : isWindowCreated_(false),
      showFPS_(showFPS),
      lastTick_(0)
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

        // Initialize lastTick_ so we can measure FPS on subsequent calls
        lastTick_ = cv::getTickCount();
    }

    // We'll show the original image, possibly with FPS text drawn on top.
    // If you want to avoid modifying 'image', do a clone() here:
    cv::Mat displayImage = image; // or => image.clone();

    // If showFPS is enabled, compute the time since last call and overlay the FPS.
    if (showFPS_)
    {
        int64 currentTick = cv::getTickCount();
        double freq = cv::getTickFrequency();

        // Compute elapsed seconds between this frame and last frame
        double elapsedSec = double(currentTick - lastTick_) / freq;
        lastTick_ = currentTick; // update for next time

        // Avoid division by zero
        double fps = (elapsedSec > 0.0) ? (1.0 / elapsedSec) : 0.0;

        // Format the FPS string
        std::string fpsText = cv::format("FPS: %.2f", fps);

        // We'll put the text in the top-left corner (X=20, Y=30),
        // using green color and a scale of 1.0
        cv::putText(
            displayImage, fpsText,
            cv::Point(20, 30),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0, 255, 0), // B=0, G=255, R=0 => green
            2);
    }

    // Display the image
    cv::imshow("Image Visualizer", displayImage);

    // Wait briefly for the window to update
    cv::waitKey(1);
}
