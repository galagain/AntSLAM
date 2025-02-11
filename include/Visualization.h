/// @file Visualization.h
/// @brief Defines the Visualizer class for displaying images.

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/opencv.hpp>

/**
 * @class Visualizer
 * @brief Handles 2D visualization of images.
 */
class Visualizer
{
public:
    /**
     * @brief Constructs a Visualizer instance.
     * @param showFPS Whether to overlay FPS on displayed images.
     */
    Visualizer(bool showFPS = true);

    /**
     * @brief Displays an image in the visualization window.
     * @param image The image to display.
     */
    void showImage(const cv::Mat &image);

private:
    bool isWindowCreated_; ///< Indicates if the visualization window is created.
    bool showFPS_;         ///< Whether to overlay FPS on the image.
    int64 lastTick_;       ///< Last tick count for FPS calculation.
};

#endif // VISUALIZATION_H
