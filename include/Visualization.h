#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/opencv.hpp>

class Visualizer
{
public:
    /**
     * @brief The window is named "Image Visualizer" and is not created until showImage() is called.
     * @param showFPS If true, we will overlay the FPS on the image whenever showImage() is called.
     */
    Visualizer(bool showFPS = true);

    /**
     * @brief Displays the image in the "Image Visualizer" window.
     *        If the image is empty, a warning is printed and nothing is shown.
     *        If showFPS is enabled, we overlay the instantaneous FPS on the top-left corner.
     *
     * @param image The cv::Mat (BGR color) image to display.
     */
    void showImage(const cv::Mat &image);

private:
    bool isWindowCreated_; ///< Tracks whether the window "Image Visualizer" has been created yet.

    // Added for optional FPS overlay
    bool showFPS_;   ///< Whether to overlay FPS on the image
    int64 lastTick_; ///< The tick count from cv::getTickCount() on the previous frame
};

#endif // VISUALIZATION_H
