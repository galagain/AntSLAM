#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <opencv2/opencv.hpp>

class Visualizer
{
public:
    // The window is named "Image Visualizer" and is not created until showImage() is called
    Visualizer();

    // Displays the image in the "Image Visualizer" window
    void showImage(const cv::Mat &image);

private:
    bool isWindowCreated_;
};

#endif // VISUALIZATION_H
