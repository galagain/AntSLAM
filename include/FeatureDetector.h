#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureDetector
{
public:
    FeatureDetector(int fastThreshold = 20, bool nonmaxSuppression = true);

    // Detect features in an image
    void detectFeatures(const cv::Mat &img, std::vector<cv::Point2f> &points);

private:
    int fastThreshold;
    bool nonmaxSuppression;
};

#endif // FEATUREDETECTOR_H
