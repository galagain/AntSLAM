#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <opencv2/opencv.hpp>
#include <vector>

class FeatureTracker
{
public:
    // Constructor
    FeatureTracker();

    // Track features between two images
    void trackFeatures(const cv::Mat &img1, const cv::Mat &img2,
                       std::vector<cv::Point2f> &points1,
                       std::vector<cv::Point2f> &points2,
                       std::vector<uchar> &status) const;

private:
    // Remove points that are poorly tracked or out of frame
    void removeInvalidPoints(std::vector<cv::Point2f> &points1,
                             std::vector<cv::Point2f> &points2,
                             std::vector<uchar> &status) const;
};

#endif // FEATURETRACKER_H
