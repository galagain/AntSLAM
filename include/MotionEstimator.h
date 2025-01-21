#ifndef MOTIONESTIMATOR_H
#define MOTIONESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

class MotionEstimator
{
public:
    // Constructor
    MotionEstimator(double focal, const cv::Point2d &pp);

    // Estimate motion (R, t) from corresponding points
    void estimateMotion(const std::vector<cv::Point2f> &points1,
                        const std::vector<cv::Point2f> &points2,
                        cv::Mat &R, cv::Mat &t) const;

private:
    double focal;   // Camera focal length
    cv::Point2d pp; // Principal point
};

#endif // MOTIONESTIMATOR_H
