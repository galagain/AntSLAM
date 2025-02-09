#ifndef SLAM_SYSTEM_H
#define SLAM_SYSTEM_H

#include <GL/freeglut.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>

#include "DataLoader.h"
#include "FeatureDetector.h"
#include "FeatureTracker.h"
#include "MotionEstimator.h"
#include "Visualization.h"
#include "Visualizer3D.h"

/**
 * @class SlamSystem
 * @brief A class that encapsulates the VO/SLAM pipeline, with optional ground-truth usage.
 *
 * If the second parameter (posesPath) is empty, the system runs without ground truth,
 * so no scales are computed and no GT line is displayed.
 */
class SlamSystem
{
public:
    /**
     * @brief Constructor that can be used in two ways:
     *        1) SlamSystem(imagesPath);               // no ground truth
     *        2) SlamSystem(imagesPath, posesPath);    // with ground truth
     *
     * If posesPath is empty, we skip all GT-related logic.
     *
     * @param datasetImagesPath Path to the folder containing the images
     * @param datasetPosesPath  Path to the ground truth file (optional).
     */
    SlamSystem(const std::string &datasetImagesPath,
               const std::string &datasetPosesPath = "");

    /**
     * @brief run: the main function that initializes the pipeline and processes all frames.
     *
     * @param argc, argv: Typically forwarded to GLUT for window setup.
     */
    void run(int argc, char **argv);

private:
    // -----------------------------------------
    // Helper methods
    // -----------------------------------------
    void processOneFrame();
    std::vector<double> loadAbsoluteScales(const std::vector<cv::Point3f> &gt);

private:
    // -----------------------------------------
    // Data paths
    // -----------------------------------------
    std::string dataset_images_location_;
    std::string dataset_poses_location_;

    // -----------------------------------------
    // Pipeline components
    // -----------------------------------------
    DataLoader dataLoader_;
    FeatureDetector featureDetector_;
    FeatureTracker featureTracker_;
    MotionEstimator motionEstimator_;
    Visualizer visualizer_; ///< For 2D (OpenCV) display

    // -----------------------------------------
    // Image lists and counters
    // -----------------------------------------
    std::vector<std::string> images_;
    int numFrame_ = 0;
    int totalFrames_ = 0;

    // -----------------------------------------
    // Global VO pose
    // -----------------------------------------
    cv::Mat R_f_, t_f_;

    // -----------------------------------------
    // Feature tracking
    // -----------------------------------------
    cv::Mat prevImage_;
    cv::Mat currImage_;
    std::vector<cv::Point2f> prevFeatures_;
    std::vector<cv::Point2f> currFeatures_;
    std::vector<uchar> status_;

    // -----------------------------------------
    // Ground truth: full vs. incremental
    // If no ground truth is provided, these remain empty
    // and we skip all scale logic.
    // -----------------------------------------
    bool hasGroundTruth_ = false; ///< Tells us if we have a valid GT file
    std::vector<cv::Point3f> groundTruthAll_;
    std::vector<cv::Point3f> groundTruth3D_;
    std::vector<double> groundScales_;

    // -----------------------------------------
    // Predicted 3D (VO)
    // -----------------------------------------
    std::vector<cv::Point3f> predicted3D_;

    // -----------------------------------------
    // 3D Visualizer
    // -----------------------------------------
    Visualizer3D *visualizer3D_;

    // -----------------------------------------
    // Constants
    // -----------------------------------------
    static constexpr int MAX_FRAME = 2000;
    static constexpr int MIN_NUM_FEAT = 2000;
};

#endif // SLAM_SYSTEM_H
