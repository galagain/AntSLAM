/// @file SlamSystem.h
/// @brief Defines the SlamSystem class for handling the SLAM (Simultaneous Localization and Mapping) pipeline.

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
#include "UtilsTransform.h"

/**
 * @class SlamSystem
 * @brief A class encapsulating the VO/SLAM pipeline, with optional ground-truth usage
 *        and optional visualization.
 */
class SlamSystem
{
public:
    /**
     * @brief Constructor with optional ground truth path and visualization toggle.
     *
     * @param datasetImagesPath Path to the folder containing images.
     * @param datasetPosesPath  Optional path to ground truth file. If empty, no GT used.
     * @param enableVisualization Whether to show the 2D window (OpenCV) and 3D window (OpenGL).
     *                           If false, runs headless: no UI is created.
     * @param savePosesPath Path to save estimated poses (optional).
     */
    SlamSystem(const std::string &datasetImagesPath,
               const std::string &datasetPosesPath = "",
               bool enableVisualization = true,
               const std::string savePosesPath = "");

    /**
     * @brief Runs the SLAM pipeline.
     *
     * If visualization is enabled, the function will create a 3D window for rendering and show 2D images.
     * Otherwise, the system runs headless.
     *
     * @param argc Argument count for GLUT (optional).
     * @param argv Argument vector for GLUT (optional).
     */
    void run(int argc, char **argv);

    /**
     * @brief Saves the estimated pose (rotation and translation) to a file.
     *
     * @param R Rotation matrix.
     * @param t Translation vector.
     */
    void savePoseToFile(const cv::Mat &R, const cv::Mat &t);

private:
    // -----------------------------------------
    // Helper methods
    // -----------------------------------------

    /**
     * @brief Processes a single frame, handling feature detection, tracking, and motion estimation.
     */
    void processOneFrame();

    /**
     * @brief Loads absolute scale values from ground-truth poses.
     *
     * @param gt Vector of ground-truth 3D points.
     * @return A vector containing absolute scale values.
     */
    std::vector<double> loadAbsoluteScales(const std::vector<cv::Point3f> &gt);

private:
    // -----------------------------------------
    // Data paths
    // -----------------------------------------

    std::string dataset_images_location_; ///< Path to the dataset containing images.
    std::string dataset_poses_location_;  ///< Path to the ground-truth poses file.

    std::string save_poses_path_; ///< Path to save estimated poses.

    // -----------------------------------------
    // Visualization toggle
    // -----------------------------------------

    bool enableVisualization_; ///< Flag to enable or disable visualization.

    // -----------------------------------------
    // Pipeline components
    // -----------------------------------------

    DataLoader dataLoader_;           ///< Module for loading images and ground-truth data.
    FeatureDetector featureDetector_; ///< Module for detecting features in images.
    FeatureTracker featureTracker_;   ///< Module for tracking features across frames.
    MotionEstimator motionEstimator_; ///< Module for estimating motion from feature correspondences.
    Visualizer visualizer_;           ///< Module for 2D visualization.

    // -----------------------------------------
    // Image lists and counters
    // -----------------------------------------

    std::vector<std::string> images_; ///< List of image file paths in the dataset.
    int numFrame_ = 0;                ///< Current frame index.
    int totalFrames_ = 0;             ///< Total number of frames in the dataset.

    // -----------------------------------------
    // Global VO pose
    // -----------------------------------------

    cv::Mat R_f_; ///< Estimated global rotation matrix.
    cv::Mat t_f_; ///< Estimated global translation vector.

    // -----------------------------------------
    // Feature tracking
    // -----------------------------------------

    cv::Mat prevImage_;                     ///< Previous frame image.
    cv::Mat currImage_;                     ///< Current frame image.
    std::vector<cv::Point2f> prevFeatures_; ///< Feature points detected in the previous frame.
    std::vector<cv::Point2f> currFeatures_; ///< Feature points detected in the current frame.
    std::vector<uchar> status_;             ///< Status vector for feature tracking.

    // -----------------------------------------
    // Ground truth: full vs. incremental
    // -----------------------------------------
    // If no ground truth is provided, these remain empty,
    // and we skip all scale-related logic.

    bool hasGroundTruth_ = false;             ///< Indicates if ground-truth data is available.
    std::vector<cv::Point3f> groundTruthAll_; ///< Complete ground-truth trajectory.
    std::vector<cv::Point3f> groundTruth3D_;  ///< Incremental ground-truth trajectory.
    std::vector<double> groundScales_;        ///< Precomputed scale values from ground truth.

    // -----------------------------------------
    // Predicted 3D (VO)
    // -----------------------------------------

    std::vector<cv::Point3f> predicted3D_; ///< Predicted 3D trajectory from visual odometry.

    // -----------------------------------------
    // 3D Visualizer
    // -----------------------------------------

    Visualizer3D *visualizer3D_; ///< 3D visualization module.

    // -----------------------------------------
    // Constants
    // -----------------------------------------

    static constexpr int MAX_FRAME = 2000;    ///< Maximum number of frames to process.
    static constexpr int MIN_NUM_FEAT = 2000; ///< Minimum number of features required for tracking.
};

#endif // SLAM_SYSTEM_H
