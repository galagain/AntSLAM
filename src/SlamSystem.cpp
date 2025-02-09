#include "SlamSystem.h"

// Constructor
SlamSystem::SlamSystem(const std::string &datasetImagesPath,
                       const std::string &datasetPosesPath,
                       bool enableVisualization)
    : dataset_images_location_(datasetImagesPath), dataset_poses_location_(datasetPosesPath), enableVisualization_(enableVisualization), motionEstimator_(718.8560, cv::Point2d(607.1928, 185.2157)), visualizer_(), hasGroundTruth_(false), visualizer3D_(nullptr)
{
    // If datasetPosesPath is not empty, we might load GT in run().
    // Otherwise, no GT is used. Visualization depends on 'enableVisualization_'.
}

void SlamSystem::run(int argc, char **argv)
{
    // 1) Load images
    images_ = dataLoader_.getImages(dataset_images_location_);
    totalFrames_ = (int)images_.size();
    if (totalFrames_ < 2)
    {
        std::cerr << "Not enough images found in " << dataset_images_location_ << std::endl;
        return;
    }

    // 2) Attempt ground-truth load if not empty
    if (!dataset_poses_location_.empty())
    {
        groundTruthAll_ = dataLoader_.loadGroundPoses3D(dataset_poses_location_);
        if (!groundTruthAll_.empty())
        {
            hasGroundTruth_ = true;
            groundScales_ = loadAbsoluteScales(groundTruthAll_);
            groundTruth3D_.clear();
            std::cout << "GT loaded successfully. Number of poses: " << groundTruthAll_.size() << std::endl;
        }
        else
        {
            std::cerr << "[Warning] Could not load ground truth. Running without GT.\n";
            hasGroundTruth_ = false;
        }
    }
    else
    {
        hasGroundTruth_ = false;
    }

    // 3) Load first two images
    cv::Mat img1_c = dataLoader_.loadImage(images_[0]);
    cv::Mat img2_c = dataLoader_.loadImage(images_[1]);
    if (img1_c.empty() || img2_c.empty())
    {
        std::cerr << "Error reading the first two images.\n";
        return;
    }

    cv::Mat img1, img2;
    cv::cvtColor(img1_c, img1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img2_c, img2, cv::COLOR_BGR2GRAY);

    // 4) Detect & track initial features
    featureDetector_.detectFeatures(img1, prevFeatures_);
    featureTracker_.trackFeatures(img1, img2, prevFeatures_, currFeatures_, status_);

    // 5) Estimate first motion
    {
        cv::Mat R, t;
        motionEstimator_.estimateMotion(prevFeatures_, currFeatures_, R, t);
        R_f_ = R.clone();
        t_f_ = t.clone();
    }

    // 6) Store the first predicted pose
    {
        float px = float(t_f_.at<double>(0));
        float py = float(t_f_.at<double>(1));
        float pz = float(t_f_.at<double>(2));
        predicted3D_.push_back(cv::Point3f(px, py, pz));
    }

    // 7) If GT is available, add the first GT pose
    if (hasGroundTruth_ && !groundTruthAll_.empty())
    {
        groundTruth3D_.push_back(groundTruthAll_[0]);
    }

    // 8) Prepare for main loop
    prevImage_ = img2.clone();
    prevFeatures_ = currFeatures_;
    numFrame_ = 2;

    // ---------------------------
    // Visualization Setup
    // ---------------------------
    if (enableVisualization_)
    {
        // Initialize GLUT for 3D, create window
        glutInit(&argc, argv);
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
        glutInitWindowSize(800, 600);
        glutCreateWindow("3D Real-Time Visualization - optional");

        // Create a 3D viewer
        visualizer3D_ = new Visualizer3D(
            &R_f_, &t_f_,
            &groundTruth3D_, // might be empty if no GT
            &predicted3D_);
        visualizer3D_->initGL();
        visualizer3D_->registerCallbacks();
    }

    // Main loop
    clock_t startClock = clock();
    while (true)
    {
        // Limit frames with or without GT
        int limit = (hasGroundTruth_) ? std::min(totalFrames_, (int)groundTruthAll_.size()) : totalFrames_;

        if (numFrame_ >= limit || numFrame_ >= MAX_FRAME)
            break;

        processOneFrame();

        // Update windows if enabled
        if (enableVisualization_)
        {
            glutMainLoopEvent();
            glutPostRedisplay();

            // Update 2D window
            int key = cv::waitKey(1);
            if (key == 27) // ESC
                break;
        }
        else
        {
            // If no visualization, we can do a minimal wait or no wait
            // to prevent a tight CPU loop
            // e.g., std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    clock_t endClock = clock();
    double secs = double(endClock - startClock) / CLOCKS_PER_SEC;
    std::cout << "Total time: " << secs << " s" << std::endl;

    // Keep 3D window open (only if we created it)
    if (enableVisualization_ && visualizer3D_)
    {
        while (true)
        {
            glutMainLoopEvent();
            glutPostRedisplay();

            if (!glutGetWindow())
                break;

            int k = cv::waitKey(10);
            if (k == 27) // ESC
                break;
        }

        delete visualizer3D_;
        visualizer3D_ = nullptr;
    }
}

void SlamSystem::processOneFrame()
{
    if (numFrame_ >= totalFrames_)
        return;

    cv::Mat currImage_c = dataLoader_.loadImage(images_[numFrame_]);
    if (currImage_c.empty())
        return;

    cv::Mat currGray;
    cv::cvtColor(currImage_c, currGray, cv::COLOR_BGR2GRAY);

    // Track
    featureTracker_.trackFeatures(prevImage_, currGray,
                                  prevFeatures_, currFeatures_,
                                  status_);

    // Estimate motion
    cv::Mat R, t;
    motionEstimator_.estimateMotion(prevFeatures_, currFeatures_, R, t);

    // If GT => scale. Otherwise, raw monocular
    if (hasGroundTruth_ && numFrame_ < (int)groundScales_.size())
    {
        double scale = groundScales_[numFrame_];
        if ((scale > 0.1) &&
            (t.at<double>(2) > t.at<double>(0)) &&
            (t.at<double>(2) > t.at<double>(1)))
        {
            t_f_ = t_f_ + scale * (R_f_ * t);
            R_f_ = R * R_f_;
        }
    }
    else
    {
        // No GT => no scale
        t_f_ = t_f_ + (R_f_ * t);
        R_f_ = R * R_f_;
    }

    // If too few features => detect new
    if (prevFeatures_.size() < MIN_NUM_FEAT)
    {
        featureDetector_.detectFeatures(prevImage_, prevFeatures_);
        featureTracker_.trackFeatures(prevImage_, currGray,
                                      prevFeatures_, currFeatures_,
                                      status_);
    }

    // Store predicted pose
    {
        float px = float(t_f_.at<double>(0));
        float py = float(t_f_.at<double>(1));
        float pz = float(t_f_.at<double>(2));
        predicted3D_.push_back(cv::Point3f(px, py, pz));
    }

    // If GT => add incremental
    if (hasGroundTruth_ && numFrame_ < (int)groundTruthAll_.size())
    {
        groundTruth3D_.push_back(groundTruthAll_[numFrame_]);
    }

    // Show 2D image only if visualization is enabled
    if (enableVisualization_)
    {
        visualizer_.showImage(currImage_c);
    }

    prevImage_ = currGray.clone();
    prevFeatures_ = currFeatures_;

    numFrame_++;
}

////////////////////////////////////////////////////////////////////////////////
// loadAbsoluteScales()
////////////////////////////////////////////////////////////////////////////////
std::vector<double> SlamSystem::loadAbsoluteScales(const std::vector<cv::Point3f> &gt)
{
    std::vector<double> scales;
    if (gt.empty())
        return scales;

    scales.push_back(0.0);
    for (size_t i = 1; i < gt.size(); i++)
    {
        float dx = gt[i].x - gt[i - 1].x;
        float dy = gt[i].y - gt[i - 1].y;
        float dz = gt[i].z - gt[i - 1].z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        scales.push_back(dist);
    }
    return scales;
}
