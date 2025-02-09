#include "SlamSystem.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////
SlamSystem::SlamSystem(const std::string &datasetImagesPath,
                       const std::string &datasetPosesPath)
    : dataset_images_location_(datasetImagesPath), dataset_poses_location_(datasetPosesPath), motionEstimator_(718.8560, cv::Point2d(607.1928, 185.2157)), visualizer_(), visualizer3D_(nullptr), hasGroundTruth_(false)
{
    // If datasetPosesPath is not empty, we'll try to load ground truth in run().
    // Otherwise, we skip GT logic entirely.
}

////////////////////////////////////////////////////////////////////////////////
// run()
////////////////////////////////////////////////////////////////////////////////
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

    // 2) If dataset_poses_location_ is not empty, attempt GT load
    if (!dataset_poses_location_.empty())
    {
        groundTruthAll_ = dataLoader_.loadGroundPoses3D(dataset_poses_location_);
        if (!groundTruthAll_.empty())
        {
            hasGroundTruth_ = true;
            groundScales_ = loadAbsoluteScales(groundTruthAll_);
            groundTruth3D_.clear();
            std::cout << "Ground truth loaded: " << groundTruthAll_.size() << " poses." << std::endl;
        }
        else
        {
            std::cerr << "[Warning] Could not load ground truth at "
                      << dataset_poses_location_
                      << ". Running without GT." << std::endl;
            hasGroundTruth_ = false;
        }
    }
    else
    {
        // no path => skip
        std::cout << "[Info] No ground-truth file provided. Monocular only." << std::endl;
        hasGroundTruth_ = false;
    }

    // 3) Load the first two images
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
    featureTracker_.trackFeatures(img1, img2,
                                  prevFeatures_, currFeatures_,
                                  status_);

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

    // 7) If we have GT, add the first pose
    if (hasGroundTruth_ && !groundTruthAll_.empty())
    {
        groundTruth3D_.push_back(groundTruthAll_[0]);
    }

    // 8) Prepare for the loop
    prevImage_ = img2.clone();
    prevFeatures_ = currFeatures_;
    numFrame_ = 2;

    // ==========================
    // GLUT + Visualizer3D
    // ==========================
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Real-Time Visualization - Optional GT");

    visualizer3D_ = new Visualizer3D(&R_f_, &t_f_,
                                     &groundTruth3D_, // GT vector (empty if no GT)
                                     &predicted3D_);  // predicted
    visualizer3D_->initGL();
    visualizer3D_->registerCallbacks();

    // Main loop
    clock_t startClock = clock();
    while (true)
    {
        // If we have GT, we limit frames to min(totalFrames_, groundTruthAll_.size())
        // If no GT, groundTruthAll_ is empty => we limit frames to totalFrames_ only
        // (We also impose MAX_FRAME).
        int limit = (hasGroundTruth_) ? std::min(totalFrames_, (int)groundTruthAll_.size()) : totalFrames_;

        if (numFrame_ >= limit || numFrame_ >= MAX_FRAME)
            break;

        processOneFrame();

        // Update 3D
        glutMainLoopEvent();
        glutPostRedisplay();

        // Update 2D window
        int key = cv::waitKey(1);
        if (key == 27) // ESC
            break;
    }
    clock_t endClock = clock();
    double secs = double(endClock - startClock) / CLOCKS_PER_SEC;
    std::cout << "Total time: " << secs << " s" << std::endl;

    // Keep 3D window open
    while (true)
    {
        glutMainLoopEvent();
        glutPostRedisplay();

        if (!glutGetWindow())
            break;

        int k = cv::waitKey(10);
        if (k == 27)
            break;
    }

    delete visualizer3D_;
    visualizer3D_ = nullptr;
}

////////////////////////////////////////////////////////////////////////////////
// processOneFrame()
////////////////////////////////////////////////////////////////////////////////
void SlamSystem::processOneFrame()
{
    if (numFrame_ >= totalFrames_)
        return;

    // 1) Load next image
    cv::Mat currImage_c = dataLoader_.loadImage(images_[numFrame_]);
    if (currImage_c.empty())
        return;

    cv::Mat currGray;
    cv::cvtColor(currImage_c, currGray, cv::COLOR_BGR2GRAY);

    // 2) Track features
    featureTracker_.trackFeatures(prevImage_, currGray,
                                  prevFeatures_, currFeatures_,
                                  status_);

    // 3) Estimate motion
    cv::Mat R, t;
    motionEstimator_.estimateMotion(prevFeatures_, currFeatures_, R, t);

    // 4) If we have GT, apply scale. Else, skip scale => pure monocular
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
        // No GT => just do raw monocular
        t_f_ = t_f_ + (R_f_ * t);
        R_f_ = R * R_f_;
    }

    // 5) If too few features => detect again
    if (prevFeatures_.size() < MIN_NUM_FEAT)
    {
        featureDetector_.detectFeatures(prevImage_, prevFeatures_);
        featureTracker_.trackFeatures(prevImage_, currGray,
                                      prevFeatures_, currFeatures_,
                                      status_);
    }

    // 6) Store new predicted pose
    {
        float px = float(t_f_.at<double>(0));
        float py = float(t_f_.at<double>(1));
        float pz = float(t_f_.at<double>(2));
        predicted3D_.push_back(cv::Point3f(px, py, pz));
    }

    // 7) If we have GT, add it
    if (hasGroundTruth_ && numFrame_ < (int)groundTruthAll_.size())
    {
        groundTruth3D_.push_back(groundTruthAll_[numFrame_]);
    }

    // 8) Show 2D image
    visualizer_.showImage(currImage_c);

    // 9) Prepare next
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
