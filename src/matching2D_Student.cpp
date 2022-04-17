#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType;
        if (descriptorType.compare("DES_BINARY") == 0)
        {
            normType = cv::NORM_HAMMING;
        }
        else if (descriptorType.compare("DES_HOG") == 0)
        {
            normType = cv::NORM_L2;
        }
        else
        {
            cerr << "Invalid descriptor type: " << descriptorType << ". Only options are: 'DES_BINARY' or 'DES_HOG' " << endl;
        }
        double t = (double)cv::getTickCount();
        matcher = cv::BFMatcher::create(normType, crossCheck);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << matcherType << " matcher took " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {

        if (descriptorType.compare("DES_BINARY") == 0)
        {
            // FLANN change to work with HAMMING Norm, because it uses L2 Norm by default
            // reference: https://stackoverflow.com/questions/43830849/opencv-use-flann-with-orb-descriptors-to-match-features
            matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        }
        else if (descriptorType.compare("DES_HOG") == 0)
        {
            matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        }
        else
        {
            cerr << "Invalid descriptor type: " << descriptorType << ". Only options are: 'DES_BINARY' or 'DES_HOG' " << endl;
        }
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << matcherType << " matcher "
        //      << "with " << selectorType << " algorithm took " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << matcherType << " matcher "
        //      << "with " << selectorType << " algorithm took " << 1000 * t / 1.0 << " ms" << endl;

        // filter matches using K-Neares Neighbor Distance Ratio (NNDR)
        double minDescDistRatio_thresh = 0.8; // matches above this tresh are very similar and thus umbiguous, hence removed.s
        for (std::vector<std::vector<cv::DMatch>>::iterator it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            std::vector<cv::DMatch> curr_k_neighbors_pair = *it;
            if (curr_k_neighbors_pair.size() != 2) // sanity check in case knnMatch does not found a pair.
            {
                continue;
            }

            if ((curr_k_neighbors_pair[0].distance / curr_k_neighbors_pair[1].distance) < minDescDistRatio_thresh)
            {
                matches.push_back(curr_k_neighbors_pair[0]);
            }
        }
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    /*
        This function inplements various keypoint descriptos in the following list:
            - BRISK
            - BRIEF
            - ORB
            - FREAK
            - AKAZE
            - SIFT

    */
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {
        /* Parameters for BRISK descriptor */
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
        /* descriptor creation */
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        /* Parameters for BRIEF descriptor */

        int len_in_bytes = 32;        // length of the descriptor in bytes, valid values are: 16, 32 (default) or 64.
        bool use_orientation = false; // sample patterns using keypoints orientation, disabled by default.

        /* descriptor creation */
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(len_in_bytes, use_orientation);
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        /*
            ORB detector uses a modified version of BRIEF called rBRIEF algorithm with some additional features to account for keypoint orientation
             more info can be found here: https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html
        */

        /* Parameters for ORB descriptor */

        int nfeatures = 500;           // The maximum number of features to retain.
        float scaleFactor = 1.2f;      // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid
        int nlevels = 8;               // The number of pyramid levels.
        int edgeThreshold = 31;        // This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
        int firstLevel = 0;            // The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
        int WTA_K = 2;                 // The number of points that produce each element of the oriented BRIEF descriptor
        cv::ORB::ScoreType scoreType = // The default HARRIS_SCORE means that Harris algorithm is used to rank features (used to retain best nfeatures)
            cv::ORB::HARRIS_SCORE;     // FAST_SCORE is alternative value that produces slightly less stable keypoints, but it is a little faster to compute.
        int patchSize = 31;            // Size of the patch used by the oriented BRIEF descriptor.
        int fastThreshold = 20;        // the FAST threshold

        /* descriptor creation */
        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        /* Parameters for FREAK descriptor */
        bool use_orientationNormalized = true; // Enable orientation normalization.
        bool use_scaleNormalized = true;       // Enable scale normalization.
        float patternScale = 22.0f;            // Scaling of the description pattern.
        int nOctaves = 4;                      // Number of pyramid octaves covered by the detected keypoints.

        /* descriptor creation */
        extractor = cv::xfeatures2d::FREAK::create(use_orientationNormalized, use_scaleNormalized, patternScale, nOctaves);
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        /* Parameters for AKAZE descriptor */
        cv::AKAZE::DescriptorType descriptor_type = // Type of the extracted descriptor: DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT,
            cv::AKAZE::DESCRIPTOR_MLDB;             // DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.
        int descriptor_size = 0;                    // Size of the descriptor in bits. 0 -> Full size
        int descriptor_channels = 3;                // Number of channels in the descriptor (1, 2, 3)
        float threshold = 0.001f;                   // Detector response threshold to accept point
        int nOctaves = 4;                           // Maximum octave evolution of the image
        int nOctaveLayers = 4;                      // Default number of sublevels per scale level
        cv::KAZE::DiffusivityType diffusivity =     // Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER
            cv::KAZE::DIFF_PM_G2;

        /* descriptor creation */
        extractor = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        /*
            SIFT detector uses Histogram of Oriented Gradients (HOG) algorithm to describe keypoints.
            more info can be found here: https://docs.opencv.org/4.x/d7/d60/classcv_1_1SIFT.html
        */

        /* Parameters from SIFT descriptor */

        int nfeatures = 0;            // Number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast)
        int nOctaveLayers = 3;        // The number of layers in each octave. The number of octaves is computed automatically from the image resolution.
        double contrastThresh = 0.04; // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.
        double edgeThreshold = 10;    // The threshold used to filter out edge-like features. The larger the edgeThreshold, the less features are filtered out (more features are retained).
        double sigma = 1.6;           // The sigma of the Gaussian applied to the input image at the octave #0.

        /* descriptor creation */
        extractor = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThresh, edgeThreshold, sigma);
    }
    else
    {
        cerr << "Invalid " << descriptorType << ". Only BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT are supported descriptors" << endl;
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << "Shi-Tomasi detector with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using the traditional Harris detector
void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;                   //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    int apertureSize = 3;                // Aperture parameter for the Sobel operator.
    double k = 0.04;                     // Harris detector empirical parameter constant which is usually between [0.04, 0.06].
    int borderType = cv::BORDER_DEFAULT; // Pixel extrapolation method
    int minResponse = 70;                // minimum value for a corner in the 8bit scaled response matrix
    double maxOverlap = 0.1;             // max. permissible overlap between two features in ratio [0, 1], used during non-maxima suppression

    // Apply corner detection
    cv::Mat dst, dst_norm;
    double t = (double)cv::getTickCount();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, borderType);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat()); // Normalize to keep values between floats [0 and 255]
    double min, max;
    cv::minMaxLoc(dst_norm, &min, &max);
    // add corners to result vector
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << "Harris detector with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using modern detectors
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    /*
        This function inplements various modern keypoint detectors in the following list:
            - FAST
            - BRISK
            - ORB
            - AKAZE
            - SIFT
    */

    cv::Ptr<cv::FeatureDetector> selected_detector; // Generic feature detector pointer to fill the selected detectorType

    if (detectorType.compare("FAST") == 0)
    {
        /* Parameters from FAST detector */

        int thresh = 30;                                  // threshold parameter to control how different the neighbors are from a proposed pixel to be considered keypoint.
        bool nms = true;                                  // boolean to set if we want to apply non max suppression.
        cv::FastFeatureDetector::DetectorType detecType = // Type of neighborhood to create TYPE_9_16 (16 pixels around the analyzed pixel and 9 are required to be considered
            cv::FastFeatureDetector::TYPE_9_16;           // a keypoint), TYPE_7_12, TYPE_5_8

        /* detector creation */
        selected_detector = cv::FastFeatureDetector::create(thresh, nms, detecType);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        /* Parameters from BRISK detector */
        /* BRISK detector uses AGAST (Adaptive and Generic Accelerated Segment Test) */

        int thresh = 30;           // AGAST detection threshold score.
        int octaves = 3;           // Image pyramid octaves. Image is blurred with different coefficients to \
                            obtain an octave space. if 3 then 3 downsamples are done, 0 means no downsample (original image)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        /* detector creation */
        selected_detector = cv::BRISK::create(thresh, octaves, patternScale);
    }
    else if (detectorType.compare("ORB") == 0)
    {

        /*
            ORB detector uses FAST algorithm with some additional features to be scale and rotation invariant
             more info can be found here: https://docs.opencv.org/3.4/db/d95/classcv_1_1ORB.html
        */

        /* Parameters from ORB detector */

        int nfeatures = 500;           // The maximum number of features to retain.
        float scaleFactor = 1.2f;      // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid
        int nlevels = 8;               // The number of pyramid levels.
        int edgeThreshold = 31;        // This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
        int firstLevel = 0;            // The level of pyramid to put source image to. Previous layers are filled with upscaled source image.
        int WTA_K = 2;                 // The number of points that produce each element of the oriented BRIEF descriptor
        cv::ORB::ScoreType scoreType = // The default HARRIS_SCORE means that Harris algorithm is used to rank features (used to retain best nfeatures)
            cv::ORB::HARRIS_SCORE;     // FAST_SCORE is alternative value that produces slightly less stable keypoints, but it is a little faster to compute.
        int patchSize = 31;            // Size of the patch used by the oriented BRIEF descriptor.
        int fastThreshold = 20;        // the FAST threshold

        /* detector creation */
        selected_detector = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        /* Parameters from AKAZE detector */

        cv::AKAZE::DescriptorType descriptor_type = // Type of the extracted descriptor: DESCRIPTOR_KAZE, DESCRIPTOR_KAZE_UPRIGHT,
            cv::AKAZE::DESCRIPTOR_MLDB;             // DESCRIPTOR_MLDB or DESCRIPTOR_MLDB_UPRIGHT.
        int descriptor_size = 0;                    // Size of the descriptor in bits. 0 -> Full size
        int descriptor_channels = 3;                // Number of channels in the descriptor (1, 2, 3)
        float threshold = 0.001f;                   // Detector response threshold to accept point
        int nOctaves = 4;                           // Maximum octave evolution of the image
        int nOctaveLayers = 4;                      // Default number of sublevels per scale level
        cv::KAZE::DiffusivityType diffusivity =     // Diffusivity type. DIFF_PM_G1, DIFF_PM_G2, DIFF_WEICKERT or DIFF_CHARBONNIER
            cv::KAZE::DIFF_PM_G2;

        /* detector creation */
        selected_detector = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);
    }
    else if (detectorType.compare("SIFT") == 0)
    {

        /*
            SIFT detector uses Laplacian of Gaussian (LoG) algorithm to detect features at various scales.
            more info can be found here: https://docs.opencv.org/4.x/d7/d60/classcv_1_1SIFT.html
        */

        /* Parameters from SIFT detector */

        int nfeatures = 0;            // Number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast)
        int nOctaveLayers = 3;        // The number of layers in each octave. The number of octaves is computed automatically from the image resolution.
        double contrastThresh = 0.04; // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.
        double edgeThreshold = 10;    // The threshold used to filter out edge-like features. The larger the edgeThreshold, the less features are filtered out (more features are retained).
        double sigma = 1.6;           // The sigma of the Gaussian applied to the input image at the octave #0.

        /* detector creation */
        selected_detector = cv::SIFT::create(nfeatures, nOctaveLayers, contrastThresh, edgeThreshold, sigma);
    }
    else
    {
        cerr << "Invalid " << detectorType << ". Only FAST, BRISK, ORB, AKAZE and SIFT are supported from modern detectors" << endl;
    }

    /* Detection part */
    double t = (double)cv::getTickCount();
    selected_detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << detectorType << " detector with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}