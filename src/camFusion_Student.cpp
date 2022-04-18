
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/*
 * The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
 * However, you can make this function work for other sizes too.
 * For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dt = 1.0 / frameRate;
    double min_prev = removeOutliers_and_get_min(lidarPointsPrev);
    double min_curr = removeOutliers_and_get_min(lidarPointsCurr);
    double dist = abs(min_prev - min_curr);
    // Calculate TTC assuming a constant velocity V = dist/dt
    TTC = min_curr * dt / dist;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    // Match keypoints to a bounding box
    map<int, int> kp_bb_matches_prevFrame;
    map<int, int> kp_bb_matches_currFrame;
    matchKeypointsWithBoundingBoxes(prevFrame, kp_bb_matches_prevFrame);
    matchKeypointsWithBoundingBoxes(currFrame, kp_bb_matches_currFrame);

    // count matches per bounding box
    map<pair<int, int>, int> bb_match_counter;
    countBB_matches(prevFrame, kp_bb_matches_prevFrame, currFrame, kp_bb_matches_currFrame, matches, bb_match_counter);

    // fill best bounding boxes matches with the bounding boxes that have more matches
    fill_bbBestMatches(bb_match_counter, bbBestMatches);
}

/* **************************************** */
/* Below this line are all helper functions */
/* **************************************** */

void matchKeypointsWithBoundingBoxes(DataFrame inputFrame, map<int, int> &kp_bb_idxs)
{
    /*
        DataFrame inputFrame: a dataframe that contains keypoints (traditional detection) and bounding boxes (YOLO)
        map<int,int> kp_bb_idxs: contains <keypoint index: boxID> match
     */
    auto kps = inputFrame.keypoints;
    auto bbxs = inputFrame.boundingBoxes;
    for (int ind = 0; ind < kps.size(); ind++)
    {
        for (auto box_i : bbxs)
        {
            if (box_i.roi.contains(kps[ind].pt))
            {
                kp_bb_idxs.insert(pair<int, int>(ind, box_i.boxID));
            }
        }
    }
}

void countBB_matches(DataFrame prevFrame, map<int, int> kp_bb_idxs_prevFrame, DataFrame currFrame, map<int, int> kp_bb_idxs_currFrame, vector<cv::DMatch> matches, map<pair<int, int>, int> &bb_match_counter)
{
    for (auto dm : matches)
    {
        int prev_ind = dm.queryIdx;
        int curr_ind = dm.trainIdx;
        map<int, int>::iterator prev_iter = kp_bb_idxs_prevFrame.find(prev_ind);
        map<int, int>::iterator curr_iter = kp_bb_idxs_currFrame.find(curr_ind);

        // only evaluate matches that have both matches in a bounding box
        if (prev_iter != kp_bb_idxs_prevFrame.end() && curr_iter != kp_bb_idxs_currFrame.end())
        {
            int prev_boxID = kp_bb_idxs_prevFrame.at(prev_ind);
            int curr_boxID = kp_bb_idxs_currFrame.at(curr_ind);

            map<pair<int, int>, int>::iterator map_iter = bb_match_counter.find(make_pair(prev_boxID, curr_boxID));
            if (map_iter == bb_match_counter.end()) // if new value insert first value
            {
                bb_match_counter.insert(pair<pair<int, int>, int>(make_pair(prev_boxID, curr_boxID), 1));
            }
            else
            {
                bb_match_counter[make_pair(prev_boxID, curr_boxID)]++;
            }
        }
    }
}

void fill_bbBestMatches(map<pair<int, int>, int> bb_match_counter, map<int, int> &bbBestMatches)
{
    map<int, int> prev_boxID_max;
    for (auto const &map_it : bb_match_counter)
    {
        int prev_boxID = map_it.first.first;
        int curr_boxID = map_it.first.second;
        int counter = map_it.second;
        map<int, int>::iterator it = prev_boxID_max.find(prev_boxID);
        if (it == prev_boxID_max.end())
        {
            prev_boxID_max.insert(pair<int, int>(prev_boxID, counter));
            bbBestMatches[prev_boxID] = curr_boxID;
        }
        else
        {
            if (counter > prev_boxID_max[prev_boxID])
            {
                prev_boxID_max[prev_boxID] = counter;
                bbBestMatches[prev_boxID] = curr_boxID;
            }
        }
    }

    // Removing any repetitive match with curr_Frame box_id
    map<int, pair<int, int>> curr_boxID_max;
    for (auto const &bbBest_it : bbBestMatches)
    {
        int prev_boxID = bbBest_it.first;
        int curr_boxID = bbBest_it.second;
        int counter = bb_match_counter[make_pair(prev_boxID, curr_boxID)];
        map<int, pair<int, int>>::iterator it = curr_boxID_max.find(curr_boxID);
        if (it == curr_boxID_max.end())
        {
            curr_boxID_max.insert(pair<int, pair<int, int>>(curr_boxID, make_pair(prev_boxID, counter)));
        }
        else
        {
            if (counter > curr_boxID_max[curr_boxID].second)
            {
                bbBestMatches.erase(curr_boxID_max[curr_boxID].first);
                curr_boxID_max[curr_boxID] = make_pair(prev_boxID, counter);
            }
        }
    }
}

double removeOutliers_and_get_min(vector<LidarPoint> points)
{
    // Extract all x values
    vector<double> xVec;
    for (auto pt : points)
    {
        xVec.push_back(pt.x);
    }

    // Ordering only key elements to get quartiles
    auto const Q1 = xVec.size() / 4;
    auto const Q2 = xVec.size() / 2;
    auto const Q3 = Q1 + Q2;

    nth_element(xVec.begin(), xVec.begin() + Q1, xVec.end());
    nth_element(xVec.begin() + Q1 + 1, xVec.begin() + Q2, xVec.end());
    nth_element(xVec.begin() + Q2 + 1, xVec.begin() + Q3, xVec.end());
    // Removing outliers that are below a lower_bound following interquartile range from statistics. Reference: https://en.wikipedia.org/wiki/Interquartile_range
    double outlier_low_bound = xVec[Q1] - (xVec[Q3] - xVec[Q1]) * 1.5;
    if (outlier_low_bound<0){outlier_low_bound=0.0;}
    xVec.erase(std::remove_if(xVec.begin(), xVec.end(), [outlier_low_bound](const double &x){ return x < outlier_low_bound; }),xVec.end());
    // return min value
    return *min_element(xVec.begin(), xVec.end());
}
