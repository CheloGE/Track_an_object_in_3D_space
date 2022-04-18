
#ifndef camFusion_hpp
#define camFusion_hpp

#include <stdio.h>
#include <vector>
#include <opencv2/core.hpp>
#include "dataStructures.h"

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches);
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait = true);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg = nullptr);
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC);
void matchKeypointsWithBoundingBoxes(DataFrame inputFrame, std::map<int, int> &kp_bb_idxs);
void countBB_matches(DataFrame prevFrame, std::map<int, int> kp_bb_idxs_prevFrame, DataFrame currFrame, std::map<int, int> kp_bb_idxs_currFrame, std::vector<cv::DMatch> matches, std::map<std::pair<int, int>, int> &bb_match_counter);
void fill_bbBestMatches(std::map<std::pair<int, int>, int> bb_match_counter, std::map<int, int> &bbBestMatches);
double removeOutliers_and_get_min(std::vector<LidarPoint> points);
#endif /* camFusion_hpp */
