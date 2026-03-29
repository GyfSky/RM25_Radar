#ifndef SRC_IMAGESTITCH_H
#define SRC_IMAGESTITCH_H

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
# define STATIC_CONFIC_PATH "src/radar_bringup/config/Config.yaml"
# define SHARED_CONFIC_PATH "install/radar_bringup/share/radar_bringup/config/Config.yaml"


class ImageStitch {

public:
    void change_F_of_image(cv::Mat org_K, cv::Mat goal_K, cv::Mat &org_image, cv::Mat &goal_image, cv::Mat &perspective_K);
    bool getKeypoints(cv::Mat &img1, cv::Mat &img2,
                   std::vector<cv::Point2f> &keypoints1, std::vector<cv::Point2f> &keypoints2);
    bool Stitching(cv::Mat &img1, cv::Mat &img2,
                   std::vector<cv::Point2f> &keypoints1, std::vector<cv::Point2f> &keypoints2);

};


#endif
