//
// Created by plusseven on 24-7-3.
//

#include "../include/ImageStitch.h"

int main(){
    cv::Mat org_K = (cv::Mat_<double>(3,3) << 1703.57857839016,0.0,720,
                                                         0.0,1698.69114037183,540,
                                                         0.0, 0.0, 1.0 );
    cv::Mat goal_K = (cv::Mat_<double>(3,3) << 2291.14722575626,0.0,960.,
                                                         0.0,2291.55696902824,720.,
                                                         0.0, 0.0, 1.0 );
    cv::Mat img, change_img, stitch_img;
    cv::Mat perspective_K;
    std::string img_path = "/home/thesky/camera/AC83-EF44/record/Born_of_Fire_1/2024-08-02_19_48_20sec/3192.jpg";
    std::string stitch_img_path = "/home/thesky/camera/AC83-EF44/record/Born_of_Fire_1/2024-08-02_19_48_20main/3192.jpg";

//    std::string img_path = "/media/plusseven/EA61-A1AF/datasets/record/I_Hiter/2024-05-23_17_08_12_n=1/5000.jpg";
//    std::string stitch_img_path = "/media/plusseven/EA61-A1AF/datasets/record/I_Hiter/2024-05-23_17_08_11_n=1/5000.jpg";

    img = cv::imread(img_path);
    stitch_img = cv::imread(stitch_img_path);

    ImageStitch imageStitch;
    imageStitch.change_F_of_image(org_K, goal_K, img, change_img, perspective_K);

//    imageStitch.Stitching(change_img,stitch_img);
    std::vector<cv::Point2f> keypoints1, keypoints2;
    imageStitch.getKeypoints(stitch_img, change_img,keypoints1, keypoints2);
    imageStitch.Stitching(stitch_img, change_img,keypoints1, keypoints2);

    cv::Mat H = findHomography(keypoints1, keypoints2, cv::RANSAC);
    cv::Mat H2 = findHomography(keypoints2, keypoints1, cv::RANSAC);


    cv::FileStorage cvMatMatrixFile = cv::FileStorage(STITCH_CONFIC_PATH, cv::FileStorage::WRITE);
    cvMatMatrixFile << "Mat" << "{"
    << "perspective_K" << perspective_K
    << "H" << H
    << "H2" << H2 << "}";
    cvMatMatrixFile.release();

    cv::namedWindow("change", cv::WINDOW_NORMAL);
    cv::namedWindow("org", cv::WINDOW_NORMAL);
    cv::namedWindow("stitch", cv::WINDOW_NORMAL);
    cv::imshow("change", change_img);
    cv::imshow("org", img);
    cv::imshow("stitch", stitch_img);
    cv::waitKey(0);

    return 0;
}