//
// Created by plusseven on 24-7-3.
//

#include "../include/ImageStitch.h"

void ImageStitch::change_F_of_image(cv::Mat org_K, cv::Mat goal_K, cv::Mat &org_image, cv::Mat &goal_image, cv::Mat &perspective_K){
    cv::Mat inv_org_K;
    cv::invert(org_K, inv_org_K);
    perspective_K = goal_K * inv_org_K;
    std::cout <<  "perspective_K: " << perspective_K << std::endl;
    warpPerspective(org_image, goal_image, perspective_K, cv::Size(960*2, 720*2));
}


/********************直接图像拼接函数*************************/
bool ImageStitch::getKeypoints(cv::Mat &img1, cv::Mat &img2,
                            std::vector<cv::Point2f> &keypoints1, std::vector<cv::Point2f> &keypoints2)
{
    cv::Mat g1(img1, cv::Rect(0, 0, img1.cols, img1.rows));  // init roi
    cv::Mat g2(img2, cv::Rect(0, 0, img2.cols, img2.rows));

    cvtColor(g1, g1, cv::COLOR_BGR2GRAY);
    cvtColor(g2, g2, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> keypoints_roi, keypoints_img;  /* keypoints found using SIFT */
    cv::Mat descriptor_roi, descriptor_img;                           /* Descriptors for SIFT */
    cv::FlannBasedMatcher matcher;                                   /* FLANN based matcher to match keypoints */

    std::vector<cv::DMatch> matches, good_matches;
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
//    int i, dist = 80;

    sift->detectAndCompute(g1, cv::Mat(), keypoints_roi, descriptor_roi);      /* get keypoints of ROI image */
    sift->detectAndCompute(g2, cv::Mat(), keypoints_img, descriptor_img);         /* get keypoints of the image */
    matcher.match(descriptor_roi, descriptor_img, matches);  //实现描述符之间的匹配

    double max_dist = 80;
    double min_dist = 30;
    //-- Quick calculation of max and min distances between keypoints
//    for (int i = 0; i < descriptor_roi.rows; i++)
//    {
//        double dist = matches[i].distance;
//        if (dist < min_dist) min_dist = dist;
//        if (dist > max_dist) max_dist = dist;
//    }
//    std::cout << "min_dist: " << min_dist << std::endl;
//    std::cout << "max_dist: " << max_dist << std::endl;

    // 特征点筛选
    for (int i = 0; i < descriptor_roi.rows; i++) {
        if (matches[i].distance > min_dist && matches[i].distance < max_dist) {
            good_matches.push_back(matches[i]);
        }
    }

    printf("%ld no. of matched keypoints in right image\n", good_matches.size());
    /* Draw matched keypoints */

    cv::Mat img_matches;
    //绘制匹配
    drawMatches(img1, keypoints_roi, img2, keypoints_img,
                good_matches, img_matches, cv::Scalar::all(-1),
                cv::Scalar::all(-1), std::vector<char>(),
                cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::namedWindow("matches", cv::WINDOW_NORMAL);
    imshow("matches", img_matches);

    for (int i = 0; i < good_matches.size(); i++) {
        keypoints1.push_back(keypoints_img[good_matches[i].trainIdx].pt);
        keypoints2.push_back(keypoints_roi[good_matches[i].queryIdx].pt);
    }

//    this->Stitching(img1, img2, keypoints1, keypoints2);
    return true;
}
bool ImageStitch::Stitching(cv::Mat &img1, cv::Mat &img2,
                            std::vector<cv::Point2f> &keypoints1, std::vector<cv::Point2f> &keypoints2)
{
    //计算单应矩阵(仿射变换矩阵)
    cv::Mat H = findHomography(keypoints1, keypoints2, cv::RANSAC);
    cv::Mat H2 = findHomography(keypoints2, keypoints1, cv::RANSAC);
//    cv::Mat H = findHomography(keypoints1, keypoints2, 0);
//    cv::Mat H2 = findHomography(keypoints2, keypoints1, 0);


    cv::Mat stitchedImage;  //定义仿射变换后的图像(也是拼接结果图像)
    cv::Mat stitchedImage2;  //定义仿射变换后的图像(也是拼接结果图像)
    int mRows = img2.rows;
    if (img1.rows > img2.rows)
    {
        mRows = img1.rows;
    }

    int count = 0;
    for (int i = 0; i < keypoints2.size(); i++)
    {
        if (keypoints2[i].x >= img2.cols / 2)
            count++;
    }
    //判断匹配点位置来决定图片是左还是右
    if (count / float(keypoints2.size()) >= 0.5)  //待拼接img2图像在zuo边
    {
        std::cout << "img1 should be left" << std::endl;
        std::vector<cv::Point2f>corners(4);
        std::vector<cv::Point2f>corners2(4);
        std::vector<cv::Point2f>corners3(4);
        corners[0] = cv::Point(0, 0);
        corners[1] = cv::Point(0, img2.rows);
        corners[2] = cv::Point(img2.cols, img2.rows);
        corners[3] = cv::Point(img2.cols, 0);
        stitchedImage = cv::Mat::zeros(img2.cols + img1.cols, mRows, CV_8UC3);
//        warpPerspective(img2, stitchedImage, H, cv::Size(img2.cols + img1.cols, mRows));

        std::cout << "H: " << H << std::endl;
        perspectiveTransform(corners, corners2, H);
        /*
        circle(stitchedImage, corners2[0], 5, Scalar(0, 255, 0), 2, 8);
        circle(stitchedImage, corners2[1], 5, Scalar(0, 255, 255), 2, 8);
        circle(stitchedImage, corners2[2], 5, Scalar(0, 255, 0), 2, 8);
        circle(stitchedImage, corners2[3], 5, Scalar(0, 255, 0), 2, 8); */
        std::cout << corners2[0].x << ", " << corners2[0].y << std::endl;
        std::cout << corners2[1].x << ", " << corners2[1].y << std::endl;
        std::cout << corners2[2].x << ", " << corners2[2].y << std::endl;
        //imwrite("temp.jpg", stitchedImage);

        int tx = int(-std::min(corners2[0].x,std::min(corners2[3].x, corners2[1].x)));
        int ty = int(-std::min(corners2[0].y,std::min(corners2[3].y, corners2[1].y)));

        cv::Mat translation = (cv::Mat_<double>(3,3) << 1.0,0.0,tx,
                                                                  0.0,1.0,ty,
                                                                  0.0, 0.0, 1.0 );
        std::cout << "translation:" << translation << std::endl;

        perspectiveTransform(corners, corners3, translation*H);
        std::cout << corners3[0].x << ", " << corners3[0].y << std::endl;
        std::cout << corners3[1].x << ", " << corners3[1].y << std::endl;
        std::cout << corners3[2].x << ", " << corners3[2].y << std::endl;
        std::cout << corners3[3].x << ", " << corners3[3].y << std::endl;
//        warpPerspective(img2, stitchedImage, translation*H, cv::Size(img2.cols + img1.cols, mRows));
        int w_img2 = std::max(corners2[2].x,std::max(corners2[3].x, corners2[1].x));
        int h_img2 = std::max(corners2[2].y,std::max(corners2[3].y, corners2[1].y));
        int w = std::max(tx + img1.cols, w_img2);
        int h = std::max(ty + img1.rows, h_img2);

        translation = (cv::Mat_<double>(3,3) << 1.0,0.0,img1.cols,
                                                                  0.0,1.0,img1.rows,
                                                                  0.0, 0.0, 1.0 );

        warpPerspective(img2, stitchedImage, translation*H,cv::Size(w ,h));
//        warpPerspective(img2, stitchedImage, H,cv::Size(w ,h));

        cv::namedWindow("temp", cv::WINDOW_NORMAL);
        imshow("temp", stitchedImage);

//        cvtColor(stitchedImage, stitchedImage, cv::COLOR_BGR2BGRA);
//        cvtColor(img1, img1, cv::COLOR_BGR2BGRA);

        // cv::Mat half(stitchedImage, cv::Rect(tx, ty, img1.cols, img1.rows));
        // img1.copyTo(half);
        //
        // cv::namedWindow("result", cv::WINDOW_NORMAL);
        // imshow("result", stitchedImage);
        // cv::waitKey(0);
    }
    else  //待拼接图像img2在左边
    {
        std::cout << "img2 should be left" << std::endl;
        stitchedImage = cv::Mat::zeros(img2.cols + img1.cols, mRows, CV_8UC3);
        warpPerspective(img1, stitchedImage, H2, cv::Size(img1.cols + img2.cols, mRows));
        cv::namedWindow("temp", cv::WINDOW_NORMAL);
        imshow("temp", stitchedImage);

        //计算仿射变换后的四个端点
        std::vector<cv::Point2f>corners(4);
        std::vector<cv::Point2f>corners2(4);
        std::vector<cv::Point2f>corners3(4);

        corners[0] = cv::Point(0, 0);
        corners[1] = cv::Point(0, img1.rows);
        corners[2] = cv::Point(img1.cols, img1.rows);
        corners[3] = cv::Point(img1.cols, 0);

        perspectiveTransform(corners, corners2, H2);  //仿射变换对应端点
        /*
        circle(stitchedImage, corners2[0], 5, Scalar(0, 255, 0), 2, 8);
        circle(stitchedImage, corners2[1], 5, Scalar(0, 255, 255), 2, 8);
        circle(stitchedImage, corners2[2], 5, Scalar(0, 255, 0), 2, 8);
        circle(stitchedImage, corners2[3], 5, Scalar(0, 255, 0), 2, 8); */
        std::cout << corners2[0].x << ", " << corners2[0].y << std::endl;
        std::cout << corners2[1].x << ", " << corners2[1].y << std::endl;
        std::cout << corners2[2].x << ", " << corners2[2].y << std::endl;

        int tx = int(-std::min(corners2[0].x,std::min(corners2[3].x, std::min(corners2[2].x, corners2[1].x))));
        int ty = int(-std::min(corners2[0].y,std::min(corners2[3].y, std::min(corners2[2].y, corners2[1].y))));


        cv::Mat translation = (cv::Mat_<double>(3,3) << 1.0,0.0,tx,
                                                                  0.0,1.0,ty,
                                                                  0.0, 0.0, 1.0 );
        std::cout << "translation:" << translation << std::endl;
        perspectiveTransform(corners, corners3, translation*H2);
        std::cout << corners3[0].x << ", " << corners3[0].y << std::endl;
        std::cout << corners3[1].x << ", " << corners3[1].y << std::endl;
        std::cout << corners3[2].x << ", " << corners3[2].y << std::endl;
        std::cout << corners3[3].x << ", " << corners3[3].y << std::endl;
        
        int w_img2 = std::max(corners2[2].x,std::max(corners2[3].x, std::max(corners2[0].x, corners2[1].x)));
        int h_img2 = std::max(corners2[2].y,std::max(corners2[3].y, std::max(corners2[0].y, corners2[1].y)));
        int w = std::max(tx + img2.cols, w_img2);
        int h = std::max(ty + img2.rows, h_img2);
        warpPerspective(img1, stitchedImage, H2,cv::Size(w ,h));
        
        cv::Mat half(stitchedImage, cv::Rect(0, 0, img2.cols, img2.rows));
        img2.copyTo(half);
        cv::namedWindow("result", cv::WINDOW_NORMAL);
        cv::imwrite("test.png",  stitchedImage);
        cv::imshow("result", stitchedImage);

    }

//    imwrite("result.bmp", stitchedImage);
    return true;
}
