//
// Created by plusseven on 24-7-5.
//
#include "../include/Radar.h"

Modes::Modes(){
    //choose  一些选项
    application   = Application::Radar;
    pictureSource = PictureSource::picture_dir ;    //图片来源          |
    isOpenMid70   = TF::false_;
    isUseMid70    = TF::false_;
    detectionMode = Detection::netDetection;
    ourPattern    = OurPattern::blue;                  //己方颜色       |
    Port_isOpen   = TF::false_  ;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    isSave        = TF::false_;                       //是否保存图片    |

};




int main(){

    cv::Mat org_K = (cv::Mat_<double>(3,3) << 1703.57857839016,0.0,720,
            0.0,1698.69114037183,540,
            0.0, 0.0, 1.0 );
    cv::Mat goal_K = (cv::Mat_<double>(3,3) << 2291.14722575626,0.0,960.,
            0.0,2291.55696902824,720.,
            0.0, 0.0, 1.0 );
    cv::Mat img, change_img, stitch_img;
    std::string img_path = "/media/plusseven/EA61-A1AF/datasets/record/I_Hiter/2024-05-24_16_42_28sec/1000.jpg";
    std::string stitch_img_path = "/media/plusseven/EA61-A1AF/datasets/record/I_Hiter/2024-05-24_16_42_28main/1000.jpg";

    img = cv::imread(img_path);
    stitch_img = cv::imread(stitch_img_path);

    cv::Mat perspective_K;
    ImageStitch imageStitch;
    imageStitch.change_F_of_image(org_K, goal_K, img, change_img, perspective_K);

    std::vector<cv::Point2f> keypoints1, keypoints2;
    imageStitch.getKeypoints(stitch_img, change_img,keypoints1, keypoints2);
    imageStitch.Stitching(stitch_img, change_img,keypoints1, keypoints2);
    cv::namedWindow("Hik60", cv::WINDOW_NORMAL);
    cv::namedWindow("org", cv::WINDOW_NORMAL);
    cv::namedWindow("stitch", cv::WINDOW_NORMAL);
    cv::imshow("org", img);

    std::vector<cv::Point2d> pts_pnp_2d_main, pts_pnp_2d_sec;

    auto Modes_ptr = std::shared_ptr<Modes>(new Modes());
    auto MainCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik60",CamPosition::right,Modes_ptr->ourPattern));
    auto SecCam_ptr = std::shared_ptr<SensorParam>(new SensorParam("Hik60",CamPosition::left,Modes_ptr->ourPattern));
    auto CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem(12));

    cv::Mat stitch_img_clone = stitch_img.clone();
    cv::Mat change_img_clone = change_img.clone();
    cv::imshow("Hik60", stitch_img);
    MainCam_ptr->pts_pnp_2d = GetPoint2d_mouse(stitch_img,"Hik60");
    cv::imshow("Hik60", change_img);
    SecCam_ptr->pts_pnp_2d = GetPoint2d_mouse(change_img,"Hik60");
    CooSystem_ptr->Get2world_matrix(MainCam_ptr->T_2world  ,MainCam_ptr->K ,MainCam_ptr->pts_pnp_2d , MainCam_ptr->pts_pnp_3d);
    CooSystem_ptr->Get2world_matrix(SecCam_ptr->T_2world  ,SecCam_ptr->K ,SecCam_ptr->pts_pnp_2d , SecCam_ptr->pts_pnp_3d);

    std::vector<cv::Point2f> keypoints1_pnp(4), keypoints2_pnp(4);

    std::vector<cv::Point3f> stitch_3d;
    stitch_3d.push_back(cv::Point3f(3.82370, 4.00119, 0.0));
    stitch_3d.push_back(cv::Point3f(8.27792, 4.87184, 0.00));
    stitch_3d.push_back(cv::Point3f(12.39134,5.53041, 0.0));
    stitch_3d.push_back(cv::Point3f(15.02442,3.40500, 0.09));
//    stitch_3d.push_back(cv::Point3f(15.02442,5, 0.5));
//    stitch_3d.push_back(cv::Point3f(10.02442,3.0, 0.1));
//    stitch_3d.push_back(cv::Point3f(12.02442,4.4, 0.7));
//    stitch_3d.push_back(cv::Point3f(13.02442,5.3, 0.6));
//    stitch_3d.push_back(cv::Point3f(7.02442,3.9, 1.6));
//    stitch_3d.push_back(cv::Point3f(9.0,3.4, 2.4));
//    stitch_3d.push_back(cv::Point3f(5.5,5.5, 0.3));
//    stitch_3d.push_back(cv::Point3f(5.5,3.5, 0.09));

    for(int i = 0; i < 4; i++){
        CooSystem_ptr->get_Point_2d(((MainCam_ptr->T_2world).inv()),MainCam_ptr->K, stitch_3d[i], keypoints1_pnp[i]);
        CooSystem_ptr->get_Point_2d(((SecCam_ptr->T_2world).inv()),SecCam_ptr->K, stitch_3d[i], keypoints2_pnp[i]);
    }

    imageStitch.Stitching(stitch_img_clone, change_img_clone,keypoints1_pnp, keypoints2_pnp);

    cv::waitKey(0);
    return 0;
}