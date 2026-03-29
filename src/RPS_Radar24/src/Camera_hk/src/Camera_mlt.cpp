#include "../include/Camera_mlt.h"
//
//int main(){
//    Camera Camera_hk;
//    Camera_hk.CamMainSet();
//     while (mainCamFuture_.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout){
//         if(is_MainCamWork){
//             cv::Mat imgMain = HikCamera_sptr_->getImage();
//             cv::waitKey(1);  //if exist the case that img is not empty but can't cv::imshow, please add this sentence.
//             if(!imgMain.empty()){
//                 if (mainCamMutex_.try_lock()){
//                     cv::Mat imgMain_clone = imgMain.clone();
//                     cv::imshow("working",imgMain_clone);
//                     cv::swap(imgMain, imgMainWait);
//                     mainCamMutex_.unlock();
//                 }
//                 std::cout << "主相机帧率为："  << HikCamera_sptr_->getFPS() << std::endl; // 当帧率小于35时,即有可能相机线与电脑接触不良,从而导致图片传输延迟
//             }else{
//                 std::cout<<"Empty   "<<std::endl;
//             }
//         }else{
//             std::cout << "Cam is not work!!!" << std::endl;
//         }
//
//     }
//        Camera_hk.CamMainClose();
//}

Camera::Camera(char *g_strSerialNumber, std::string Name,rclcpp::Node* node, std::string config_path, TF Image_isSave) {

    //加载默认主相机配置参数
    YAML::Node mainCamConfg = YAML::LoadFile(config_path);
    CamGain_ = mainCamConfg[Name]["gain"].as<float>();
    CamExposureTime_ = mainCamConfg[Name]["exposureTime"].as<int>();
    CamGamma_ = mainCamConfg[Name]["gamma"].as<float>();
    CamFps_ = mainCamConfg[Name]["fps"].as<float>();
    CamNum_ = mainCamConfg[Name]["camNum"].as<int>();
    std::string save_root_dir = mainCamConfg[Name]["save_root_dir"].as<std::string>();  // TODO
    bool is_save;
    if(Image_isSave == false_){
        is_save = false;
    }else{
        is_save = true;
    }
    mainCamFuture_ = mainCamExit_.get_future();
    std::shared_ptr<Camera_hk::HikCamera> HikCamera_sptr(new Camera_hk::HikCamera(Name,save_root_dir,node,CamNum_,is_save));
    HikCamera_sptr_ = HikCamera_sptr;

    //初始化主相机
    if(Name == "Hik30"){
        HikCamera_sptr_->open_thread(g_strSerialNumber);
        HikCamera_sptr_->setGamma(CamGamma_);
    } else{
        HikCamera_sptr_->open(g_strSerialNumber);
    }
    // HikCamera_sptr_->setPixelFormat2BayerRG8_12();
    HikCamera_sptr_->setGain(CamGain_);
    HikCamera_sptr_->setExposureTime(CamExposureTime_);
    HikCamera_sptr_->startGrabImage();
    //    camera.setWhiteBalance();
}


/**
 * @brief 动态设置相机相关参数
 */
void Camera::CamMainSet(){
    //  调节窗口  
    cv::namedWindow("trackbars",(360,240));
    // cv::createTrackbar("Gain","trackbars",&CamGain_,24);
    cv::createTrackbar("ExposureTime","trackbars",&CamExposureTime_,12000);
    while (is_MainCamSet){
        cv::Mat imgMain = HikCamera_sptr_->getImage();
        // cv::waitKey(1);  //if exist the case that img is not empty but can't cv::imshow, please add this sentence.
        if(cv::waitKey(1) == 'n'){
            is_MainCamWork = true;
            break;
        }
        if(!imgMain.empty()){
            if (mainCamMutex_.try_lock()){
                HikCamera_sptr_->setGain(CamGain_);
                HikCamera_sptr_->setExposureTime(CamExposureTime_);
                cv::Mat imgMain_clone = imgMain.clone();
                cv::imshow("set",imgMain_clone);
                mainCamMutex_.unlock();
            }
            std::cout << "主相机帧率为："  << HikCamera_sptr_->getFPS() << std::endl; // 当帧率小于35时,即有可能相机线与电脑接触不良,从而导致图片传输延迟
        }else{
            std::cout<<"Empty   "<<std::endl;
        }
    }
    if(!is_MainCamSet){
        is_MainCamWork = true;
        std::cout << "----MainCam will Work----" << std::endl;
    }
    cv::destroyAllWindows();
}

/**
 * @brief 关闭相机
 */
void Camera::CamMainClose(){
    is_MainCamSet = false;
    is_MainCamWork = false;
    HikCamera_sptr_->close();
}


/**
 * @brief 可能开启录制mode
 */
void Camera::CamMainSave(){
    HikCamera_sptr_->setSaveMode();
}


// /**
//  * @brief 主相机工作
//  */
// void Camera::CamMainWork(){   
//     while (mainCamFuture_.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout){
//         if(is_MainCamWork){
//             cv::Mat imgMain = HikCamera_sptr_->getImage();
//             cv::waitKey(1);  //if exist the case that img is not empty but can't cv::imshow, please add this sentence.
//             if(!imgMain.empty()){
//                 if (mainCamMutex_.try_lock()){
//                     cv::Mat imgMain_clone = imgMain.clone();
//                     cv::imshow("working",imgMain_clone);
//                     cv::swap(imgMain, imgMainWait);
//                     mainCamMutex_.unlock();
//                 }
//                 std::cout << "主相机帧率为："  << HikCamera_sptr_->getFPS() << std::endl; // 当帧率小于35时,即有可能相机线与电脑接触不良,从而导致图片传输延迟
//             }else{
//                 std::cout<<"Empty   "<<std::endl;
//             }
//         }else{
//             std::cout << "Cam is not work!!!" << std::endl;
//         }
        
//     }
    
// }



//TODO:动态调参后,参数的保存