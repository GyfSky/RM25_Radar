//
// Created by plusseven on 23-10-22.
//

#include "../include/Image.h"

// int main(){
//     Modes modes;
// //    Image image(modes.application, modes.pictureSource,"Hik30", modes.Image_isSave, modes.saveImagePath);
//     Image image(Common, camera_,"Hik30",  Hikang, false_, modes.saveImagePath);
//     image.Init();
//     int after_picture=1;
//     while (1){
//         if(cv::waitKey(1) == 'q'){
//             break;
//         }
//         image.Image_Get(after_picture);
//         image.Image_Show();
//     }
    
// }


Image::Image(Application application,PictureSource pictureSource,std::string Name ,TF Image_isSave,SaveImagePath saveImagePath,int serial_number ){
    //mode
    this->pictureSource = pictureSource;
    this->application = application;
    this->serial_number = serial_number;
//    SetNet(cls_to_string);
    this->net_config = YAML::LoadFile(YAML_NETCONFIC_PATH);
    this->classWithoutCar = net_config["classWithoutCar"].as<int>();
    std::cout << "classWithoutCar: " << classWithoutCar << std::endl;
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
{//pictureSource path
    if(this->pictureSource == single_picture){
        //image_path = "/home/plusseven/桌面/radar2023_withtrt/radar_val_picture/6.jpg";
        image_path = config[Name]["source_path"]["singleImg_path"].as<std::string>();
    }
    else if(this->pictureSource == picture_dir){
        //image_dir =  "/home/plusseven/桌面/radar2023/radar_val_picture/";
        image_dir =  config[Name]["source_path"]["picture_dir"].as<std::string>();
        start_picture = config[Name]["source_path"]["picture_dir_start"].as<int>();
    }
    else if(this->pictureSource == video){
        //video_path = "/media/plusseven/4E21-0000/2023radarData/important/adapt1.mp4";
        video_path =  config[Name]["source_path"]["video_path"].as<std::string>();
    }else if(this->pictureSource == ros){
        // ROS_INFO("ros_img is on the way" );
        // RCLCPP_INFO(node->get_logger(), "ros_img is on the way");
    }
    else   Cam_isOpen = true;
}
{//save image or not and save image_path 
    if(Image_isSave == TF::true_){
        Image_issave = true;
    }
    else if(Image_isSave == TF::false_){
        Image_issave = false;
    }
    if(saveImagePath == SaveImagePath::disk02){
        Image_savepath = config["ImageSavePath"]["disk02"].as<std::string>();
    }
    else if(saveImagePath == SaveImagePath::ssdgaoyuan){
        Image_savepath = config["ImageSavePath"]["ssdgaoyuan"].as<std::string>();
    }
}
    Cam_winname = config[Name]["winname"].as<std::string>();
    input_w = config[Name]["picture_size"]["input_w"].as<int>();
    input_h = config[Name]["picture_size"]["input_h"].as<int>();
    // if(useCamera == Hikang30 || useCamera == Hikang31)  useCamera = Hikang;

    if(application==Radar){
        mapImage_winname = config["map"]["mapImage_winname"].as<std::string>();
        mapImage_path = config["map"]["mapPath"].as<std::string>();
        map_w = config["map"]["map_w"].as<int>();
        map_h = config["map"]["map_h"].as<int>();
    }
}

//带有序列号 g_strSerialNumber
Image::Image(Application application,PictureSource pictureSource,char g_strSerialNumber[64],rclcpp::Node* node,std::string Name,TF Image_isSave,SaveImagePath saveImagePath,int serial_number){
    ////mode
    this->pictureSource = pictureSource;
    this->application = application;
    this->serial_number = serial_number;//start
//    SetNet(cls_to_string);
    this->net_config = YAML::LoadFile(YAML_NETCONFIC_PATH);
    this->classWithoutCar = net_config["classWithoutCar"].as<int>();
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
{//pictureSource path
    if(this->pictureSource == single_picture){
        //image_path = "/home/plusseven/桌面/radar2023_withtrt/radar_val_picture/6.jpg";
        image_path = config[Name]["source_path"]["singleImg_path"].as<std::string>();
    }
    else if(this->pictureSource == picture_dir){
        //image_dir =  "/home/plusseven/桌面/radar2023/radar_val_picture/";
        image_dir =  config[Name]["source_path"]["picture_dir"].as<std::string>();
        start_picture = config[Name]["source_path"]["picture_dir_start"].as<int>();
    }
    else if(this->pictureSource == video){
        //video_path = "/media/plusseven/4E21-0000/2023radarData/important/adapt1.mp4";
        video_path =  config[Name]["source_path"]["video_path"].as<std::string>();
    }
    else if(this->pictureSource == ros){
        std::cout << "ros_img is on the way" << std::endl;
    }
    else if(this->pictureSource == camera_){
        Cam_isOpen = true;
        this->Camerahk_prt = std::shared_ptr<Camera>(new Camera(g_strSerialNumber, Name, node,Image_isSave));
    }
}
{//save image or not and save image_path
    if(Image_isSave == TF::true_){
        Image_issave = true;
    }
    else if(Image_isSave == TF::false_){
        Image_issave = false;
    }
    if(saveImagePath == SaveImagePath::disk02){
        Image_savepath = config["ImageSavePath"]["disk02"].as<std::string>();
    }
    else if(saveImagePath == SaveImagePath::ssdgaoyuan){
        Image_savepath = config["ImageSavePath"]["ssdgaoyuan"].as<std::string>();
    }
}
    Cam_winname = config[Name]["winname"].as<std::string>();
    input_w = config[Name]["picture_size"]["input_w"].as<int>();//
    input_h = config[Name]["picture_size"]["input_h"].as<int>();//
    // if(useCamera == Hikang30 || useCamera == Hikang31)  useCamera = Hikang;

    if(application==Radar){
        mapImage_winname = config["map"]["mapImage_winname"].as<std::string>();
        mapImage_path = config["map"]["mapPath"].as<std::string>();
        map_w = config["map"]["map_w"].as<int>();
        map_h = config["map"]["map_h"].as<int>();
    }
}

void Image::getImg(const sensor_msgs::msg::CompressedImage::ConstPtr &rosImg_ptr){
    cv::Mat img = cv::imdecode(rosImg_ptr->data,1);
    if(!img.empty()){
        cv::cvtColor(img,this->Cam_img,CV_8UC3);
        this->Cam_img = img.clone();
    }else{
        std::cout << "error!!!!!" << std::endl;
    }
    return;
}

void Image::Init(int argc,char *argv[]){
    //视频的初始化ing
    if(pictureSource==video ){
        cap = cv::VideoCapture(video_path);
    }
    //相机的初始化ing
    else if(pictureSource==camera_){
//        Camerahk_prt->CamMainSet();
        std::cout << "go on";
    }
    else if(pictureSource==ros){
        // rclcpp::init(argc, argv);
        // auto nh = rclcpp::Node::make_shared("img_listener");
        // sub_img = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/compressed_image", 1, std::bind(&Image::getImg, this, std::placeholders::_1));
        // while (rclcpp::ok())
        // {
        //     if(Cam_img.empty()){
        //         // ros::spinOnce();
        //         rclcpp::spin_some(nh);
        //     }else{
        //         break;
        //     }
        // }
        // rclcpp::spin(nh);
        // rclcpp::shutdown();
    }


    //窗口配置
    cv::namedWindow(this->Cam_winname,0);
    cv::resizeWindow(this->Cam_winname,cv::Size(input_w,input_h));
    //小地图相关配置
    if(application == Radar){
        cv::namedWindow(this->mapImage_winname,0);
        cv::resizeWindow(this->mapImage_winname,cv::Size(map_w/3, map_h/3));  ////TODO:
        map_img = cv::imread(this->mapImage_path);
        map_cloneing = map_img.clone();
    }
}

void Image::GetGammaCorrection(cv::Mat &src, cv::Mat &dst, const float fGamma) {
    unsigned char bin[256];
    for (int i = 0; i < 256; ++i) {
        bin[i] = saturate_cast<uchar>(pow((float) (i / 255.0), fGamma) * 255.0f);
    }
    dst = src.clone();
    const int channels = dst.channels();
    switch (channels) {
        case 1: {
            MatIterator_<uchar> it, end;
            for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
                *it = bin[(*it)];
            break;
        }
        case 3: {
            MatIterator_<Vec3b> it, end;
            for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++) {
                (*it)[0] = bin[((*it)[0])];
                (*it)[1] = bin[((*it)[1])];
                (*it)[2] = bin[((*it)[2])];
            }
            break;
        }
    }
}

cv::Mat Image::Image_Get(int &after_picture,int argc, char **argv){
    //get new picture
    if(this->pictureSource==picture_dir)
        if(this->serial_number == -1){
            dynamic_image_path = this->image_dir + std::to_string(this->start_picture + after_picture) +".jpg";
        }else{
            dynamic_image_path = this->image_dir + std::to_string(this->serial_number + after_picture) +".jpg";
        }
    // get picture
    switch (this->pictureSource) {
        case video:
            (this->cap).read(this->Cam_img);
            //change image
        // image.image = image.image(cv::Range(476,1500),cv::Range(120,1400));
            break;
        case camera_:
            // if (this->useCamera == Cameras::Dahang) std::cout << "Maybe Have Error!!!";//  image.radar_img = cameraReader.getframe();
            // else if (this->useCamera == Cameras::Hikang) this->Cam_img = this->Camerahk_prt->imgMainWait;           /////////
            // else if (this->useCamera == Cameras::Hikang){
            this->Cam_img = this->Camerahk_prt->HikCamera_sptr_->getImage();
            if (this->Cam_winname=="Hik60") {
                this->ros_time=this->Camerahk_prt->HikCamera_sptr_->getTime();
            }

            // }         /////////
            // else std::cout << "Maybe Have Error!!!";
            break;
        case single_picture:
            this->Cam_img = cv::imread(this->image_path);
            break;
        case picture_dir:
            this->Cam_img = cv::imread(this->dynamic_image_path);
            break;
        case ros:
            // auto nh = rclcpp::Node::make_shared("img_listener1");
            // sub_img = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/compressed_image", 1, std::bind(&Image::getImg, this, std::placeholders::_1));
            // bool ros_img_is_ok = false;
            // rclcpp::spin_some(nh);
            break;
        default:
            std::cerr << "no picture" << std::endl;
            break;
    }
    // this->Cam_cloneing = this->Cam_img.clone();//clone为深拷贝
    this->Cam_draw     = this->Cam_img.clone();
    if(application==Application::Radar){
        // this->map_cloneing = this->map_img.clone();
        this->map_draw     = this->map_img.clone();
    }
    if(cv::waitKey(1) == 'x'){
        after_picture++;
    }
    else if(cv::waitKey(1) == 'z'){
        after_picture--;
    }
    return this->Cam_img;
}


void Image::Image_Show(){
    cv::imshow(this->Cam_winname, this->Cam_draw);
    if(application==Application::Radar){
        cv::imshow(this->mapImage_winname, this->map_draw);  
    }
    cv::waitKey(1);
}

void Image::setSaveMode() {
    Camerahk_prt->CamMainSave();
}

void Image::Close(){
    if(pictureSource==camera_){
        Camerahk_prt->CamMainClose();
    }
    cv::destroyAllWindows();
}

//void SetNet(std::map<int,std::string> &cls_to_string){
//    // int cls to string
//    {
//        cls_to_string[0] = "RG";
//        cls_to_string[1] = "R1";
//        cls_to_string[2] = "R2";
//        cls_to_string[3] = "R3";
//        cls_to_string[4] = "R4";
//        cls_to_string[5] = "R5";
//        cls_to_string[6] = "BG";
//        cls_to_string[7] = "B1";
//        cls_to_string[8] = "B2";
//        cls_to_string[9] = "B3";
//        cls_to_string[10] = "B4";
//        cls_to_string[11] = "B5";
//        cls_to_string[12] = "car";
//    }
//}



void Image::draw_rusult(std::vector<TRTInferV1::Object> objs, bool isShow/*=true*/){
    // std::cout << "draw1" << std::endl;
    for(TRTInferV1::Object obj:objs){
        cv::rectangle(Cam_draw,cv::Rect(obj.x1,obj.y1,obj.w,obj.h),cv::Scalar(75, 150, 225),2);
        cv::putText(Cam_draw,net_config[obj.classId].as<std::string>(),cv::Point(obj.x2+10,obj.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {225, 150, 75},2);
        cv::putText(Cam_draw,std::to_string(obj.confidence),cv::Point(obj.x2+10,obj.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
    }
    if(isShow){
        cv::imshow(Cam_winname,Cam_draw);
    }
}


void Image::draw_rusult(std::vector<Car> cars,std::vector<Armor> armors,bool isShow/*=true*/){
    // std::cout << "draw1" << std::endl;
    for(Armor armor:armors){
        cv::rectangle(Cam_draw,armor.rect,cv::Scalar(75, 150, 225),2);
        cv::circle(Cam_draw,cv::Point((armor.rect.x+armor.rect.width/2),(armor.rect.y+armor.rect.height/2)),1,cv::Scalar(25,125,225),2);
        cv::putText(Cam_draw,net_config[armor.cls].as<std::string>(),cv::Point(armor.x2+10,armor.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {225, 150, 75},2);
        cv::putText(Cam_draw,std::to_string(armor.conf),cv::Point(armor.x2+10,armor.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
    }
    // std::cout << "draw2" << std::endl;
    for(Car car:cars){
        if(car.cls < (this->classWithoutCar/2)){
            cv::rectangle(Cam_draw,car.rect,cv::Scalar(150, 150, 250),2);
            int cls = car.cls;
            if(car.cls<0){
                cls = classWithoutCar;
            }
            cv::putText(Cam_draw,net_config[cls].as<std::string>(),cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {150, 150, 250},2);
            cv::putText(Cam_draw,std::to_string(car.conf),cv::Point(car.x2+10,car.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
            if(application==Radar){
                circle(map_draw,cv::Point(car.Locate3D.x/28*map_w,map_h-car.Locate3D.y/15*map_h),0,cv::Scalar(150, 150, 250),15 );
                putText(map_draw,net_config[cls].as<std::string>(),cv::Point(car.Locate3D.x/28*map_w,map_h-car.Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 150, 250), 2);
            }
        }
        else if(car.cls < this->classWithoutCar){
            cv::rectangle(Cam_draw,car.rect,cv::Scalar(255, 150, 150),2);
            cv::putText(Cam_draw,net_config[car.cls].as<std::string>(),cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {255, 150, 150},2);
            cv::putText(Cam_draw,std::to_string(car.conf),cv::Point(car.x2+10,car.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
            if(application==Radar){
                circle(map_draw,cv::Point(car.Locate3D.x/28*map_w,map_h-car.Locate3D.y/15*map_h),0,cv::Scalar(255, 150, 150),15 );
                putText(map_draw,net_config[car.cls].as<std::string>(),cv::Point(car.Locate3D.x/28*map_w,map_h-car.Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 150, 150), 2);
            }
        }else{
            cv::rectangle(Cam_draw,car.rect,cv::Scalar(150, 150, 150),2);
            cv::putText(Cam_draw,"unknown",cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {150, 150, 150},2);
            cv::putText(Cam_draw,std::to_string(car.conf),cv::Point(car.x2+10,car.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
            if(application==Radar){
                circle(map_draw,cv::Point(car.Locate3D.x/28*map_w,map_h-car.Locate3D.y/15*map_h),0,cv::Scalar(150, 150, 150),15 );
                putText(map_draw,"unknown",cv::Point(car.Locate3D.x/28*map_w,map_h-car.Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 150, 150), 2);
            }
        }
    }
    if(isShow){
        if(application==Radar){
            cv::imshow(mapImage_winname,map_draw);
        }
            cv::imshow(Cam_winname,Cam_draw);
    }
}

void Image::draw_rusult(std::vector<Car> &cars,std::vector<Armor> &armors,cv::Mat img_draw,std::string winname /*= "draw"*/){
    for(Armor armor:armors){
        cv::rectangle(img_draw,armor.rect,cv::Scalar(75, 150, 225),2);
        cv::circle(img_draw,cv::Point((armor.rect.x+armor.rect.width/2),(armor.rect.y+armor.rect.height/2)),1,cv::Scalar(25,125,225),2);
        cv::putText(img_draw,net_config[armor.cls].as<std::string>(),cv::Point(armor.x2+10,armor.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {225, 150, 75},2);
        cv::putText(img_draw,std::to_string(armor.conf),cv::Point(armor.x2+10,armor.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
    }
    for(Car car:cars){
        if(car.cls < (this->classWithoutCar/2) && -1 < car.cls){
            cv::rectangle(img_draw,car.rect,cv::Scalar(150, 150, 250),2);
            cv::putText(img_draw,net_config[car.cls].as<std::string>(),cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {150, 150, 250},2);
            cv::putText(img_draw,std::to_string(car.conf),cv::Point(car.x2+10,car.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
        }
        else if(car.cls < this->classWithoutCar && -1 < car.cls ){
            cv::rectangle(img_draw,car.rect,cv::Scalar(255, 150, 150),2);
            cv::putText(img_draw,net_config[car.cls].as<std::string>(),cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {255, 150, 150},2);
            cv::putText(img_draw,std::to_string(car.conf),cv::Point(car.x2+10,car.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
        }else{
            cv::rectangle(img_draw,car.rect,cv::Scalar(150, 150, 150),2);
            cv::putText(img_draw,"unknown",cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {150, 150, 150},2);
            cv::putText(img_draw,std::to_string(car.conf),cv::Point(car.x2+10,car.y1-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
        }
    }
    cv::imshow(winname,img_draw);
}

void Image::draw_test(vector<TRTInferV1::DetectionObj> car,vector<vector<TRTInferV1::DetectionObj>> Armors){
    for (int i=0;i<Armors.size(); i++) {
        for (auto armor:Armors[i]) {
            putText(Cam_draw, format("%d", armor.classId), Point(armor.x1+car[i].x1, armor.y1+car[i].y1 - 5),0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
        }
    }
    cv::imshow(Cam_winname,Cam_draw);
}

void Image::draw_cls(std::vector<STrack> output_stracks,int cam_id) {
    for (int i = 0; i < output_stracks.size(); i++)
    {
        if (cam_id==1) {
            if (output_stracks[i].camid!=cam_id) continue;
            int cls = output_stracks[i].cls ;
            if(cls != -1){
                std::vector<float> tlwh = output_stracks[i].tlwh;
                cv::Scalar s = get_color(cls);
                putText(Cam_draw, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5),
                    0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

                if( -1 < cls && cls < classWithoutCar){
                    putText(Cam_draw,net_config[cls].as<std::string>(),Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                    cv::putText(Cam_draw,std::to_string(output_stracks[i].conf_armor),cv::Point(output_stracks[i].tlbr[2]+10,output_stracks[i].tlbr[1]-30),cv::FONT_HERSHEY_SIMPLEX, 2, {100, 225, 100},2);
                } else{
                    putText(Cam_draw,"car",Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                }

                rectangle(Cam_draw, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
            }else if(application==Radar){
                cv::Scalar s = get_color(cls);
                rectangle(Cam_draw, Rect(output_stracks[i].tlwh[0], output_stracks[i].tlwh[1], output_stracks[i].tlwh[2], output_stracks[i].tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
                putText(Cam_draw,"car",Point(output_stracks[i].tlwh[0]-5, output_stracks[i].tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
            }

            cv::imshow(Cam_winname,Cam_draw);
        }else {
            if (output_stracks[i].camid!=cam_id) continue;
            int cls = output_stracks[i].cls ;
            if(cls != -1){
                std::vector<float> tlwh = output_stracks[i].tlwh;
                tlwh[0]=output_stracks[i].sec_rect[0];
                tlwh[1]=output_stracks[i].sec_rect[1];
                tlwh[2]=output_stracks[i].sec_rect[2]-output_stracks[i].sec_rect[0];
                tlwh[3]=output_stracks[i].sec_rect[3]-output_stracks[i].sec_rect[1];
                cv::Scalar s = get_color(cls);
                putText(Cam_draw, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5),
                    0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

                if( -1 < cls && cls < classWithoutCar){
                    putText(Cam_draw,net_config[cls].as<std::string>(),Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                    cv::putText(Cam_draw,std::to_string(output_stracks[i].conf_armor),cv::Point(output_stracks[i].tlbr[2]+10,output_stracks[i].tlbr[1]-30),cv::FONT_HERSHEY_SIMPLEX, 2, {100, 225, 100},2);
                } else{
                    putText(Cam_draw,"car",Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                }

                rectangle(Cam_draw, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
            }else if(application==Radar){
                cv::Scalar s = get_color(cls);
                rectangle(Cam_draw, Rect(output_stracks[i].tlwh[0], output_stracks[i].tlwh[1], output_stracks[i].tlwh[2], output_stracks[i].tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
                putText(Cam_draw,"car",Point(output_stracks[i].tlwh[0]-5, output_stracks[i].tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
            }

            cv::imshow(Cam_winname,Cam_draw);
        }

    }
}

void Image::draw_lidar(interfaces::msg::DetectResult lidar) {
    for (int i=0;i<5;i++) {
        circle(map_draw,cv::Point(lidar.red_x[i]/28*map_w,map_h-lidar.red_y[i]/15*map_h),0,cv::Scalar(255,255,255),15 );
        circle(map_draw,cv::Point(lidar.blue_x[i]/28*map_w,map_h-lidar.blue_y[i]/15*map_h),0,cv::Scalar(255,255,255),15 );
    }
}

void Image::draw_rusult(std::vector<STrack> output_stracks, bool is_cls){

    if(is_cls){
        for (int i = 0; i < output_stracks.size(); i++)
        {
            int cls = output_stracks[i].cls ;
            if(cls != -1){
                std::vector<float> tlwh = output_stracks[i].tlwh;
                bool vertical = tlwh[2] / tlwh[3] > 1.6;
                // if (tlwh[2] * tlwh[3] > 20)
                // {
                    cv::Scalar s = get_color(cls);
                    putText(Cam_draw, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5),
                            0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

                    if( -1 < cls && cls < classWithoutCar){
                        putText(Cam_draw,net_config[cls].as<std::string>(),Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                        cv::putText(Cam_draw,std::to_string(output_stracks[i].conf_armor),cv::Point(output_stracks[i].tlbr[2]+10,output_stracks[i].tlbr[1]-30),cv::FONT_HERSHEY_SIMPLEX, 2, {100, 225, 100},2);
                    } else{
                        putText(Cam_draw,"car",Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                    }

                    rectangle(Cam_draw, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                    cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
                    if(application==Radar){
                        circle(map_draw,cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),0,s,15 );
//                int track_id = output_stracks[i].track_id;
//                int cls = track_id%7 + track_id/7*classWithoutCar/2;
                        if( -1 < cls && cls < classWithoutCar){
                            putText(map_draw,net_config[cls].as<std::string>(),cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                        } else{
                            putText(map_draw,"car",cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                        }
                    }
                // }

            }else if(application==Radar){
                cv::Scalar s = get_color(cls);
                circle(map_draw,cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),0,s,15 );
//                int track_id = output_stracks[i].track_id;
//                int cls = track_id%7 + track_id/7*classWithoutCar/2;
                rectangle(Cam_draw, Rect(output_stracks[i].tlwh[0], output_stracks[i].tlwh[1], output_stracks[i].tlwh[2], output_stracks[i].tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
                putText(Cam_draw,"car",Point(output_stracks[i].tlwh[0]-5, output_stracks[i].tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                putText(map_draw,"car",cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
            }

            cv::imshow(Cam_winname,Cam_draw);
            if(application==Radar){
                cv::imshow(mapImage_winname,map_draw);
            }
        }
    }else{
        for (int i = 0; i < output_stracks.size(); i++)
        {
            std::vector<float> tlwh = output_stracks[i].tlwh;
            bool vertical = tlwh[2] / tlwh[3] > 1.6;
            if (tlwh[2] * tlwh[3] > 20 && !vertical)
            {
                cv::Scalar s = get_color(output_stracks[i].track_id);
                putText(Cam_draw, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5),
                        0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

                int track_id = output_stracks[i].track_id;
                int cls = track_id%classWithoutCar;
                if( -1 < track_id && track_id < classWithoutCar){
                    putText(Cam_draw,net_config[cls].as<std::string>(),Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                    cv::putText(Cam_draw,std::to_string(output_stracks[i].conf_armor),cv::Point(output_stracks[i].tlbr[2]+10,output_stracks[i].tlbr[1]-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
                } else{
                    putText(Cam_draw,"car",Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                }

                rectangle(Cam_draw, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
                if(application==Radar){
                    circle(map_draw,cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),0,s,15 );
//                int track_id = output_stracks[i].track_id;
//                int cls = track_id%7 + track_id/7*classWithoutCar/2;
                    if( -1 < cls && cls < classWithoutCar){
                        putText(map_draw,net_config[cls].as<std::string>(),cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                    } else{
                        putText(map_draw,"car",cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                    }
                }
            }
            cv::imshow(Cam_winname,Cam_draw);
            if(application==Radar){
                cv::imshow(mapImage_winname,map_draw);
            }
        }
    }

}





void Image::draw_rusult(cv::Rect rect, cv::Point3d xyz, cv::Mat img_draw){
//    for (int i = 0; i < output_stracks.size(); i++)
//    {
        cv::rectangle(img_draw,rect,cv::Scalar(255, 150, 150),2);
        if(application==Radar){
            circle(map_draw,cv::Point(xyz.x/28*map_w,map_h-xyz.y/15*map_h),0,cv::Scalar(255, 150, 150),15 );
        }

        cv::imshow(Cam_winname,Cam_draw);
        if(application==Radar){
            cv::imshow(mapImage_winname,map_draw);
        }
//    }
}


void Image::draw_line(std::vector<MapVertex> &vexs){
    for (int i = 0; i < vexs.size(); i++) {
        for (int j = 0; j < vexs[i].point_3d_number; j++) {
            if(j < vexs[i].point_3d_number-1 ){
                cv::line(Cam_draw,vexs[i].points_predict_2d[j],vexs[i].points_predict_2d[j+1],cv::Scalar(250,255,250));
            }
            //第一个点和最后一个点连线
            else if(j == vexs[i].point_3d_number-1 ){
                cv::line(Cam_draw,vexs[i].points_predict_2d[j],vexs[i].points_predict_2d[0],cv::Scalar(250,255,250));
            }else{
                std::cout << "def draw_line may have error " << std::endl;
            }
        }
    }
    cv::imshow(Cam_winname,Cam_draw);
}



cv::Scalar Image::get_color(int idx){
    idx += 3;
    return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}




//void draw_rusult(std::vector<Car> &cars,std::vector<Armor> &armors,Image &image,std::map<int,std::string> cls_to_string){
//    for(Armor armor:armors){
//        cv::rectangle(image.radar_draw,armor.rect,cv::Scalar(75, 150, 225),1);
//        cv::putText(image.radar_draw,cls_to_string[armor.cls],cv::Point(armor.x2+10,armor.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {225, 150, 75});
//    }
//    for(Car car:cars){
//        if(car.cls < 6){
//            cv::rectangle(image.radar_draw,car.rect,cv::Scalar(150, 150, 250),1);
//            cv::putText(image.radar_draw,cls_to_string[car.cls],cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {150, 150, 250});
//            circle(image.map_draw,cv::Point(car.Locate3D.x/28*1330,713-car.Locate3D.y/15*713),0,cv::Scalar(150, 150, 250),15 );
//            putText(image.map_draw,cls_to_string[car.cls],cv::Point(car.Locate3D.x/28*1330,713-car.Locate3D.y/15*713),cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 150, 250), 1);
//        }
//        else if(car.cls < 12){
//            cv::rectangle(image.radar_draw,car.rect,cv::Scalar(255, 150, 150),1);
//            cv::putText(image.radar_draw,cls_to_string[car.cls],cv::Point(car.x2+10,car.y1-10),cv::FONT_HERSHEY_SIMPLEX, 1, {255, 150, 150});
//            circle(image.map_draw,cv::Point(car.Locate3D.x/28*1330,713-car.Locate3D.y/15*713),0,cv::Scalar(255, 150, 150),15 );
//            putText(image.map_draw,cls_to_string[car.cls],cv::Point(car.Locate3D.x/28*1330,713-car.Locate3D.y/15*713),cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 150, 150), 1);
//        }else{
//            circle(image.map_draw,cv::Point(car.Locate3D.x/28*1330,713-car.Locate3D.y/15*713),0,cv::Scalar(150, 150, 150),15 );
//            putText(image.map_draw,"unknown",cv::Point(car.Locate3D.x/28*1330,713-car.Locate3D.y/15*713),cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 150, 150), 1);
//        }
//        cv::imshow(image.mapImage_winname,image.map_draw);
//        cv::imshow(image.camera_winname,image.radar_draw);
//    }
//
//}

