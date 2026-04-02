#include "../include/Image.h"

//带有序列号 g_strSerialNumber
Image::Image(Application application,std::string config_path,PictureSource pictureSource,char g_strSerialNumber[64],rclcpp::Node* node,std::string Name,int serial_number){
    this->pictureSource = pictureSource;
    this->application = application;
    this->serial_number = serial_number;//start
    this->config = YAML::LoadFile(config_path);
    this->classWithoutCar = config["general"]["classWithoutCar"].as<int>();
    this->Cam_img=cv::Mat::zeros(0, 0, CV_8UC3);
{//pictureSource path
    if(this->pictureSource == single_picture){
        image_path = config[Name]["source_path"]["singleImg_path"].as<std::string>();
    }
    else if(this->pictureSource == picture_dir){
        image_dir =  config[Name]["source_path"]["picture_dir"].as<std::string>();
        start_picture = config[Name]["source_path"]["picture_dir_start"].as<int>();
    }
    else if(this->pictureSource == video){
        video_path =  config[Name]["source_path"]["video_path"].as<std::string>();
    }
    else if(this->pictureSource == camera_){
        Cam_isOpen = true;
        this->Camerahk_prt = std::shared_ptr<Camera>(new Camera(g_strSerialNumber, Name, node,config_path));
    }
}
    Cam_winname = config[Name]["winname"].as<std::string>();
    input_w = config[Name]["picture_size"]["input_w"].as<int>();//
    input_h = config[Name]["picture_size"]["input_h"].as<int>();//

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
}

void Image::Init(){
    if(pictureSource==video ){
        cap = cv::VideoCapture(video_path);
    }
    if(application == Radar){
        map_img = cv::imread(this->mapImage_path);
        map_cloneing = map_img.clone();
    }
}

void Image::Init_calib(){
    if(pictureSource==video ){
        cap = cv::VideoCapture(video_path);
    }
    cv::namedWindow(this->Cam_winname,0);
    cv::resizeWindow(this->Cam_winname,cv::Size(input_w,input_h));
    if(application == Radar){
        map_img = cv::imread(this->mapImage_path);
        map_cloneing = map_img.clone();
    }
}

cv::Mat Image::Image_Get(int &after_picture){
    if(this->pictureSource==picture_dir) {
        if(this->serial_number == -1){
            dynamic_image_path = this->image_dir + std::to_string(this->start_picture + after_picture) +".jpg";
        }else{
            dynamic_image_path = this->image_dir + std::to_string(this->serial_number + after_picture) +".jpg";
        }
    }
    // get picture
    switch (this->pictureSource) {
        case video:
            (this->cap).read(this->Cam_img);
            break;
        case camera_:
            this->Cam_img = this->Camerahk_prt->HikCamera_sptr_->getImage();
            if (this->Cam_winname=="Hik60")
                this->ros_time=this->Camerahk_prt->HikCamera_sptr_->getTime();
            break;
        case single_picture:
            this->Cam_img = cv::imread(this->image_path);
            break;
        case picture_dir:
            this->Cam_img = cv::imread(this->dynamic_image_path);
            break;
        case ros:
            break;
        default:
            std::cerr << "no picture" << std::endl;
            break;
    }
    this->Cam_draw = this->Cam_img.clone();
    if(application==Application::Radar)
        this->map_draw = this->map_img.clone();
    if(cv::waitKey(1) == 'x')
        after_picture++;
    else if(cv::waitKey(1) == 'z')
        after_picture--;
    return this->Cam_img;
}

void Image::Image_Show(){
    cv::imshow(this->Cam_winname, this->Cam_draw);
    cv::waitKey(1);
}

void Image::Close(){
    if(pictureSource==camera_)
        Camerahk_prt->CamMainClose();
    cv::destroyAllWindows();
}

void Image::draw_lidar(interfaces::msg::DetectResult lidar) {
    for (int i=0;i<5;i++) {
        circle(map_draw,cv::Point(lidar.red_x[i]/28*map_w,map_h-lidar.red_y[i]/15*map_h),0,cv::Scalar(255,255,255),15 );
        circle(map_draw,cv::Point(lidar.blue_x[i]/28*map_w,map_h-lidar.blue_y[i]/15*map_h),0,cv::Scalar(255,255,255),15 );
    }
}

void Image::draw_result(std::vector<STrack> output_stracks, bool is_cls){
    if(is_cls){
        for (int i = 0; i < output_stracks.size(); i++){
            int cls = output_stracks[i].cls ;
            if(cls != -1){
                std::vector<float> tlwh = output_stracks[i].tlwh;
                cv::Scalar s = get_color(cls);
                putText(Cam_draw, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5),
                0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                if( -1 < cls && cls < classWithoutCar){
                    putText(Cam_draw,config["class_mapping"][cls].as<std::string>(),Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                    cv::putText(Cam_draw,std::to_string(output_stracks[i].conf_armor),cv::Point(output_stracks[i].tlbr[2]+10,output_stracks[i].tlbr[1]-30),cv::FONT_HERSHEY_SIMPLEX, 2, {100, 225, 100},2);
                } else{
                    putText(Cam_draw,"car",Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                }

                rectangle(Cam_draw, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,10, 5);
                if(application==Radar){
                    circle(map_draw,cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),0,s,15 );
                    if( -1 < cls && cls < classWithoutCar){
                        putText(map_draw,config["class_mapping"][cls].as<std::string>(),cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                    } else{
                        putText(map_draw,"car",cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                    }
                }
            }else if(application==Radar){
                cv::Scalar s = get_color(cls);
                circle(map_draw,cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),0,s,15 );
                rectangle(Cam_draw, Rect(output_stracks[i].tlwh[0], output_stracks[i].tlwh[1], output_stracks[i].tlwh[2], output_stracks[i].tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 10);
                putText(Cam_draw,"car",Point(output_stracks[i].tlwh[0]-5, output_stracks[i].tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 5, s, 2);
                putText(map_draw,"car",cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
            }
        }
    }else{
        for (int i = 0; i < output_stracks.size(); i++){
            std::vector<float> tlwh = output_stracks[i].tlwh;
            bool vertical = tlwh[2] / tlwh[3] > 1.6;
            if (tlwh[2] * tlwh[3] > 20 && !vertical){
                cv::Scalar s = get_color(output_stracks[i].track_id);
                putText(Cam_draw, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5),
                        0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
                int track_id = output_stracks[i].track_id;
                int cls = track_id%classWithoutCar;
                if( -1 < track_id && track_id < classWithoutCar){
                    putText(Cam_draw,config["class_mapping"][cls].as<std::string>(),Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                    cv::putText(Cam_draw,std::to_string(output_stracks[i].conf_armor),cv::Point(output_stracks[i].tlbr[2]+10,output_stracks[i].tlbr[1]-30),cv::FONT_HERSHEY_SIMPLEX, 1, {100, 225, 100},2);
                } else{
                    putText(Cam_draw,"car",Point(tlwh[0]-5, tlwh[1] - 10),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                }

                rectangle(Cam_draw, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
                cv::circle(Cam_draw,output_stracks[i].Locate2D,0,s, 5);
                if(application==Radar){
                    circle(map_draw,cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),0,s,15 );
                    if( -1 < cls && cls < classWithoutCar){
                        putText(map_draw,config["class_mapping"][cls].as<std::string>(),cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                    } else{
                        putText(map_draw,"car",cv::Point(output_stracks[i].Locate3D.x/28*map_w,map_h-output_stracks[i].Locate3D.y/15*map_h),cv::FONT_HERSHEY_PLAIN, 1, s, 2);
                    }
                }
            }
        }
    }
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
}

void Image::draw_line_calib(std::vector<MapVertex> &vexs){
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