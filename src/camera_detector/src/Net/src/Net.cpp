#include "../include/Net.h"

/**
 * @brief 初始化神经网络，加载参数
 */
Net::Net(std::string config_path,std::string Name) {
    //加载神经网络所需的参数
    YAML::Node config = YAML::LoadFile(config_path);
    this->isArmor = config[Name]["isArmor"].as<bool>();
    this->onnx_path = config[Name]["onnx_path"].as<std::string>();
    this->trt_path  = config[Name]["trt_path"].as<std::string>();
    this->conf_thres = config[Name]["conf_thres"].as<float>();
    this->nms_thres  = config[Name]["nms_thres"].as<float>();
    this->batch_size = config[Name]["batch_size"].as<int>();
    this->num_class  = config[Name]["num_class"].as<int>();
    this->input_h = config[Name]["input_h"].as<int>();
    this->input_w = config[Name]["input_w"].as<int>();
    this->classWithoutCar= config["general"]["classWithoutCar"].as<int>();

    //神经网络初始化
    myInfer.getDevice(0);
    const char* temp_trtpath = this->trt_path.c_str();
    if(access(temp_trtpath, F_OK) != 0){
        nvinfer1::IHostMemory *data = myInfer.createEngine(this->onnx_path, this->batch_size, this->input_h, this->input_w);
        myInfer.saveEngineFile(data, this->trt_path);
    }
    myInfer.initModule(this->trt_path, this->batch_size, this->num_class);
}

/**
 * @brief 多线程版调用神经网络
 */
std::vector<std::vector<TRTInferV1::DetectionObj>> Net::NetWork_mlt(std::vector<cv::Mat> &frames){
        return (this->myInfer.doInference(ref(frames), this->conf_thres, this->nms_thres));
}

/**
 * @brief 创建多线程神经网络
 */
void Net::Spin(std::vector<cv::Mat> &frames){
    // //模板传参的时候使用ref，否则传参失败
    auto func = std::bind(&Net::NetWork_mlt, this, ref(frames));
    this->futureObjs =  std::async(std::launch::async,func,ref(frames));
}

void Net::getCarImgs(std::vector<std::vector<TRTInferV1::DetectionObj>> Objs, cv::Mat img, std::vector<cv::Mat> &car_imgs){
    for(int i=0;i<Objs.size();i++) {
        std::vector<std::thread> threads;
        int thread_num=Objs[i].size();
        std::vector<cv::Mat> car_img(thread_num);
        for(int j=0;j<thread_num;j++) {
            threads.push_back(std::thread([i,j,&car_img,img,Objs](){
                car_img[j]=img.clone()(cv::Rect(cv::Point_<int>(Objs[i][j].x1,Objs[i][j].y1),cv::Point_<int>(Objs[i][j].x2,Objs[i][j].y2)));
            }));
        }
        for(auto &t:threads){
            t.join();
        }
        for(auto &car:car_img){
            car_imgs.push_back(car);
        }
    }
}

void Net::getCarImgs(const interfaces::msg::ClusterRect rects, cv::Mat &img,std::vector<cv::Mat> &car_imgs,int cam_id){
        std::vector<std::thread> threads;
        int thread_num;
        if (cam_id==1)thread_num=rects.rects1.size();
        else if (cam_id==2)thread_num=rects.rects2.size();
        std::vector<cv::Mat> car_img(thread_num);
        for(int j=0;j<thread_num;j++) {
            threads.push_back(std::thread([j,&car_img,&img,rects,cam_id](){
                if (cam_id==1) {
                    car_img[j]=img(cv::Rect(cv::Point(rects.rects1[j].x1,rects.rects1[j].y1),cv::Point(rects.rects1[j].x2,rects.rects1[j].y2)));
                }else if (cam_id==2) {
                    car_img[j]=img(cv::Rect(cv::Point(rects.rects2[j].x1,rects.rects2[j].y1),cv::Point(rects.rects2[j].x2,rects.rects2[j].y2)));
                }
            }));
        }
        for(auto &t:threads){
            t.join();
        }
        for(auto &car:car_img){
            car_imgs.push_back(car);
        }
}