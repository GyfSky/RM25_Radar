//
// Created by plusseven on 23-7-15.
//

#include "../include/Net.h"


/**
 * @brief 初始化神经网络，加载参数
 */
Net::Net(std::string Name) {
    //加载神经网络所需的参数
    YAML::Node config = YAML::LoadFile(YAML_CONFIC_PATH);
    this->isArmor = config[Name]["isArmor"].as<bool>();
    this->onnx_path = config[Name]["onnx_path"].as<std::string>();
    this->trt_path  = config[Name]["trt_path"].as<std::string>();
    this->obj_thres = config[Name]["obj_thres"].as<float>();
    this->conf_thres = config[Name]["conf_thres"].as<float>();
    this->nms_thres  = config[Name]["nms_thres"].as<float>();
    this->batch_size = config[Name]["batch_size"].as<int>();
    this->num_class  = config[Name]["num_class"].as<int>();
    this->input_h = config[Name]["input_h"].as<int>();
    this->input_w = config[Name]["input_w"].as<int>();

    this->net_config = YAML::LoadFile(YAML_NETCONFIC_PATH);
    this->classWithoutCar= net_config["classWithoutCar"].as<int>();
    //神经网络初始化
    myInfer.getDevice(0);
    const char* temp_trtpath = this->trt_path.c_str();
    if(access(temp_trtpath, F_OK) != 0){
        nvinfer1::IHostMemory *data = myInfer.createEngine(this->onnx_path, this->batch_size, this->input_h, this->input_w);
        myInfer.saveEngineFile(data, this->trt_path);
    }
    myInfer.initModule(this->trt_path, this->batch_size, this->num_class);
}


// /**
//  * @brief 调用神经网络
//  */
// void Net::NetWork(cv::Mat img ,std::vector<std::vector<TRTInferV1::DetectionObj>> &DetectionObjs){
//     std::vector<cv::Mat> frames;
//     frames.push_back(img);
//     DetectionObjs = this->myInfer.doInference(frames, this->obj_thres, this->conf_thres, this->nms_thres);
// }


/**
 * @brief 多线程版调用神经网络
 */
std::vector<std::vector<TRTInferV1::DetectionObj>> Net::NetWork_mlt(std::vector<cv::Mat> &frames){

        return (this->myInfer.doInference(ref(frames), this->obj_thres, this->conf_thres, this->nms_thres,0));

}


std::vector<std::vector<TRTInferV1::Object>> Net::NetWork_confs_mlt(std::vector<cv::Mat> &frames){
    return (this->myInfer.doInference_confs(ref(frames), this->obj_thres, this->conf_thres, this->nms_thres));
}


// /**
// * @brief 多线程版调用神经网络(more pic)
// */
// std::vector<std::vector<TRTInferV1::DetectionObj>> Net::NetWork_mlt(std::vector<cv::Mat> frames){
//    return (this->myInfer.doInference(frames, this->obj_thres, this->conf_thres, this->nms_thres));
// }


/**
 *
 * @brief 创建多线程神经网络
 */
void Net::Spin(std::vector<cv::Mat> &frames){
    // //模板传参的时候使用ref，否则传参失败
    // this->netMainloop = std::thread(std::bind(&Net::NetWork, this, img, ref(promiseObjs)));
    auto func = std::bind(&Net::NetWork_mlt, this, ref(frames));
    this->futureObjs =  std::async(std::launch::async,func,ref(frames));
}

void Net::Spin_confs(std::vector<cv::Mat> &frames){
    // //模板传参的时候使用ref，否则传参失败
    // this->netMainloop = std::thread(std::bind(&Net::NetWork, this, img, ref(promiseObjs)));
    auto func_change = std::bind(&Net::NetWork_confs_mlt, this, ref(frames));
    this->futureObjs_change =  std::async(std::launch::async,func_change,ref(frames));
}

// void Net::getCarImgs(std::vector<std::vector<TRTInferV1::DetectionObj>> Objs, cv::Mat img, std::vector<cv::Mat> &car_imgs){
//     for(int i=0;i<Objs.size();i++){
//         for(auto obj : Objs[i]){
//             cv::Mat car_img = (img.clone())(cv::Rect(cv::Point_<int>(obj.x1,obj.y1),cv::Point_<int>(obj.x2,obj.y2)));
//             car_imgs.emplace_back(car_img);
//         }
//     }
// }

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


void Net::getCarImgs(std::vector<std::vector<TRTInferV1::Object>> Objs, cv::Mat img, std::vector<cv::Mat> &car_imgs){
    for(int i=0;i<Objs.size();i++){
        for(auto obj : Objs[i]){
            cv::Mat car_img = (img.clone())(cv::Rect(cv::Point_<int>(obj.x1,obj.y1),cv::Point_<int>(obj.x2,obj.y2)));
            car_imgs.push_back(car_img);
        }
    }
}

// /**
// * @brief 创建多线程神经网络（more pic）
// */
// void Net::Spin(std::vector<cv::Mat> frames){
//    // //模板传参的时候使用ref，否则传参失败
//    // this->netMainloop = std::thread(std::bind(&Net::NetWork, this, img, ref(promiseObjs)));
//    auto func = std::bind(&Net::NetWork_mlt, this, frames);
//    this->futureObjs =  std::async(std::launch::async,func,frames);
// }

