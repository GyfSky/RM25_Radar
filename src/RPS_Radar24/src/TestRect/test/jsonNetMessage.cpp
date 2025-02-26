#include "../include/jsonNetMessage.h"

Modes::Modes(){
    //choose  一些选项
/////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------- 这里需要修改！！！！---------------------------------------------|
    application   = Application::Radar;
    pictureSource = PictureSource::camera_;    //图片来源          |
    ourPattern    = OurPattern::blue;                  //己方颜色       |
    Port_isOpen   = TF::false_;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    Image_isSave  = TF::false_;                       //是否保存图片    |
    saveImagePath = SaveImagePath::disk02;            //保存路径       |
};


JsonNetMessage::JsonNetMessage(int serial_number){
    config = YAML::LoadFile(YAML_CONFIC_PATH);
    net_config = YAML::LoadFile(YAML_NETCONFIC_PATH);
    jsonPathDir = config["json"]["jsonPathDir"].as<std::string>();
    random = config["json"]["random"].as<bool>();

    this->serial_number = serial_number;
}

void JsonNetMessage::getJsonNetMessage(std::vector<std::vector<TRTInferV1::DetectionObj>> &DetectionObjs, int after_picture){
    // input
    jsonPath = jsonPathDir + std::to_string(this->serial_number + after_picture) + ".json";
    //output 
    std::vector<TRTInferV1::DetectionObj> DetectionObjs_;
    //config
    std::ifstream infile(jsonPath);
    if(reader.parse(infile, root)) {
        Json::Value arrayObj = root["shapes"];
        for(unsigned int i = 0; i < arrayObj.size();i++){
            TRTInferV1::DetectionObj detectionObj;
            std::string label = arrayObj[i]["label"].asString();
            if( label == "BN" || label == "RN" || label == "ignore" ){
                continue;
            } else if(label == "carD"){
                label = "car";
            }
            detectionObj.classId = net_config[label].as<int>();
            detectionObj.x1 = arrayObj[i]["points"][0][0].asDouble();
            detectionObj.y1 = arrayObj[i]["points"][0][1].asDouble();
            detectionObj.x2 = arrayObj[i]["points"][1][0].asDouble();
            detectionObj.y2 = arrayObj[i]["points"][1][1].asDouble();
            if(random){
                std::random_device e; 
                std::uniform_real_distribution<float> u(0.7, 0.9); //随机数分布对象
                detectionObj.confidence = u(e);
            }else{
                detectionObj.confidence = 0.95;
            }
            DetectionObjs_.push_back(detectionObj);
        }
        DetectionObjs.push_back(DetectionObjs_);
    } else{
        std::cout << "error" << std::endl;
    }
    root.clear();
}

int main(int argc, char **argv){
//    int serial_number = 1317;
    int serial_number = 1160;

    Modes modes;

    auto JsonNetMessage_ptr = std::shared_ptr<JsonNetMessage>(new JsonNetMessage(serial_number));
    auto CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem());
    auto MapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx(modes.ourPattern));
    auto Predict_ptr = std::shared_ptr<Predict>(new Predict());
    auto Image_ptr = std::shared_ptr<Image>(new Image(modes.application,single_picture,"Hik30",Hikang,false_,disk02,serial_number));
    auto Net_ptr   = std::shared_ptr<Net>(new Net("Hik30"));
    auto PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs());
    auto BYTETracker_ptr = std::shared_ptr<BYTETracker>(new BYTETracker(300, 300));

    std::vector<std::vector<TRTInferV1::DetectionObj>> DetectionObjs;
    std::vector<Car> cars;
    std::vector<Armor> armors;
    std::vector<STrack>  tracked_stracks, lost_stracks;

    std::vector<Car> redCars,blueCars,restCars,lastCars;

    std::vector<Object> objects;

    cv::Mat mainCamMat;
    
    Image_ptr->Init(argc, argv);
    int after = 0;

    if(Image_ptr->is_getPoint2d_mouse_Cam){
        mainCamMat = Image_ptr->Image_Get(after,argc,argv);
        CooSystem_ptr->pts_pnp_2d_MainCam = GetPoint2d_mouse(mainCamMat,"Hik30");
        CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Main2world  ,CooSystem_ptr->K_M ,CooSystem_ptr->pts_pnp_2d_MainCam , MapGraph_ptr->pts_pnp_3d);
        Image_ptr->is_getPoint2d_mouse_Cam = false;
        MapGraph_ptr->get_predict_2d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,CooSystem_ptr->cx_M,CooSystem_ptr->cy_M);
        MapGraph_ptr->get_roughH_config();
    }else{
        std::cerr << "error" << std::endl;
    }
    int bafter = after;
    //----------MAIN-----------------
    while(true){

        cars.clear();
        lastCars.clear();
        armors.clear();
        DetectionObjs.clear();
        mainCamMat = Image_ptr->Image_Get(after,argc,argv);
        cv::Mat img_draw = mainCamMat.clone();
//        Image_ptr->Image_Show();
        JsonNetMessage_ptr->getJsonNetMessage(DetectionObjs, after-1);

        if(bafter !=after)
        {
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    //-----------------------------change1--------------------------------------
    //json
    ////net
    ////        //mlt-thread
    ////        Net_ptr->Spin(mainCamMat);
    ////        DetectionObjs = Net_ptr->futureObjs.get();
            ////common
//             Net_ptr->NetWork(mainCamMat,DetectionObjs);
//    -------------------------------------------------------------------
            Net_ptr->Car_Armor(DetectionObjs[0],cars,armors);
//            Image_ptr->draw_rusult(cars,armors,img_draw);
            PretreatObjs_ptr->getArmors_wconf(cars,redCars,blueCars,restCars,armors);
            PretreatObjs_ptr->getLastCar(armors,lastCars);
            cars.insert(cars.end(),lastCars.begin(),lastCars.end());
            std::cout << "cars.size()" << cars.size() << std::endl;
            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, MapGraph_ptr->vexs,MapGraph_ptr->arcs,cars, modes.ourPattern);

            BYTETracker_ptr->update(tracked_stracks,lost_stracks, cars);
            std::cout << "updata is OK" << std::endl;
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, MapGraph_ptr->vexs,MapGraph_ptr->arcs,tracked_stracks, modes.ourPattern);
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);

            std::cout << "lost_stracks " << lost_stracks.size() << std::endl;
            Predict_ptr->loss_track_predict(lost_stracks);
//            Predict_ptr->loss_track_predict(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                            CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);
//            Predict_ptr->getRectFromLocate3D(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,
//                                             CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, lost_stracks);
            std::cout << "lost_stracks.size():  " << lost_stracks.size() << std::endl;
            Image_ptr->draw_line(MapGraph_ptr->vexs);
            Image_ptr->draw_rusult(cars ,armors, false);
            std::cout << (after + serial_number) << std::endl;
            bafter = after;
            Image_ptr->draw_rusult(tracked_stracks);
            Image_ptr->draw_rusult(lost_stracks);
            auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "Fps: " << 1.0/(endTime-startTime)*1000.0 << std::endl;
        }
//        cv::waitKey(300);
//        after++;

    }
    return 0;
}