//
// Created by plusseven on 24-1-17.
//

#include "test_new_graph.h"

int main(int argc, char **argv){
    int serial_number = 1400;

    auto CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem());
    auto MapGraph_ptr = std::shared_ptr<MapGraphMtx>(new MapGraphMtx());
    auto Image_ptr = std::shared_ptr<Image>(new Image(Radar,picture_dir,"Hik30",Hikang,false_,disk02,serial_number));
    auto Net_ptr   = std::shared_ptr<Net>(new Net("Hik30"));
    auto PretreatObjs_ptr = std::shared_ptr<PretreatObjs>(new PretreatObjs());
    auto BYTETracker_ptr = std::shared_ptr<BYTETracker>(new BYTETracker(300, 300));

    std::vector<std::vector<TRTInferV1::DetectionObj>> DetectionObjs;
    std::vector<Car> cars;
    std::vector<Armor> armors;

    std::vector<Car> redCars,blueCars,restCars,lastCars;

    std::vector<Object> objects;
    std::vector<STrack> output_stracks;

    cv::Mat mainCamMat;

    Image_ptr->Init();
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


        if(bafter !=after)
        {
            auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

//            Net_ptr->Car_Armor(DetectionObjs,cars,armors);
//            PretreatObjs_ptr->getArmors_wconf(cars,redCars,blueCars,restCars,armors);
//            PretreatObjs_ptr->getLastCar(armors,lastCars);
//            cars.insert(cars.end(),lastCars.begin(),lastCars.end());
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, MapGraph_ptr->vexs,cars);
//            std::cout << "cars.size()" << cars.size() << std::endl;
//            output_stracks = BYTETracker_ptr->update(cars);
//            CooSystem_ptr->solve_reality_3d(CooSystem_ptr->T_Main2world,CooSystem_ptr->fx_M,CooSystem_ptr->fy_M,CooSystem_ptr->cx_M,CooSystem_ptr->cy_M, (*Place_ptr),output_stracks);
            Image_ptr->draw_line(MapGraph_ptr->vexs);
            std::cout << (after + serial_number) << std::endl;
            bafter = after;
            auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            std::cout << "Fps: " << 1.0/(endTime-startTime)*1000.0 << std::endl;
        }
//        cv::waitKey(300);
//        after++;


    }
    return 0;

}