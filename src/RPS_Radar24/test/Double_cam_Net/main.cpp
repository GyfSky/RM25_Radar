//
// Created by plusseven on 24-2-1.
//

#include "main.h"

Modes::Modes(){
    //choose  一些选项
/////////////////////////////////////////////////////////////////////////////////////////////////////////
////------------------------------------- 这里需要修改！！！！---------------------------------------------|
    application   = Application::Common;
    pictureSource = PictureSource::single_picture;    //图片来源          |
    ourPattern    = OurPattern::blue;                  //己方颜色       |
    Port_isOpen   = TF::false_;                       //串口的开启与否  |
    usePort       = UsePort::USB0;                    //所使用的串口    |
    Image_isSave  = TF::false_;                       //是否保存图片    |
    saveImagePath = SaveImagePath::disk02;            //保存路径       |
};

int main(int argc, char **argv) {
//    int serial_number = 1317;
    int serial_number = 1160;

    Modes modes;
    std::vector<std::string> camNames = {"K_Hik60", "K_S"};
    std::vector<cv::Point3d> pts_pnp_3d;
    std::vector<cv::Point2d> pts_pnp_2d_main, pts_pnp_2d_sec;
    pts_pnp_3d.emplace_back(cv::Point3d(9.5500, 0.0000, 0.40));
    pts_pnp_3d.emplace_back(cv::Point3d(8.4100, 4.3670, 1.10));
    pts_pnp_3d.emplace_back(cv::Point3d(7.1600, 4.3670, 1.10));
    pts_pnp_3d.emplace_back(cv::Point3d(5.1390, 2.2020, 0.00));
    pts_pnp_3d.emplace_back(cv::Point3d(6.5870, 6.7770, 0.6));
    pts_pnp_3d.emplace_back(cv::Point3d(9.4100, 5.5000, 0.5));

   pts_pnp_2d_main.emplace_back(cv::Point2d(335,109));
   pts_pnp_2d_main.emplace_back(cv::Point2d(584,171));
   pts_pnp_2d_main.emplace_back(cv::Point2d(1044,151));
   pts_pnp_2d_main.emplace_back(cv::Point2d(1357,273));
   pts_pnp_2d_main.emplace_back(cv::Point2d(1731,705));
   pts_pnp_2d_main.emplace_back(cv::Point2d(158,606));//7

//    pts_pnp_2d_main.emplace_back(cv::Point2d(745,127 ));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(967,158 ));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(1428,150));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(1718,304));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(2128,735));
//    pts_pnp_2d_main.emplace_back(cv::Point2d(534,556 ));//10
//
   pts_pnp_2d_sec.emplace_back(cv::Point2d(875,1236 ));
   pts_pnp_2d_sec.emplace_back(cv::Point2d(1079,1300));
   pts_pnp_2d_sec.emplace_back(cv::Point2d(1483,1312));
   pts_pnp_2d_sec.emplace_back(cv::Point2d(1752,1432));
   pts_pnp_2d_sec.emplace_back(cv::Point2d(2213,1912));
   pts_pnp_2d_sec.emplace_back(cv::Point2d(637,1634));//8
//

    // pts_pnp_2d_main.emplace_back(cv::Point2d(700,1363 ));
    // pts_pnp_2d_main.emplace_back(cv::Point2d(879,1285 ));
    // pts_pnp_2d_main.emplace_back(cv::Point2d(1276,1280));
    // pts_pnp_2d_main.emplace_back(cv::Point2d(1577,1478));
    // pts_pnp_2d_main.emplace_back(cv::Point2d(1901,1578));
    // pts_pnp_2d_main.emplace_back(cv::Point2d(455,1559 ));//11

    // pts_pnp_2d_sec.emplace_back(cv::Point2d(262,84  ));
    // pts_pnp_2d_sec.emplace_back(cv::Point2d(436,24  ));
    // pts_pnp_2d_sec.emplace_back(cv::Point2d(834,39  ));
    // pts_pnp_2d_sec.emplace_back(cv::Point2d(1114,254));
    // pts_pnp_2d_sec.emplace_back(cv::Point2d(1395,396));
    // pts_pnp_2d_sec.emplace_back(cv::Point2d(28,281  ));//22
    cv::Mat dist = (cv::Mat_<double>(1,4)<< -0.108117059568587,0.0525202375483879,0,0) ;

    auto CooSystem_ptr = std::shared_ptr<MatrixCoordinateSystem>(new MatrixCoordinateSystem(camNames));
    auto main_Image_ptr = std::shared_ptr<Image>(
            new Image(modes.application, single_picture, "DA0926631", "Hik30", false_, disk02,
                      serial_number));
    auto sec_Image_ptr = std::shared_ptr<Image>(
            new Image(modes.application, single_picture, "DA0926631", "Hik30", false_, disk02,
                      serial_number));
    auto main_Net_ptr = std::shared_ptr<Net>(new Net("Hik30"));
    auto sec_Net_ptr = std::shared_ptr<Net>(new Net("Hik30"));
    auto costMatrix_ptr = std::shared_ptr<CostMatrix>(new CostMatrix());


    std::vector<std::vector<TRTInferV1::DetectionObj>> main_DetectionObjs, sec_DetectionObjs;
    std::vector<Car> cars;
    std::vector<Armor> armors;

    std::vector<Car> redCars, blueCars, restCars, lastCars;

    std::vector<Object> objects;

    cv::Mat mainCamMat, secCamMat;

    main_Image_ptr->Init();
    sec_Image_ptr->Init();
    int after = 0;

    if (main_Image_ptr->is_getPoint2d_mouse_Cam) {
        mainCamMat = main_Image_ptr->Image_Get(after,argc,argv);
        secCamMat = sec_Image_ptr->Image_Get(after,argc,argv);
        CooSystem_ptr->pts_pnp_2d_MainCam = GetPoint2d_mouse(mainCamMat, "Hik30");
        CooSystem_ptr->pts_pnp_2d_SecondCam = GetPoint2d_mouse(secCamMat, "Hik30");
//        CooSystem_ptr->pts_pnp_2d_MainCam = pts_pnp_2d_main;
//        CooSystem_ptr->pts_pnp_2d_SecondCam = pts_pnp_2d_sec;
        CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Main2world, CooSystem_ptr->K_M,
                                        CooSystem_ptr->pts_pnp_2d_MainCam, pts_pnp_3d);
        CooSystem_ptr->Get2world_matrix(CooSystem_ptr->T_Second2world, CooSystem_ptr->K_S,
                                        CooSystem_ptr->pts_pnp_2d_SecondCam, pts_pnp_3d);
        main_Image_ptr->is_getPoint2d_mouse_Cam = false;
    } else {
        std::cerr << "error" << std::endl;
    }
    auto sec2main_ptr = std::shared_ptr<Sec2main>(
            new Sec2main((CooSystem_ptr->T_Second2world).inv(), CooSystem_ptr->fx_S, CooSystem_ptr->fy_S, CooSystem_ptr->cx_S,
                         CooSystem_ptr->cy_S,
                         CooSystem_ptr->T_Main2world, CooSystem_ptr->fx_M, CooSystem_ptr->fy_M, CooSystem_ptr->cx_M,CooSystem_ptr->cy_M));
    auto main2main_ptr = std::shared_ptr<Sec2main>(
                new Sec2main((CooSystem_ptr->T_Main2world).inv(), CooSystem_ptr->fx_M, CooSystem_ptr->fy_M, CooSystem_ptr->cx_M,CooSystem_ptr->cy_M,
                             CooSystem_ptr->T_Main2world, CooSystem_ptr->fx_M, CooSystem_ptr->fy_M, CooSystem_ptr->cx_M,CooSystem_ptr->cy_M));

    auto test_ptr = std::shared_ptr<Sec2main>(
            new Sec2main((CooSystem_ptr->T_Main2world).inv() * (CooSystem_ptr->T_Second2world),CooSystem_ptr->K_M, CooSystem_ptr->K_S));
    int bafter = after;
    //----------MAIN-----------------
    while (true) {

        cars.clear();
        lastCars.clear();
        armors.clear();
        main_DetectionObjs.clear();
        sec_DetectionObjs.clear();
        mainCamMat = main_Image_ptr->Image_Get(after,argc,argv);
        secCamMat = sec_Image_ptr->Image_Get(after,argc,argv);
        cv::Mat main_img_draw = mainCamMat.clone();
        cv::Mat sec_img_draw = secCamMat.clone();



//        if (bafter != after) {
        auto startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        main_Net_ptr->Spin(mainCamMat);
        main_DetectionObjs = main_Net_ptr->futureObjs.get();
        sec_Net_ptr->Spin(secCamMat);
        sec_DetectionObjs = sec_Net_ptr->futureObjs.get();


        sec_Net_ptr->Car_Armor(sec_DetectionObjs, cars, armors);
        sec_Image_ptr->draw_rusult(cars, armors, true);
        cars.clear();
        armors.clear();

        for(int b=0;b<sec_DetectionObjs.size();b++){
            for(int i=0;i<sec_DetectionObjs[b].size();i++){
                float x = (sec_DetectionObjs[b][i].x2+sec_DetectionObjs[b][i].x1)/2.,
                      y = (sec_DetectionObjs[b][i].y1+sec_DetectionObjs[b][i].y2)/2.,
                      w = (sec_DetectionObjs[b][i].x2-sec_DetectionObjs[b][i].x1)/sec2main_ptr->fx*main2main_ptr->fx,
                      h = (sec_DetectionObjs[b][i].y2-sec_DetectionObjs[b][i].y1)/sec2main_ptr->fy*main2main_ptr->fy;
                sec2main_ptr->change2mainCam(x,y);
//                test_ptr->sec2mainCam(x,y);

                sec_DetectionObjs[b][i].x1 = x - w/2;
                sec_DetectionObjs[b][i].x2 = x + w/2;
                sec_DetectionObjs[b][i].y1 = y - h/2;
                sec_DetectionObjs[b][i].y2 = y + h/2;
            }
            vector<vector<float> > dists;
            int dist_size, dist_size_size;
            vector<vector<int> > matches;
            vector<int> u_track, u_detection;
            vector<TRTInferV1::DetectionObj> res;
            float match_thresh = 0.85;
            Eigen::MatrixXd cost_matrix = costMatrix_ptr->getIouAndDistancetCost(main_DetectionObjs[b],sec_DetectionObjs[b],dist_size, dist_size_size);
            eigenMat2VecVec(cost_matrix,dists);
            costMatrix_ptr->linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);
//            std::cout << "cost_matrix: \n" <<cost_matrix << std::endl;
            for(int j=0;j<dist_size_size;j++){
                int flag = 1;
                for(int i=0;i<matches.size();i++ ){
                    if(j==matches[i][1]){
                        flag = -1;
                        break;
                    }
                }
                if(flag==1){
                    main_DetectionObjs[b].push_back(sec_DetectionObjs[b][j]);
                }
            }

        }

        main_Net_ptr->Car_Armor(sec_DetectionObjs,cars,armors);
        main_Net_ptr->Car_Armor(main_DetectionObjs, cars, armors);
        main_Image_ptr->draw_rusult(cars, armors, true);

        std::cout << "-----------------------------" << std::endl;

        for(int b=0;b<main_DetectionObjs.size();b++){
            for(int i=0;i<main_DetectionObjs[b].size();i++){

                float x = (main_DetectionObjs[b][i].x2+main_DetectionObjs[b][i].x1)/2.,
                        y = main_DetectionObjs[b][i].y2,
                        w = (main_DetectionObjs[b][i].x2-main_DetectionObjs[b][i].x1),
                        h = (main_DetectionObjs[b][i].y2-main_DetectionObjs[b][i].y1);
                main2main_ptr->change2mainCam(x,y);
                main_DetectionObjs[b][i].x1 = x - w/2;
                main_DetectionObjs[b][i].x2 = x + w/2;
                main_DetectionObjs[b][i].y1 = y - h;
                main_DetectionObjs[b][i].y2 = y ;
            }
        }



        std::cout << "T： \n" <<  ((CooSystem_ptr->T_Main2world).inv() * CooSystem_ptr->T_Second2world) << std::endl;
        std::cout << "T-1: \n" << ((CooSystem_ptr->T_Main2world).inv() * CooSystem_ptr->T_Second2world).inv() << std::endl;

        bafter = after;
        auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        std::cout << "Fps: " << 1.0 / (endTime - startTime) * 1000.0 << std::endl;
//        }
        cv::waitKey(30000);
//        after++;

    }

}

