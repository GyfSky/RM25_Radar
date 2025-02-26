#include "../include/Mouse.h"

//e.g 1
// test main
int main(){
    Mouse mouse;
    mouse.flag_num = 0;
    // mouse.flag_back = 0;
    mouse.pointNumber = 5;
    mouse.winname = "window_radar";
    mouse.image = cv::imread("/home/plusseven/RM_radardemo24/src/RadarPackage/doc/1439.jpg");
    cv::namedWindow(mouse.winname);
//key
    while (true){
        cv::setMouseCallback("window_radar", onMouse, &mouse);
        cv::imshow("window_radar", mouse.image);
        cv::waitKey(1);
    }
    // std::cout << mouse.point2d_mouse_xy.;
    return 0;
}

//e.g.  2
// {
//     Mouse mouse(imshowDepthBackgoundMat,"Livox");
//     while (mouse.point2d_mouse_xy.size() != mouse.pointNumber){
//         cv::setMouseCallback(mouse.winname, onMouse, &mouse);
//         cv::imshow(mouse.winname, mouse.image);
//         cv::waitKey(1);
//     }
//     std::cout << "over mouse_livox:" + std::to_string(mouse.point2d_mouse_xy[0].x) << std::endl;
//     return mouse.point2d_mouse_xy;
// }