#include <string>
#include "../../General/include/General.h"
#include "json/json.h"

# define YAML_NET_PATH "/home/plusseven/下载/RM_radardemo24/src/RPS_Radar24/src/TargetTracking/doc/net.yaml"


int main(){
    // yaml
    YAML::Node net_config = YAML::LoadFile(YAML_NETCONFIC_PATH);

    std::string jsonPath;
    std::string jsonPathDir = "/media/plusseven/KESU/img_DATA/2023radarData/2023-06-12_01_10_45_data/";
    Json::Reader reader;
    Json::Value root;
    int start = 1150;
    int end = 1152;

    std::vector<Car> Cars;

    for(int k = start;k < end; k++){
        // output
        Car car;
        Armor armor;

        // input
        jsonPath = jsonPathDir + std::to_string(start) + ".json";
        //config
        std::ifstream infile(jsonPath);
        if(reader.parse(infile, root)) {
            Json::Value arrayObj = root["shapes"];
            for(unsigned int i = 0; i < arrayObj.size();i++){
                std::string label = arrayObj[i]["label"].asString();
                armor.cls = net_config[label].as<int>();
                double pointx, pointy;
                pointx = arrayObj[i]["points"][0][0].asDouble();
                pointy = arrayObj[i]["points"][0][1].asDouble();
                cv::Point2d lefttop = cv::Point2d(pointx,pointy);
                pointx = arrayObj[i]["points"][1][0].asDouble();
                pointy = arrayObj[i]["points"][1][1].asDouble();
                cv::Point2d rightdown = cv::Point2d(pointx,pointy);
                armor.rect = cv::Rect(lefttop,rightdown);

                //test
                std::cout << "armor.cls : " << armor.cls << std::endl;
                std::cout << "armor.rect : " << armor.rect << std::endl;
            }
        } else{
            std::cout << "error" << std::endl;
        }

        root.clear();
    }

    return 0;
}