#include <string>
//#include "../../General/include/General.h"
#include "../../Net/include/Net.h"
#include "../../General/include/Mouse.h"
//#include "../../Camera_hk/include/Camera_mlt.h"
#include "../../Locate/include/Predict.h"
#include "../../Locate/include/KRepresent.h"

#include "../../ByteTrack/include/PretreatObjs.h"

#include "json/json.h"

class JsonNetMessage{
private:
    YAML::Node config;
    YAML::Node net_config;
    bool random = false;
    std::string jsonPath;
    std::string jsonPathDir;
    Json::Reader reader;
    Json::Value root;
    int serial_number;
public:
    JsonNetMessage(int serial_number);
    void getJsonNetMessage(std::vector<std::vector<TRTInferV1::DetectionObj>> &DetectionObjs, int after_picture);

};
