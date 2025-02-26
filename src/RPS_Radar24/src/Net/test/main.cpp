#include "../include/Inference.h"

int main()
{
    cv::namedWindow("Test", cv::WINDOW_NORMAL);
     cv::namedWindow("Test2", cv::WINDOW_NORMAL);
     cv::namedWindow("Test3", cv::WINDOW_NORMAL);
     cv::namedWindow("Test4", cv::WINDOW_NORMAL);
    TRTInferV1::TRTInfer myInfer(0);
//    nvinfer1::IHostMemory *data = myInfer.createEngine("/home/plusseven/下载/TRTInferenceForYolov5/best_armor.onnx", 16, 224, 224);
//    myInfer.saveEngineFile(data, "/home/plusseven/下载/TRTInferenceForYolov5/best_armor.trt");
//    myInfer.initModule("/home/plusseven/下载/TRTInferenceForYolov5/best_armor.trt", 16, 12);
    myInfer.initModule("/home/thesky/RM_radardemo24/src/RPS_Radar24/model/best_armor.trt", 16, 12);

    // cv::VideoCapture cap(0);
    std::vector<cv::Mat> frames;

     cv::Mat src = cv::imread("/home/plusseven/文档/1.png");
     cv::Mat src2 = cv::imread("/home/plusseven/文档/2.png");
     cv::Mat src3 = cv::imread("/media/plusseven/EA61-A1AF/datasets/byhand_onlycar/695_2.png");
     cv::Mat src4 = cv::imread("/media/plusseven/EA61-A1AF/datasets/byhand_onlycar/695_1.png");

    // myInfer.calculate_inter_frame_compensation(120);

    while (true)
    {
        frames.clear();
         cv::Mat img = src.clone();
         cv::Mat img2 = src2.clone();
         cv::Mat img3 = src3.clone();
         cv::Mat img4 = src4.clone();
        // if (!cap.isOpened())
        // {
        //     continue;
        // }
//         cv::Mat img, img2, img3, img4;
        // cap.read(img);
        // img2 = img.clone();
        // img3 = img.clone();
        // img4 = img.clone();
        // assert(!img.empty());
        frames.emplace_back(img.clone());
         frames.emplace_back(img2);
         frames.emplace_back(img3);
         frames.emplace_back(img4);
        auto start_t = std::chrono::system_clock::now().time_since_epoch();
        std::vector<std::vector<TRTInferV1::DetectionObj>> result = myInfer.doInference(frames, 0.2, 0.5, 0.45);
        auto end_t = std::chrono::system_clock::now().time_since_epoch();
        char ch[255];
        for (int i(0); i < int(frames.size()); ++i)
        {
            for (int j(0); j < int(result[i].size()); ++j)
            {
                cv::Rect r = cv::Rect(result[i][j].x1, result[i][j].y1, result[i][j].x2 - result[i][j].x1, result[i][j].y2 - result[i][j].y1);
                cv::rectangle(frames[i], r, cv::Scalar(255, 255, 255), 1);
                cv::putText(frames[i], std::to_string(result[i][j].classId), cv::Point (result[i][j].x1, result[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255, 100, 255),1);
                cv::putText(frames[i], std::to_string(result[i][j].confidence), cv::Point (result[i][j].x2, result[i][j].y1),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(100, 100, 255),1);
//                cv::putText(frames[i], std::to_string(result[i][j].),)  `
                std::cout << result[i][j].x1 << " " << result[i][j].y1 << " " << result[i][j].x2 << " " << result[i][j].y2 << std::endl;
                std::cout << result[i][j].classId << "|" << result[i][j].confidence << std::endl;
            }
            sprintf(ch, "FPS %d", int(std::chrono::nanoseconds(1000000000).count() / (end_t - start_t).count()));
            std::string fps_str = ch;
            cv::putText(frames[i], fps_str, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
        }

        std::cout << "-------------------" << std::endl << std::endl;

        // std::cout << ch << std::endl;
        cv::imshow("Test", frames[0]);
         cv::imshow("Test2", frames[1]);
         cv::imshow("Test3", frames[2]);
         cv::imshow("Test4", frames[3]);
        cv::waitKey(0);
    }

    return 0;
}