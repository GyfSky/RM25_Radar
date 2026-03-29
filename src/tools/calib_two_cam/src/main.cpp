#include "ImageStitch.h"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <cctype>
#include <fstream>
#include <sstream>

namespace {
bool ReadAllText(const std::string& path, std::string& out) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    out = buffer.str();
    return true;
}

bool WriteAllText(const std::string& path, const std::string& content) {
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << content;
    return true;
}

size_t FindTopLevelKey(const std::string& content, const std::string& key) {
    const std::string expected = key + ":";
    size_t pos = 0;
    while (pos < content.size()) {
        size_t end = content.find('\n', pos);
        if (end == std::string::npos) {
            end = content.size();
        }
        if (end - pos >= expected.size() && content.compare(pos, expected.size(), expected) == 0) {
            return pos;
        }
        pos = end + (end < content.size() ? 1 : 0);
    }
    return std::string::npos;
}

bool IsTopLevelKeyLine(const std::string& line) {
    if (line.empty()) {
        return false;
    }
    const char first = line[0];
    if (!(std::isalpha(static_cast<unsigned char>(first)) || first == '_')) {
        return false;
    }
    return line.find(':') != std::string::npos;
}

size_t FindNextTopLevelKey(const std::string& content, size_t from) {
    size_t pos = from;
    while (pos < content.size()) {
        size_t end = content.find('\n', pos);
        if (end == std::string::npos) {
            end = content.size();
        }
        const std::string line = content.substr(pos, end - pos);
        if (IsTopLevelKeyLine(line)) {
            return pos;
        }
        pos = end + (end < content.size() ? 1 : 0);
    }
    return std::string::npos;
}

bool UpdateMatNode(const std::string& file_path,
                   const cv::Mat& perspective_k,
                   const cv::Mat& h,
                   const cv::Mat& h2) {
    cv::Mat merged_perspective_k;
    cv::Mat merged_h;
    cv::Mat merged_h2;

    // 先从文件读取已有 Mat 节点，后续按需覆盖。
    cv::FileStorage reader(file_path, cv::FileStorage::READ);
    if (reader.isOpened()) {
        const cv::FileNode mat_node = reader["Mat"];
        if (!mat_node.empty()) {
            mat_node["perspective_K"] >> merged_perspective_k;
            mat_node["H"] >> merged_h;
            mat_node["H2"] >> merged_h2;
        }
        reader.release();
    }

    if (!perspective_k.empty()) {
        merged_perspective_k = perspective_k;
    }
    if (!h.empty()) {
        merged_h = h;
    }
    if (!h2.empty()) {
        merged_h2 = h2;
    }

    const std::string mat_temp_path = file_path + ".mat.tmp";
    cv::FileStorage writer(mat_temp_path, cv::FileStorage::WRITE);
    if (!writer.isOpened()) {
        std::cerr << "Failed to open yaml for writing: " << mat_temp_path << std::endl;
        return false;
    }

    writer <<"Mat" <<"{"
           <<"perspective_K"<< merged_perspective_k
           <<"H"<< merged_h
           <<"H2"<< merged_h2
           <<"}";
    writer.release();

    std::string mat_temp_content;
    if (!ReadAllText(mat_temp_path, mat_temp_content)) {
        std::cerr << "Failed to read temp mat yaml: " << mat_temp_path << std::endl;
        std::remove(mat_temp_path.c_str());
        return false;
    }
    std::remove(mat_temp_path.c_str());

    const size_t mat_pos_in_temp = FindTopLevelKey(mat_temp_content, "Mat");
    if (mat_pos_in_temp == std::string::npos) {
        std::cerr << "Temp mat yaml missing Mat node." << std::endl;
        return false;
    }

    const std::string mat_section = mat_temp_content.substr(mat_pos_in_temp);

    std::string original_content;
    if (!ReadAllText(file_path, original_content)) {
        original_content = "%YAML:1.0\n---\n";
    }

    const size_t old_mat_start = FindTopLevelKey(original_content, "Mat");
    std::string merged_content;
    if (old_mat_start == std::string::npos) {
        merged_content = original_content;
        if (!merged_content.empty() && merged_content.back() != '\n') {
            merged_content.push_back('\n');
        }
        merged_content += mat_section;
    } else {
        size_t mat_line_end = original_content.find('\n', old_mat_start);
        if (mat_line_end == std::string::npos) {
            mat_line_end = original_content.size();
        } else {
            mat_line_end += 1;
        }
        const size_t old_mat_end = FindNextTopLevelKey(original_content, mat_line_end);
        merged_content = original_content.substr(0, old_mat_start);
        merged_content += mat_section;
        if (old_mat_end != std::string::npos) {
            merged_content += original_content.substr(old_mat_end);
        }
    }

    const std::string temp_path = file_path + ".tmp";
    if (!WriteAllText(temp_path, merged_content)) {
        std::cerr << "Failed to write merged yaml: " << temp_path << std::endl;
        return false;
    }

    if (std::rename(temp_path.c_str(), file_path.c_str()) != 0) {
        std::cerr << "Failed to replace yaml file: " << file_path << std::endl;
        std::remove(temp_path.c_str());
        return false;
    }

    return true;
}
} // namespace

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("calib_two_cam");

    cv::Mat goal_K = (cv::Mat_<double>(3,3) << 8/3.45,0.0,960,
                                                         0.0,8/3.45,720,
                                                         0.0, 0.0, 1.0 );
    cv::Mat org_K = (cv::Mat_<double>(3,3) << 6/3.45,0.0,720.,
                                                         0.0,6/3.45,540.,
                                                         0.0, 0.0, 1.0 );
    cv::Mat img, change_img, stitch_img;
    cv::Mat perspective_K;
    std::string img_path = "resource/img/ct1.png";//
    std::string stitch_img_path = "resource/img/ct2.png";

    // std::string img_path = "/home/thesky/桌面/22.png";//
    // std::string stitch_img_path = "/home/thesky/桌面/11.png";
//    std::string img_path = "/media/plusseven/EA61-A1AF/datasets/record/I_Hiter/2024-05-23_17_08_12_n=1/5000.jpg";
//    std::string stitch_img_path = "/media/plusseven/EA61-A1AF/datasets/record/I_Hiter/2024-05-23_17_08_11_n=1/5000.jpg";

    change_img = cv::imread(img_path);
    stitch_img = cv::imread(stitch_img_path);

    ImageStitch imageStitch;
    imageStitch.change_F_of_image(org_K, goal_K, stitch_img, stitch_img, perspective_K);

//    imageStitch.Stitching(change_img,stitch_img);
    std::vector<cv::Point2f> keypoints1, keypoints2;
    imageStitch.getKeypoints(stitch_img, change_img,keypoints1, keypoints2);
    imageStitch.Stitching(stitch_img, change_img,keypoints1, keypoints2);

    cv::Mat H = findHomography(keypoints1, keypoints2, cv::RANSAC);
    cv::Mat H2 = findHomography(keypoints2, keypoints1, cv::RANSAC);


    if (!UpdateMatNode(STATIC_CONFIC_PATH, perspective_K, H, H2)) {
        return 1;
    }

    if (!UpdateMatNode(SHARED_CONFIC_PATH, perspective_K, H, H2)) {
        return 1;
    }

    cv::namedWindow("change", cv::WINDOW_NORMAL);
    cv::namedWindow("org", cv::WINDOW_NORMAL);
    cv::namedWindow("stitch", cv::WINDOW_NORMAL);
    cv::imshow("change", change_img);
    // cv::imshow("org", img);
    cv::imshow("stitch", stitch_img);
    cv::waitKey(0);

    rclcpp::shutdown();
    return 0;
}