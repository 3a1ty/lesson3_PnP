#pragma once

#include <vector>

#include <opencv2/core.hpp>

// 灯条：识别与配对的核心对象。
// 用旋转矩形描述，长宽、偏转角、面积、中心点都是后续灯条配对的依据。
class LightDescriptor {
public:
    float width = 0.0f;
    float length = 0.0f;
    float angle = 0.0f;
    float area = 0.0f;
    cv::Point2f center{0.0f, 0.0f};

    LightDescriptor() = default;
    explicit LightDescriptor(const cv::RotatedRect& light);
};

// 识别一帧 BGR 图像（CV_8UC3）：二值化 -> 轮廓 -> 灯条筛选 -> 两两配对，
// 并在 frame 上绘制匹配到的装甲板框。
//
// 返回每块装甲板的 4 个角点，顺序固定为 左上、右上、右下、左下
// （TL, TR, BR, BL）；多块装甲板时按 4 个一组依次排列，未识别到时返回空。
// 函数内部不创建窗口，显示由调用方负责。
std::vector<cv::Point2f> detect_armors(cv::Mat& frame);
