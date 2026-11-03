#pragma once

#include <vector>

#include <opencv2/core.hpp>

// 相机在世界（装甲板）坐标系下的位姿。
//
// 坐标约定：
//   原点在装甲板中心；x 向右（左灯条 -> 右灯条）；y 向下；z 指向装甲板内部。
//   因此相机在装甲板前方时 position.z < 0，相机到装甲板的距离 = |position|。
struct CameraPose {
    cv::Vec3d position = cv::Vec3d(0, 0, 0); // 相机中心，mm
    double yaw = 0.0;                        // 绕 y 轴，deg
    double pitch = 0.0;                      // 绕 x 轴，deg
    double roll = 0.0;                       // 绕 z 轴，deg
    double reprojError = 0.0;                // 重投影 RMSE，px
    bool valid = false;
};

class PnPSolver {
public:
    // armorWidth / armorHeight 是四个角点所围矩形的真实尺寸，单位 mm。
    // 注意要和 detect_armors 给出的点定义一致：现在的四点是灯条端边中点，
    // 横向尺寸应该是灯条中心距，而不是装甲板外宽（除非你改了检测）。
    explicit PnPSolver(double armorWidth = 135.0,
                       double armorHeight = 56.0,
                       double maxReprojError = 5.0);

    // imagePoints 必须是 4 个点，顺序 TL, TR, BR, BL（与 detect_armors 一致）。
    //
    // 内部用 IPPE 解出两个候选位姿；平面目标在近仿射时会二义，所以：
    //   - 第一帧没有历史，取重投影误差较小的解；
    //   - 后续帧取与上一帧位置/姿态最连续的解（误差相近时才参与比较）。
    // 解算失败或重投影误差超过 maxReprojError 时返回 false，pose.valid 为 false。
    bool solve(const std::vector<cv::Point2f>& imagePoints, CameraPose& pose);

    // 目标丢失、重新开始一段视频时调用，清空上一帧状态。
    void reset();

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    std::vector<cv::Point3f> objectPoints_;
    double maxReprojError_ = 5.0;

    bool hasLast_ = false;
    CameraPose lastPose_;
    cv::Mat lastR_wc_;
};
