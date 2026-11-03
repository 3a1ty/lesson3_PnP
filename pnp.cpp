#include "pnp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>

namespace {

// R_wc = Ry(yaw) * Rx(pitch) * Rz(roll)
void eulerFromRwc(const cv::Mat& R, double& yaw, double& pitch, double& roll)
{
    const double r02 = R.at<double>(0, 2);
    const double r12 = R.at<double>(1, 2);
    const double r22 = R.at<double>(2, 2);
    const double r10 = R.at<double>(1, 0);
    const double r11 = R.at<double>(1, 1);

    const double sp = std::max(-1.0, std::min(1.0, -r12));
    pitch = std::asin(sp) * 180.0 / CV_PI;
    yaw   = std::atan2(r02, r22) * 180.0 / CV_PI;
    roll  = std::atan2(r10, r11) * 180.0 / CV_PI;
}

// 两个旋转之间的测地角，弧度
double rotationAngle(const cv::Mat& R1, const cv::Mat& R2)
{
    const cv::Mat dR = R1.t() * R2;
    cv::Mat rvec;
    cv::Rodrigues(dR, rvec);
    return cv::norm(rvec);
}

struct Candidate {
    cv::Mat R_wc;   // 相机 -> 世界
    cv::Vec3d C;    // 相机中心，世界坐标
    double err = 0.0;
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
};

}  // namespace

PnPSolver::PnPSolver(double armorWidth, double armorHeight, double maxReprojError)
    : maxReprojError_(maxReprojError)
{
    // 作业给定的相机内参
    cameraMatrix_ = (cv::Mat_<double>(3, 3) <<
        1789.2913422305087, 0.0, 702.93444417420312,
        0.0, 1789.214342198638, 557.65168675325629,
        0.0, 0.0, 1.0);

    distCoeffs_ = (cv::Mat_<double>(1, 5) <<
        -0.075551988063684988, 0.13117079840026666,
        0.0014612753636792362, -0.001318090407814957, 0.0);

    const float w = static_cast<float>(armorWidth) * 0.5f;
    const float h = static_cast<float>(armorHeight) * 0.5f;
    // 与 detect_armors 的 TL, TR, BR, BL 对应；y 向下，上边是 -h
    objectPoints_ = {
        {-w, -h, 0.0f}, { w, -h, 0.0f},
        { w,  h, 0.0f}, {-w,  h, 0.0f}
    };
}

bool PnPSolver::solve(const std::vector<cv::Point2f>& imagePoints, CameraPose& pose)
{
    pose = CameraPose{};
    if (imagePoints.size() != objectPoints_.size())
        return false;

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errs;
    int n = 0;
    try {
        n = cv::solvePnPGeneric(objectPoints_, imagePoints, cameraMatrix_, distCoeffs_,
                                rvecs, tvecs, false, cv::SOLVEPNP_IPPE,
                                cv::noArray(), cv::noArray(), errs);
    } catch (const cv::Exception&) {
        return false;
    }

    if (n <= 0 || static_cast<int>(rvecs.size()) != n ||
        static_cast<int>(tvecs.size()) != n) {
        return false;
    }

    std::vector<Candidate> candidates(n);
    for (int i = 0; i < n; ++i) {
        cv::Mat R;
        cv::Rodrigues(rvecs[i], R);       // 装甲板 -> 相机
        candidates[i].R_wc = R.t();       // 相机 -> 装甲板
        const cv::Mat C = -candidates[i].R_wc * tvecs[i];
        candidates[i].C = cv::Vec3d(C.at<double>(0), C.at<double>(1), C.at<double>(2));
        candidates[i].err = (static_cast<int>(errs.size()) == n) ? errs[i] : 0.0;
        eulerFromRwc(candidates[i].R_wc,
                     candidates[i].yaw, candidates[i].pitch, candidates[i].roll);
    }

    // 先找误差最小的解
    int best = 0;
    for (int i = 1; i < n; ++i) {
        if (candidates[i].err < candidates[best].err)
            best = i;
    }
    const double minErr = candidates[best].err;

    // 有上一帧时，在误差接近的候选里选最连续的那个，消解平面二义性
    if (hasLast_) {
        double bestScore = std::numeric_limits<double>::max();
        for (int i = 0; i < n; ++i) {
            if (candidates[i].err > minErr + 1.0)
                continue;

            const double dPos = cv::norm(candidates[i].C - lastPose_.position);
            const double dAng = rotationAngle(candidates[i].R_wc, lastR_wc_);
            const double score = dPos + 200.0 * dAng;  // mm + mm/rad

            if (score < bestScore) {
                bestScore = score;
                best = i;
            }
        }
    }

    if (candidates[best].err > maxReprojError_)
        return false;

    pose.position = candidates[best].C;
    pose.yaw = candidates[best].yaw;
    pose.pitch = candidates[best].pitch;
    pose.roll = candidates[best].roll;
    pose.reprojError = candidates[best].err;
    pose.valid = true;

    lastPose_ = pose;
    lastR_wc_ = candidates[best].R_wc.clone();
    hasLast_ = true;
    return true;
}

void PnPSolver::reset()
{
    hasLast_ = false;
    lastPose_ = CameraPose{};
    lastR_wc_.release();
}
