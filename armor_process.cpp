#include "armor_process.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

//让得到的灯条套上一个旋转矩形，以方便之后对角度这个特殊因素作为匹配标准
LightDescriptor::LightDescriptor(const cv::RotatedRect& light)
{
    width = light.size.width;
    length = light.size.height;
    center = light.center;
    angle = light.angle;
    area = light.size.area();
}

namespace {

// 取灯条两个“端边”的中点（即长边两端），并按图像 y 坐标分成上、下。
// 不依赖 RotatedRect::points() 的起始顶点，也不依赖 width/height 谁大：
// 四个边中点里，距离更大的那一对才是两个端边中点，另一对是两条长边的中点。
void bar_end_points(const cv::Point2f v[4],
                    cv::Point2f& top, cv::Point2f& bottom)
{
    const cv::Point2f m01 = (v[0] + v[1]) * 0.5f;
    const cv::Point2f m23 = (v[2] + v[3]) * 0.5f;
    const cv::Point2f m12 = (v[1] + v[2]) * 0.5f;
    const cv::Point2f m30 = (v[3] + v[0]) * 0.5f;

    cv::Point2f e0, e1;
    if (cv::norm(m01 - m23) >= cv::norm(m12 - m30)) {
        e0 = m01;
        e1 = m23;
    } else {
        e0 = m12;
        e1 = m30;
    }

    if (e0.y <= e1.y) { top = e0; bottom = e1; }
    else              { top = e1; bottom = e0; }
}

}  // namespace

vector<Point2f> detect_armors(cv::Mat& frame)
{
    Mat channels[3], binary, Gaussian, dilatee;
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Point2f> cornerPts;  // 每 4 个一组：TL, TR, BR, BL
    vector<RotatedRect> bars;

    split(frame, channels); //通道分离
    threshold(channels[0], binary, 220, 255, 0);//二值化
    GaussianBlur(binary, Gaussian, Size(5, 5), 0);//滤波
    dilate(Gaussian, dilatee, element);
    findContours(dilatee, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);//轮廓检测

    vector<LightDescriptor> lightInfos;//创建一个灯条类的动态数组
    //筛选灯条
    for (size_t i = 0; i < contours.size(); ++i) {
        // 求轮廓面积
        const double area = contourArea(contours[i]);
        // 去除较小轮廓
        if (area < 5 || contours[i].size() <= 1)
            continue;

        const RotatedRect lightRec = minAreaRect(contours[i]);
        const float longSide = max(lightRec.size.width, lightRec.size.height);
        const float shortSide = min(lightRec.size.width, lightRec.size.height);
        if (shortSide <= 0.0f)
            continue;

        // 长宽比限制（灯条远近都会影响绝对尺寸，所以只看比例）
        const float aspect = longSide / shortSide;
        if (aspect > 20.0f || aspect < 3.0f)
            continue;

        lightInfos.push_back(LightDescriptor(lightRec));
        bars.push_back(lightRec);
    }

    //二重循环多条件匹配灯条
    for (size_t i = 0; i < lightInfos.size(); ++i) {
        for (size_t j = i + 1; j < lightInfos.size(); ++j) {
            const LightDescriptor& a = lightInfos[i];
            const LightDescriptor& b = lightInfos[j];

            const float dis = static_cast<float>(cv::norm(a.center - b.center));
            const float scale = max(max(a.length, a.width), max(b.length, b.width));
            if (scale <= 0.0f)
                continue;

            //灯条长度会因为远近而受到影响，所以按比值匹配灯条
            const float ratio = dis / scale;
            if (ratio > 5.0f || ratio < 1.0f)
                continue;

            cv::Point2f va[4], vb[4];
            bars[i].points(va);
            bars[j].points(vb);

            cv::Point2f aTop, aBottom, bTop, bBottom;
            bar_end_points(va, aTop, aBottom);
            bar_end_points(vb, bTop, bBottom);

            // i/j 只是轮廓下标，不保证谁在图像左边；按 x 重新分左右，
            // 保证输出顺序固定为 TL, TR, BR, BL。
            if (aTop.x > bTop.x) {
// （TL, TR, BR, BL）；多块装甲板时按 4 个一组依次排列，未识别到时返回空。
                std::swap(aTop, bTop);
                std::swap(aBottom, bBottom);
            }

            const vector<Point2f> quad = {aTop, bTop, bBottom, aBottom};
            cornerPts.insert(cornerPts.end(), quad.begin(), quad.end());

            // polylines 的 InputArray 版本只接受整型点，这里转换一下
            vector<Point> quad_i;
            quad_i.reserve(quad.size());
            for (const Point2f& p : quad)
                quad_i.emplace_back(cvRound(p.x), cvRound(p.y));
            polylines(frame, quad_i, true, Scalar(0, 255, 0), 2);
        }
    }

    return cornerPts;
}
