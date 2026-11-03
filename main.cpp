#include <cstdio>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_process.hpp"
#include "pnp.hpp"
#include "video_reader.hpp"

using namespace std;
using namespace cv;

int main()
{
    VideoReader reader("raw.mp4", false);
    if (!reader.isOpened()) {
        std::cerr << "视频打开失败！" << std::endl;
        return -1;
    }

    PnPSolver solver;
    CameraPose pose;
    cv::Mat frame;

    while (reader.read(frame)) {
        const vector<Point2f> corners = detect_armors(frame);

        if (corners.size() >= 4) {
            const vector<Point2f> quad(corners.begin(), corners.begin() + 4);
            solver.solve(quad, pose); 
        } else {
            pose.valid = false;
        }

        if (pose.valid) {
            char line1[128], line2[128];
            std::snprintf(line1, sizeof(line1), "x=%.0f y=%.0f z=%.0f mm",
                          pose.position[0], pose.position[1], pose.position[2]);
            std::snprintf(line2, sizeof(line2), "yaw=%.1f pitch=%.1f roll=%.1f",
                          pose.yaw, pose.pitch, pose.roll);

            const int font = FONT_HERSHEY_SIMPLEX;
            const double scale = 0.7;
            const int thickness = 2;
            const int margin = 20;

            int baseline = 0;
            const Size s1 = getTextSize(line1, font, scale, thickness, &baseline);
            const Size s2 = getTextSize(line2, font, scale, thickness, &baseline);

            // 两行取最大宽度
            const int textWidth = max(s1.width, s2.width);
            const int x = max(margin, frame.cols - textWidth - margin);
            const int y1 = margin + s1.height;  
            const int y2 = y1 + s2.height + 10;

            putText(frame, line1, Point(x, y1), font, scale,
                    Scalar(0, 255, 255), thickness);
            putText(frame, line2, Point(x, y2), font, scale,
                    Scalar(0, 255, 255), thickness);
        }

        imshow("video", frame);
        if (waitKey(33) == 27)
            break;
    }
    return 0;
}
