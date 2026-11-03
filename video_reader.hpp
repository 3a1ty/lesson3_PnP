#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

// 视频读取模块：对 cv::VideoCapture 做一层薄封装，
// 统一处理“打开失败、读到结尾、循环播放、按帧取时间戳”这些琐事。
class VideoReader {
public:
    VideoReader() = default;
    // source 可以是视频文件路径（如 "raw.mp4"），也可以是摄像头序号（如 "0"）
    explicit VideoReader(const std::string& source, bool loop = false);
    ~VideoReader();

    // VideoCapture 不可拷贝，这里也禁止拷贝，避免重复 release
    VideoReader(const VideoReader&) = delete;
    VideoReader& operator=(const VideoReader&) = delete;

    // 打开视频源；返回 false 表示打不开（文件不存在 / 编解码器缺失 / 摄像头被占用）
    bool open(const std::string& source, bool loop = false);
    bool isOpened() const { return cap_.isOpened(); }
    void close();

    // 读一帧到 frame；读不到（结尾或出错）返回 false。
    // loop_ 为 true 时，读到结尾会自动回到第一帧继续读。
    bool read(cv::Mat& frame);
    // 当前帧在视频中的时间戳（毫秒），摄像头 / 实时流下可能一直是 0
    double timestampMs() const;
    // 回到第 0 帧（只对可定位的文件源有意义）
    void rewind();

    // 元信息；拿不到时给出安全默认值（fps 默认 30）
    double fps() const;
    int width() const;
    int height() const;
    int frameCount() const;
    bool seekable() const { return seekable_; }

private:
    // 纯数字字符串按摄像头索引处理，其余按文件路径处理
    static bool isCameraIndex(const std::string& source);

    cv::VideoCapture cap_;
    std::string source_;
    bool loop_ = false;
    bool seekable_ = false;  // 文件源才能 seek，摄像头不行
};
