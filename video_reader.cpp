#include "video_reader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>

VideoReader::VideoReader(const std::string& source, bool loop)
{
    open(source, loop);
}

VideoReader::~VideoReader()
{
    close();
}

bool VideoReader::isCameraIndex(const std::string& source)
{
    return !source.empty() &&
           std::all_of(source.begin(), source.end(),
                       [](unsigned char c) { return std::isdigit(c) != 0; });
}

bool VideoReader::open(const std::string& source, bool loop)
{
    close();
    source_ = source;
    loop_ = loop;

    bool ok = false;
    if (isCameraIndex(source_)) {
        ok = cap_.open(std::stoi(source_), cv::CAP_ANY);
    } else {
        ok = cap_.open(source_, cv::CAP_ANY);
    }

    if (!ok) {
        std::cerr << "[VideoReader] 无法打开视频源: " << source_ << std::endl;
        return false;
    }

    // 文件源通常有帧数，摄像头则拿不到（或为 0）
    seekable_ = frameCount() > 0;
    return true;
}

void VideoReader::close()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
    seekable_ = false;
}

bool VideoReader::read(cv::Mat& frame)
{
    if (!cap_.isOpened()) {
        return false;
    }

    // 用 read() 而不是 cap_ >> frame，能明确区分“读到结尾/出错”
    if (!cap_.read(frame) || frame.empty()) {
        if (!loop_) {
            return false;
        }
        rewind();
        if (!cap_.read(frame) || frame.empty()) {
            return false;
        }
    }
    return true;
}

double VideoReader::timestampMs() const
{
    return cap_.get(cv::CAP_PROP_POS_MSEC);
}

void VideoReader::rewind()
{
    if (!cap_.isOpened() || !seekable_) {
        return;
    }
    cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
}

double VideoReader::fps() const
{
    const double f = cap_.get(cv::CAP_PROP_FPS);
    return (std::isfinite(f) && f > 0.0) ? f : 30.0;
}

int VideoReader::width() const
{
    return static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
}

int VideoReader::height() const
{
    return static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
}

int VideoReader::frameCount() const
{
    if (!cap_.isOpened()) {
        return 0;
    }
    const double n = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    return (std::isfinite(n) && n > 0.0) ? static_cast<int>(n) : 0;
}
