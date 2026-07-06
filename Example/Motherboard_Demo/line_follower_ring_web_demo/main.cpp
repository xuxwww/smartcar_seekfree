// line_follower_ring_web_demo
//
// 目标：
//   提供一个类似 OpenCV 的 `imshow(窗口名, 图像)` 接口，
//   让网页端能够根据不同的"窗口名"自动创建并刷新不同的独立窗口。
//
// 实现已迁移至 find_line_lib 库 (web_imshow.h / web_imshow.cpp)，
// 本文件仅负责摄像头采集 + 调用库内算法 + 调用库内 imshow。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "find_line_lib/web_imshow.h"
#include "zf_driver_encoder.h"

extern "C" {
#include "image.h"
int Speed_Goal;
}

inline std::atomic<bool> g_running{false};

namespace {
constexpr const char *kEncoderLeft = "/dev/zf_encoder_1";
constexpr const char *kEncoderRight = "/dev/zf_encoder_2";
std::chrono::steady_clock::time_point g_last_speed_time =
    std::chrono::steady_clock::now();
int g_last_left_count = 0;
int g_last_right_count = 0;
bool g_speed_feedback_ready = false;
} // namespace

namespace find_line_lib {
WebImShow server("8089");
} // namespace find_line_lib

static std::pair<float, float> PredictTargetWheelSpeeds(float base_speed) {
  int tow_row = ImageStatus.TowPoint_True;
  if (tow_row < ImageStatus.OFFLine || tow_row >= LCDH) {
    tow_row = ImageStatus.TowPoint;
  }
  if (tow_row < ImageStatus.OFFLine)
    tow_row = ImageStatus.OFFLine;
  if (tow_row >= LCDH)
    tow_row = LCDH - 1;

  float center_error = static_cast<float>(ImageDeal[tow_row].Center) -
                       static_cast<float>(ImageStatus.MiddleLine);
  float det_error = static_cast<float>(ImageStatus.Det_True) -
                    static_cast<float>(ImageStatus.MiddleLine);
  float error = 0.5f * center_error + 0.5f * det_error;

  const float turn_gain = 0.45f;
  float left_speed = base_speed + error * turn_gain;
  float right_speed = base_speed - error * turn_gain;

  left_speed = std::clamp(left_speed, 10.0f, 120.0f);
  right_speed = std::clamp(right_speed, 10.0f, 120.0f);
  return {left_speed, right_speed};
}

static void Data_Settings(void) {
  ImageStatus.MiddleLine = 43;
  ImageStatus.TowPoint_Gain = 0.2;
  ImageStatus.TowPoint_Offset_Max = 5;
  ImageStatus.TowPoint_Offset_Min = -2;
  ImageStatus.TowPointAdjust_v = 160;
  ImageStatus.Det_all_k = 0.7;
  ImageStatus.CirquePass = 'F';
  ImageStatus.IsCinqueOutIn = 'F';
  ImageStatus.CirqueOut = 'F';
  ImageStatus.CirqueOff = 'F';
  ImageStatus.Barn_Flag = 0;
  ImageStatus.straight_acc = 0;
  ImageStatus.TowPoint = 23;
  ImageStatus.Threshold_static = 70;
  ImageStatus.Threshold_detach = 180;
  ImageScanInterval = 2;
  ImageScanInterval_Cross = 5;
  ImageStatus.variance_acc = 25;
  SystemData.Stop = 0;
  Var_speed_start = 1; // 启用速度控制
}

static void UpdateSpeedFeedback(void) {
  const auto now = std::chrono::steady_clock::now();
  const int left = encoder_get_count(kEncoderLeft);
  const int right = encoder_get_count(kEncoderRight);

  if (!g_speed_feedback_ready) {
    g_last_left_count = left;
    g_last_right_count = right;
    g_last_speed_time = now;
    g_speed_feedback_ready = true;
    return;
  }

  const auto dt_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_speed_time)
          .count();
  if (dt_ms > 0) {
    const float delta_left = std::abs(left - g_last_left_count);
    const float delta_right = std::abs(right - g_last_right_count);
    const float avg_delta = (delta_left + delta_right) * 0.5f;
    const float speed = avg_delta / static_cast<float>(dt_ms) * 1000.0f;
    SystemData.SpeedData.nowspeed = speed;
  }

  g_last_left_count = left;
  g_last_right_count = right;
  g_last_speed_time = now;

  const int goal = (Speed_Goal > 0) ? Speed_Goal : 85;
  SystemData.SpeedData.expectspeed = goal;
  SystemData.SpeedData.expect_True_speed = static_cast<float>(goal);
  SystemData.SpeedData.motor_duty = goal;
}

static const char *RoadTypeStr(RoadType_e t) {
  switch (t) {
  case Normol:
    return "Normal";
  case Straight:
    return "Straight";
  case Cross:
    return "Cross";
  case Ramp:
    return "Ramp";
  case LeftCirque:
    return "LeftCircle";
  case RightCirque:
    return "RightCircle";
  case Forkin:
    return "ForkIn";
  case Forkout:
    return "ForkOut";
  case Barn_out:
    return "BarnOut";
  case Barn_in:
    return "BarnIn";
  case Cross_ture:
    return "CrossTrue";
  default:
    return "Unknown";
  }
}

int main() {
    std::cout << "[INFO] 正在尝试通过 V4L2 后端打开摄像头 0..." << std::endl;
    
    // 1. 显式指定 CAP_V4L2 增强在 Linux/Buildroot 上的兼容性
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[ERROR] 错误：无法打开摄像头！请检查 /dev/video0 是否存在或权限是否正确。" << std::endl;
        return 1;
    }

    // 2. 尝试设置低分辨率以节省嵌入式 CPU
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    
    double actual_w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    std::cout << "[INFO] 摄像头初始化成功。当前硬件输出分辨率: " << actual_w << "x" << actual_h << std::endl;

    // 3. 启动 Web 服务器（RAII：离开作用域时自动停止）
    g_running.store(true);

    cv::Mat frame_raw, frame, gray;

    std::cout << "[INFO] 工作台已就绪，请在浏览器中打开 http://板子IP:8089" << std::endl;

    while (g_running.load()) {
        if (!cap.read(frame_raw) || frame_raw.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // 4. 安全防御：如果摄像头固件拒绝了 160x120 的请求，手动缩放
        if (frame_raw.cols != 160 || frame_raw.rows != 120) {
            cv::resize(frame_raw, frame, cv::Size(160, 120), 0, 0, cv::INTER_AREA);
        } else {
            frame = frame_raw;
        }

        // ─── 算法基本操作 (image.c) ───
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::resize(gray, gray, cv::Size(LCDW, LCDH), 0, 0, cv::INTER_AREA);
        for (int y = 0; y < LCDH; y++)
          for (int x = 0; x < LCDW; x++)
            Image_Use[y][x] = gray.at<uint8_t>(y, x);
        Data_Settings();
        ImageProcess();
        UpdateSpeedFeedback();

        const int goal = (Speed_Goal > 0) ? Speed_Goal : 85;
        auto [pred_left_speed, pred_right_speed] =
            PredictTargetWheelSpeeds(static_cast<float>(goal));

        // ---- 调试图：灰度图 + 边界/中线 ----
        const int dispScale = 4; // 80x60 -> 320x240
        cv::Mat debugImg;
        cv::resize(gray, debugImg, cv::Size(LCDW * dispScale, LCDH * dispScale),
                   0, 0, cv::INTER_NEAREST);
        cv::cvtColor(debugImg, debugImg, cv::COLOR_GRAY2BGR);

        for (int y = ImageStatus.OFFLine; y < LCDH; y++) {
          int lx = ImageDeal[y].LeftBorder;
          int rx = ImageDeal[y].RightBorder;
          int cx = ImageDeal[y].Center;
          cv::circle(debugImg, cv::Point(lx * dispScale, y * dispScale), 2,
                     cv::Scalar(0, 255, 0), -1); // 左边界 - 绿
          cv::circle(debugImg, cv::Point(rx * dispScale, y * dispScale), 2,
                     cv::Scalar(0, 0, 255), -1); // 右边界 - 红
          cv::circle(debugImg, cv::Point(cx * dispScale, y * dispScale), 2,
                     cv::Scalar(255, 0, 0), -1); // 中线 - 蓝
        }

        // 前瞻行 - 黄色水平线
        int tp = ImageStatus.TowPoint;
        if (tp >= 0 && tp < LCDH) {
          cv::line(debugImg, cv::Point(0, tp * dispScale),
                   cv::Point(LCDW * dispScale - 1, tp * dispScale),
                   cv::Scalar(0, 255, 255), 1);
        }

        // ---- 二值化图 ----
        const int left = encoder_get_count(kEncoderLeft);
        const int right = encoder_get_count(kEncoderRight);

        std::string road_name = RoadTypeStr(ImageStatus.Road_type);
        float det_error = static_cast<float>(ImageStatus.Det_True) -
                          static_cast<float>(ImageStatus.MiddleLine);

        // ---- 二值化图 ----
        cv::Mat pixleImg(LCDH, LCDW, CV_8UC1);
        for (int y = 0; y < LCDH; y++)
          for (int x = 0; x < LCDW; x++)
            pixleImg.at<uint8_t>(y, x) = Pixle[y][x] ? 255 : 0;
        cv::Mat bin_frame;
        cv::resize(pixleImg, bin_frame,
                   cv::Size(LCDW * dispScale, LCDH * dispScale), 0, 0,
                   cv::INTER_NEAREST);
        cv::cvtColor(bin_frame, bin_frame, cv::COLOR_GRAY2BGR);

        // 只要你更换窗口名，网页上就会自动多弹出一个独立的窗口！
        find_line_lib::server.imshow("1. Debug", debugImg);
        find_line_lib::server.imshow("2. Binary", bin_frame);

        int road_type_num = static_cast<int>(ImageStatus.Road_type);
        cv::Mat speedPanel(300, 400, CV_8UC3, cv::Scalar(24, 24, 24));
        cv::putText(speedPanel, "Predicted Speed & Image Data", cv::Point(16, 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(240, 240, 240),
                    2, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("RoadType: %d %s", road_type_num, road_name.c_str()),
                    cv::Point(16, 58), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("Thresh: %d  Det: %d  Err: %.1f",
                               ImageStatus.Threshold, ImageStatus.Det_True,
                               det_error),
                    cv::Point(16, 86), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("TowTrue: %d  Tow: %d OFF: %d", ImageStatus.TowPoint_True,
                               ImageStatus.TowPoint, ImageStatus.OFFLine),
                    cv::Point(16, 114), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("LeftLine: %d  RightLine: %d Mid: %d", ImageStatus.Left_Line,
                               ImageStatus.Right_Line, ImageStatus.MiddleLine),
                    cv::Point(16, 142), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("LCount: %d  RCount: %d", left, right),
                    cv::Point(16, 170), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 180), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("CurSpeed: %.1f  GoalSpeed: %.1f", SystemData.SpeedData.nowspeed,
                               SystemData.SpeedData.expect_True_speed),
                    cv::Point(16, 198), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 180), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("Goal: %d  Duty: %d", goal,
                               SystemData.SpeedData.motor_duty),
                    cv::Point(16, 226), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 180), 1, cv::LINE_AA);
        cv::putText(speedPanel,
                    cv::format("Pred L: %.1f  Pred R: %.1f", pred_left_speed,
                               pred_right_speed),
                    cv::Point(16, 254), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
        find_line_lib::server.imshow("3. Speed", speedPanel);
    }

    g_running.store(false);
    cap.release();
    // server 离开作用域时自动调用析构停止服务器
    return 0;
}