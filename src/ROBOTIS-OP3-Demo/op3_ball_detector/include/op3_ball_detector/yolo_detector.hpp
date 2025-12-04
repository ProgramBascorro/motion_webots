#ifndef OP3_BALL_DETECTOR__YOLO_DETECTOR_HPP_
#define OP3_BALL_DETECTOR__YOLO_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <chrono>

namespace robotis_op
{

struct Detection
{
  int class_id;
  float confidence;
  cv::Rect box;
  cv::Point2f center;
};

class YoloDetector
{
public:
  YoloDetector(
    const std::string& model_path,
    float conf_threshold,
    float nms_threshold,
    int input_width,
    int input_height,
    bool use_gpu
  );
  
  ~YoloDetector();
  
  bool initialize();
  std::vector<Detection> detect(const cv::Mat& image);
  
  // FPS tracking
  double getFPS() const { return fps_; }
  double getInferenceTime() const { return inference_time_ms_; }

private:
  std::vector<Detection> postProcess(
    const cv::Mat& image,
    const std::vector<cv::Mat>& outputs
  );
  
  void calculateFPS();
  
  std::string model_path_;
  float conf_threshold_;
  float nms_threshold_;
  int input_width_;
  int input_height_;
  bool use_gpu_;
  
  cv::dnn::Net net_;
  std::vector<std::string> output_names_;
  bool initialized_;
  
  // FPS tracking
  int frame_count_;
  std::chrono::steady_clock::time_point start_time_;
  double fps_;
  double inference_time_ms_;
  static constexpr int FPS_UPDATE_INTERVAL = 10;
};

}  // namespace robotis_op

#endif  // OP3_BALL_DETECTOR__YOLO_DETECTOR_HPP_