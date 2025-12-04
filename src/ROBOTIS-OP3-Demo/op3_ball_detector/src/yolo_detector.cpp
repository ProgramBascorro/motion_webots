#include "op3_ball_detector/yolo_detector.hpp"
#include <iostream>

namespace robotis_op
{

YoloDetector::YoloDetector(
  const std::string& model_path,
  float conf_threshold,
  float nms_threshold,
  int input_width,
  int input_height,
  bool use_gpu)
: model_path_(model_path),
  conf_threshold_(conf_threshold),
  nms_threshold_(nms_threshold),
  input_width_(input_width),
  input_height_(input_height),
  use_gpu_(use_gpu),
  initialized_(false),
  frame_count_(0),
  fps_(0.0),
  inference_time_ms_(0.0)
{
  start_time_ = std::chrono::steady_clock::now();
}

YoloDetector::~YoloDetector()
{
}

bool YoloDetector::initialize()
{
  try
  {
    std::cout << "Loading YOLO ONNX model from: " << model_path_ << std::endl;
    net_ = cv::dnn::readNetFromONNX(model_path_);
    
    if (net_.empty())
    {
      std::cerr << "Failed to load ONNX model" << std::endl;
      return false;
    }
    
    if (use_gpu_)
    {
      std::cout << "Attempting to use CUDA backend" << std::endl;
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else
    {
      std::cout << "Using CPU backend" << std::endl;
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    
    output_names_ = net_.getUnconnectedOutLayersNames();
    
    std::cout << "YOLO model loaded successfully" << std::endl;
    std::cout << "  OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "  Output layers: " << output_names_.size() << std::endl;
    
    initialized_ = true;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    return false;
  }
}

std::vector<Detection> YoloDetector::detect(const cv::Mat& image)
{
  std::vector<Detection> detections;
  
  if (!initialized_ || image.empty())
    return detections;
  
  try
  {
    auto start = std::chrono::high_resolution_clock::now();
    
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob, 1.0/255.0, 
                          cv::Size(input_width_, input_height_), 
                          cv::Scalar(), true, false);
    
    net_.setInput(blob);
    
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, output_names_);
    
    auto end = std::chrono::high_resolution_clock::now();
    inference_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
    
    detections = postProcess(image, outputs);
    calculateFPS();
  }
  catch (const cv::Exception& e)
  {
    std::cerr << "OpenCV Detection error: " << e.what() << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Detection error: " << e.what() << std::endl;
  }
  
  return detections;
}

void YoloDetector::calculateFPS()
{
  frame_count_++;
  
  if (frame_count_ % FPS_UPDATE_INTERVAL == 0)
  {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(current_time - start_time_).count();
    
    if (elapsed > 0)
      fps_ = FPS_UPDATE_INTERVAL / elapsed;
    
    start_time_ = current_time;
    frame_count_ = 0;
  }
}

std::vector<Detection> YoloDetector::postProcess(
  const cv::Mat& image,
  const std::vector<cv::Mat>& outputs)
{
  std::vector<Detection> detections;
  
  if (outputs.empty())
  {
    // std::cerr << "No outputs from network" << std::endl;
    return detections;
  }
  
  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  
  // Get first output
  cv::Mat output = outputs[0];
  
  // HAPUS SEMUA DEBUG LOG
  // std::cout << "Output total elements: " << output.total() << std::endl;
  // std::cout << "Output type: " << output.type() << std::endl;
  
  // Handle YOLOv8 output format: [1, num_classes+4, 8400]
  // Need to transpose to [8400, num_classes+4]
  
  cv::Mat detections_mat;
  
  if (output.dims == 3 && output.size[0] == 1)
  {
    // Shape: [1, num_features, num_detections]
    int num_features = output.size[1];
    int num_detections = output.size[2];
    
    // std::cout << "Detected YOLOv8 format: [1, " << num_features 
    //           << ", " << num_detections << "]" << std::endl;
    
    // Reshape to [num_features, num_detections]
    cv::Mat reshaped(num_features, num_detections, CV_32F, output.data);
    
    // Transpose to [num_detections, num_features]
    cv::transpose(reshaped, detections_mat);
    
    // std::cout << "Transposed to: [" << detections_mat.rows 
    //           << ", " << detections_mat.cols << "]" << std::endl;
  }
  else if (output.dims == 2)
  {
    // Already in correct format
    detections_mat = output.clone();
  }
  else
  {
    // std::cerr << "Unsupported output format" << std::endl;
    return detections;
  }
  
  // Now parse detections
  float x_factor = static_cast<float>(image.cols) / input_width_;
  float y_factor = static_cast<float>(image.rows) / input_height_;
  
  int num_detections = detections_mat.rows;
  int num_features = detections_mat.cols;
  
  // std::cout << "Processing " << num_detections << " detections..." << std::endl;
  
  for (int i = 0; i < num_detections; ++i)
  {
    // Access row safely
    const float* detection = detections_mat.ptr<float>(i);
    
    // YOLOv8 format: [x_center, y_center, width, height, class_scores...]
    float x_center = detection[0];
    float y_center = detection[1];
    float width = detection[2];
    float height = detection[3];
    
    // Find best class from remaining features
    float max_score = 0.0f;
    int class_id = -1;
    
    for (int j = 4; j < num_features; ++j)
    {
      if (detection[j] > max_score)
      {
        max_score = detection[j];
        class_id = j - 4;
      }
    }
    
    // Filter by confidence threshold
    if (max_score > conf_threshold_)
    {
      // Convert from center format to corner format
      int left = static_cast<int>((x_center - width / 2.0f) * x_factor);
      int top = static_cast<int>((y_center - height / 2.0f) * y_factor);
      int w = static_cast<int>(width * x_factor);
      int h = static_cast<int>(height * y_factor);
      
      // Clamp to image bounds
      left = std::max(0, std::min(left, image.cols - 1));
      top = std::max(0, std::min(top, image.rows - 1));
      w = std::min(w, image.cols - left);
      h = std::min(h, image.rows - top);
      
      if (w > 0 && h > 0)
      {
        class_ids.push_back(class_id);
        confidences.push_back(max_score);
        boxes.push_back(cv::Rect(left, top, w, h));
      }
    }
  }
  
  // std::cout << "Found " << boxes.size() << " boxes before NMS" << std::endl;
  
  // Apply Non-Maximum Suppression
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
  
  // std::cout << "Found " << indices.size() << " boxes after NMS" << std::endl;
  
  // Create Detection objects from NMS results
  for (int idx : indices)
  {
    Detection det;
    det.class_id = class_ids[idx];
    det.confidence = confidences[idx];
    det.box = boxes[idx];
    detections.push_back(det);
  }
  
  return detections;
}

}  // namespace robotis_op