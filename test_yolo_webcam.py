#!/usr/bin/env python3
"""
Test YOLO detector with webcam
Runs YOLO detection on webcam feed without ROS2

Usage:
    ./test_yolo_webcam.py                    # Use default webcam and model
    ./test_yolo_webcam.py --camera 1         # Use different camera
    ./test_yolo_webcam.py --no-gpu           # Force CPU
"""

import cv2
import numpy as np
import argparse
import time
from pathlib import Path


class YOLODetector:
    CLASS_NAMES = ['ball', 'goal post', 'robot', 'L-intersection', 'T-intersection', 'X-intersection']
    CLASS_COLORS = [
        (0, 255, 255),    # ball - yellow
        (0, 255, 0),      # goal post - green
        (255, 0, 0),      # robot - blue
        (255, 255, 0),    # L-intersection - cyan
        (255, 0, 255),    # T-intersection - magenta
        (0, 255, 255),    # X-intersection - yellow
    ]

    def __init__(self, model_path, use_gpu=False, input_size=640):
        self.input_size = input_size
        self.net = cv2.dnn.readNetFromONNX(model_path)

        # Configure backend
        if use_gpu:
            print("Attempting to use GPU...")
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                print("✓ GPU enabled (CUDA)")
            except:
                print("✗ GPU not available, falling back to CPU")
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        else:
            print("Using CPU")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.class_thresholds = [0.2, 0.2, 0.2, 0.4, 0.4, 0.4]
        self.nms_threshold = 0.5
        self.nms_score_threshold = 0.1

    def preprocess(self, image):
        """Preprocess image for YOLO."""
        height, width = image.shape[:2]
        max_dim = max(width, height)

        if width == height:
            processed = cv2.resize(image, (self.input_size, self.input_size))
        else:
            square = np.full((max_dim, max_dim, 3), 114, dtype=np.uint8)
            square[0:height, 0:width] = image
            processed = cv2.resize(square, (self.input_size, self.input_size))

        scale = max_dim / self.input_size
        return processed, scale

    def detect(self, image):
        """Run detection on image."""
        height, width = image.shape[:2]

        # Preprocess
        processed, scale = self.preprocess(image)

        # Create blob and run inference
        blob = cv2.dnn.blobFromImage(
            processed, 1.0 / 255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False
        )

        self.net.setInput(blob)
        start = time.perf_counter()
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        inference_time = (time.perf_counter() - start) * 1000

        if not outputs:
            return [], inference_time

        # Decode detections
        detections = self.decode_detections(outputs[0], scale, width, height)

        if not detections:
            return [], inference_time

        # NMS
        boxes = [d['box'] for d in detections]
        scores = [d['score'] for d in detections]
        indices = cv2.dnn.NMSBoxes(
            boxes, scores, self.nms_score_threshold, self.nms_threshold
        )

        final_detections = [detections[i] for i in indices]
        return final_detections, inference_time

    def decode_detections(self, output, scale, image_width, image_height):
        """Decode YOLO output."""
        detections = []

        # Handle different output shapes
        if len(output.shape) == 3:
            dim1, dim2 = output.shape[1], output.shape[2]
            if dim1 in [10, 11]:
                output = output[0].T
            elif dim2 in [10, 11]:
                output = output[0]

        num_classes = len(self.CLASS_NAMES)

        for row in output:
            cx, cy, w, h = row[0:4]

            has_objectness = len(row) >= (5 + num_classes)
            class_offset = 5 if has_objectness else 4
            objectness = row[4] if has_objectness else 1.0

            class_scores = row[class_offset:class_offset + num_classes] * objectness
            best_class = int(np.argmax(class_scores))
            best_score = float(class_scores[best_class])

            if best_class < 0 or best_class >= num_classes:
                continue

            if best_score < self.class_thresholds[best_class]:
                continue

            # Convert to pixel coordinates
            left = int((cx - 0.5 * w) * scale)
            top = int((cy - 0.5 * h) * scale)
            width = int(w * scale)
            height = int(h * scale)

            # Clamp to image bounds
            left = max(0, min(left, image_width - 1))
            top = max(0, min(top, image_height - 1))
            width = max(1, min(width, image_width - left))
            height = max(1, min(height, image_height - top))

            detections.append({
                'box': [left, top, width, height],
                'class_id': best_class,
                'score': best_score
            })

        return detections

    def draw_detections(self, image, detections, inference_time):
        """Draw bounding boxes on image."""
        output = image.copy()

        for det in detections:
            x, y, w, h = det['box']
            class_id = det['class_id']
            score = det['score']

            color = self.CLASS_COLORS[class_id]
            class_name = self.CLASS_NAMES[class_id]

            # Draw box
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)

            # Draw label
            label = f'{class_name} {score:.2f}'
            cv2.putText(output, label, (x, max(0, y - 5)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Draw stats
        fps = 1000.0 / inference_time if inference_time > 0 else 0
        stats = f'Inference: {inference_time:.1f}ms | FPS: {fps:.1f} | Detections: {len(detections)}'
        cv2.putText(output, stats, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        return output


def main():
    parser = argparse.ArgumentParser(description='Test YOLO detector with webcam')
    parser.add_argument('--camera', type=str, default='0',
                       help='Camera device ID or URL (e.g., 0, 1, or http://192.168.1.100:8080/video)')
    parser.add_argument('--model', type=str,
                       default='src/op3_yolo_vision/models/yolo.onnx',
                       help='Path to YOLO ONNX model')
    parser.add_argument('--gpu', action='store_true', help='Use GPU acceleration')
    parser.add_argument('--input-size', type=int, default=640, help='YOLO input size (default: 640)')
    parser.add_argument('--width', type=int, default=1280, help='Camera width')
    parser.add_argument('--height', type=int, default=720, help='Camera height')

    args = parser.parse_args()

    # Check model exists
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"✗ Model not found: {args.model}")
        print("Available models:")
        for p in Path('.').rglob('*.onnx'):
            print(f"  - {p}")
        return

    print("=" * 60)
    print("YOLO Webcam Test")
    print("=" * 60)
    print(f"Model: {model_path}")
    camera_display = f"/dev/video{args.camera}" if args.camera.isdigit() else args.camera
    print(f"Camera: {camera_display}")
    print(f"Resolution: {args.width}x{args.height}")
    print(f"OpenCV version: {cv2.__version__}")
    print("=" * 60)

    # Initialize detector
    try:
        detector = YOLODetector(str(model_path), use_gpu=args.gpu, input_size=args.input_size)
    except Exception as e:
        print(f"✗ Failed to load model: {e}")
        return

    # Open camera
    camera_input = args.camera
    if camera_input.isdigit():
        camera_input = int(camera_input)
        print(f"Camera: /dev/video{camera_input}")
    else:
        print(f"Camera: {camera_input}")

    cap = cv2.VideoCapture(camera_input)
    if not cap.isOpened():
        print(f"✗ Failed to open camera: {camera_input}")
        return

    # Only set resolution for local cameras (not IP cameras)
    if isinstance(camera_input, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"✓ Camera opened: {actual_width}x{actual_height}")
    print("\nPress 'q' to quit, 's' to save screenshot")
    print("=" * 60)

    frame_count = 0
    total_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to read frame")
                break

            # Run detection
            detections, inference_time = detector.detect(frame)

            # Draw results
            output = detector.draw_detections(frame, detections, inference_time)

            # Show
            cv2.imshow('YOLO Webcam Test', output)

            # Stats
            frame_count += 1
            total_time += inference_time

            if frame_count % 30 == 0:
                avg_time = total_time / frame_count
                avg_fps = 1000.0 / avg_time if avg_time > 0 else 0
                print(f"Frames: {frame_count} | Avg: {avg_time:.1f}ms ({avg_fps:.1f} FPS)")

            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f'screenshot_{int(time.time())}.jpg'
                cv2.imwrite(filename, output)
                print(f"✓ Saved {filename}")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        if frame_count > 0:
            avg_time = total_time / frame_count
            avg_fps = 1000.0 / avg_time if avg_time > 0 else 0
            print("\n" + "=" * 60)
            print("Session Summary:")
            print(f"  Total frames: {frame_count}")
            print(f"  Average inference: {avg_time:.1f}ms")
            print(f"  Average FPS: {avg_fps:.1f}")
            print("=" * 60)


if __name__ == '__main__':
    main()
