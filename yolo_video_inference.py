#!/usr/bin/env python3
"""
Run YOLO inference on video file and save labeled output
Processes MP4/AVI videos with YOLO detection and saves annotated results

Usage:
    ./yolo_video_inference.py input.mp4                          # Basic usage
    ./yolo_video_inference.py input.mp4 --output labeled.mp4     # Custom output
    ./yolo_video_inference.py input.mp4 --gpu                    # Use GPU
    ./yolo_video_inference.py input.mp4 --fps 15                 # Change output FPS
    ./yolo_video_inference.py input.mp4 --no-display             # No preview (faster)
"""

import cv2
import numpy as np
import argparse
import time
from pathlib import Path
import json


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

    def draw_detections(self, image, detections, inference_time, frame_num):
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

            # Draw label with background
            label = f'{class_name} {score:.2f}'
            (label_w, label_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(output, (x, y - label_h - 10), (x + label_w, y), color, -1)
            cv2.putText(output, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Draw stats background
        stats_bg_height = 80
        overlay = output.copy()
        cv2.rectangle(overlay, (0, 0), (output.shape[1], stats_bg_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, output, 0.4, 0, output)

        # Draw stats
        fps = 1000.0 / inference_time if inference_time > 0 else 0
        cv2.putText(output, f'Frame: {frame_num}', (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(output, f'Inference: {inference_time:.1f}ms | FPS: {fps:.1f}', (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(output, f'Detections: {len(detections)}', (10, 75),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        return output


def process_video(input_path, output_path, model_path, use_gpu=False,
                  output_fps=None, show_display=True, save_json=False):
    """Process video file with YOLO detection."""

    # Check input exists
    if not Path(input_path).exists():
        print(f"✗ Input video not found: {input_path}")
        return False

    # Check model exists
    if not Path(model_path).exists():
        print(f"✗ Model not found: {model_path}")
        return False

    print("=" * 70)
    print("YOLO Video Inference")
    print("=" * 70)
    print(f"Input:  {input_path}")
    print(f"Output: {output_path}")
    print(f"Model:  {model_path}")
    print(f"OpenCV: {cv2.__version__}")
    print("=" * 70)

    # Load detector
    try:
        detector = YOLODetector(model_path, use_gpu=use_gpu)
    except Exception as e:
        print(f"✗ Failed to load model: {e}")
        return False

    # Open input video
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"✗ Failed to open video: {input_path}")
        return False

    # Get video properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    input_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fps = output_fps if output_fps else input_fps

    print(f"Resolution: {width}x{height}")
    print(f"Input FPS:  {input_fps:.2f}")
    print(f"Output FPS: {fps:.2f}")
    print(f"Frames:     {total_frames}")
    print("=" * 70)

    # Create output video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    if not out.isOpened():
        print(f"✗ Failed to create output video: {output_path}")
        cap.release()
        return False

    # Process frames
    frame_num = 0
    total_inference_time = 0
    all_detections = []  # For JSON export

    try:
        print("\nProcessing video...")
        print("Press 'q' to stop early\n")

        start_time = time.time()

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_num += 1

            # Run detection
            detections, inference_time = detector.detect(frame)
            total_inference_time += inference_time

            # Draw results
            labeled_frame = detector.draw_detections(frame, detections, inference_time, frame_num)

            # Write to output
            out.write(labeled_frame)

            # Save detections for JSON
            if save_json:
                frame_data = {
                    'frame': frame_num,
                    'timestamp': frame_num / fps,
                    'inference_ms': inference_time,
                    'detections': [
                        {
                            'class': detector.CLASS_NAMES[d['class_id']],
                            'class_id': d['class_id'],
                            'confidence': float(d['score']),
                            'bbox': d['box']
                        }
                        for d in detections
                    ]
                }
                all_detections.append(frame_data)

            # Display progress
            if frame_num % 30 == 0 or frame_num == total_frames:
                progress = (frame_num / total_frames) * 100
                avg_inference = total_inference_time / frame_num
                elapsed = time.time() - start_time
                eta = (elapsed / frame_num) * (total_frames - frame_num)

                print(f"Progress: {frame_num}/{total_frames} ({progress:.1f}%) | "
                      f"Avg: {avg_inference:.1f}ms | ETA: {eta:.1f}s", end='\r')

            # Show preview
            if show_display:
                cv2.imshow('YOLO Video Processing', labeled_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\n\nStopped by user")
                    break

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        # Cleanup
        cap.release()
        out.release()
        if show_display:
            cv2.destroyAllWindows()

        # Calculate stats
        total_time = time.time() - start_time
        avg_inference = total_inference_time / frame_num if frame_num > 0 else 0
        avg_fps = frame_num / total_time if total_time > 0 else 0

        print("\n" + "=" * 70)
        print("Processing Complete!")
        print("=" * 70)
        print(f"Output file:       {output_path}")
        print(f"Frames processed:  {frame_num}/{total_frames}")
        print(f"Total time:        {total_time:.2f}s")
        print(f"Avg inference:     {avg_inference:.1f}ms")
        print(f"Processing speed:  {avg_fps:.1f} FPS")

        # File size
        if Path(output_path).exists():
            size_mb = Path(output_path).stat().st_size / (1024 * 1024)
            print(f"Output size:       {size_mb:.2f} MB")

        # Save JSON if requested
        if save_json:
            json_path = output_path.replace('.mp4', '_detections.json')
            with open(json_path, 'w') as f:
                json.dump({
                    'video': input_path,
                    'model': model_path,
                    'total_frames': frame_num,
                    'fps': fps,
                    'resolution': [width, height],
                    'frames': all_detections
                }, f, indent=2)
            print(f"Detections JSON:   {json_path}")

        print("=" * 70)

    return True


def main():
    parser = argparse.ArgumentParser(
        description='Run YOLO inference on video and save labeled output',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s recording.mp4
  %(prog)s recording.mp4 --output labeled_video.mp4
  %(prog)s recording.mp4 --gpu --fps 15
  %(prog)s recording.mp4 --json --no-display
        """
    )

    parser.add_argument('input', type=str, help='Input video file (MP4, AVI, etc.)')
    parser.add_argument('--output', '-o', type=str, help='Output video file (default: input_labeled.mp4)')
    parser.add_argument('--model', type=str,
                       default='src/op3_yolo_vision/models/yolo.onnx',
                       help='Path to YOLO ONNX model')
    parser.add_argument('--gpu', action='store_true', help='Use GPU acceleration')
    parser.add_argument('--fps', type=float, help='Output FPS (default: same as input)')
    parser.add_argument('--no-display', action='store_true', help='Disable preview window (faster)')
    parser.add_argument('--json', action='store_true', help='Export detections to JSON file')

    args = parser.parse_args()

    # Generate output filename if not specified
    if not args.output:
        input_path = Path(args.input)
        args.output = str(input_path.parent / f"{input_path.stem}_labeled.mp4")

    # Process video
    success = process_video(
        input_path=args.input,
        output_path=args.output,
        model_path=args.model,
        use_gpu=args.gpu,
        output_fps=args.fps,
        show_display=not args.no_display,
        save_json=args.json
    )

    if success:
        print(f"\n✓ Success! Labeled video saved to: {args.output}")
    else:
        print("\n✗ Failed to process video")
        return 1

    return 0


if __name__ == '__main__':
    exit(main())
