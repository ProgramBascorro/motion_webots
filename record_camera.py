#!/usr/bin/env python3
"""
ROS2 Camera Topic Recorder
Records camera topic to video file (MP4) for later analysis (YOLO, debugging, etc.)

Usage:
    ./record_camera.py                          # Interactive mode
    ./record_camera.py --topic /camera/image    # Record specific topic
    ./record_camera.py --duration 30            # Record for 30 seconds
    ./record_camera.py --output my_video.mp4    # Custom output name
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import argparse
import sys
import signal
from datetime import datetime
import os


class CameraRecorder(Node):
    def __init__(self, topic_name, output_file, duration=None, fps=30, show_preview=True):
        super().__init__('camera_recorder')

        self.topic_name = topic_name
        self.output_file = output_file
        self.duration = duration
        self.fps = fps
        self.show_preview = show_preview

        self.writer = None
        self.frame_count = 0
        self.start_time = None
        self.recording = False

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Camera Recorder initialized")
        self.get_logger().info(f"  Topic: {topic_name}")
        self.get_logger().info(f"  Output: {output_file}")
        self.get_logger().info(f"  FPS: {fps}")
        if duration:
            self.get_logger().info(f"  Duration: {duration} seconds")
        else:
            self.get_logger().info(f"  Duration: Until Ctrl+C")

        if show_preview:
            self.get_logger().info(f"  Preview: Enabled (press 'q' to stop)")

        self.get_logger().info("Waiting for images...")

    def ros_to_cv2(self, msg):
        """Convert ROS Image message to OpenCV image without cv_bridge."""
        encoding = msg.encoding
        width, height = msg.width, msg.height

        if encoding == 'bgr8':
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
        elif encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif encoding == 'rgba8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif encoding == 'bgra8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 4)
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        elif encoding == 'mono8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width)
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f'Unsupported encoding: {encoding}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.ros_to_cv2(msg)
            height, width = cv_image.shape[:2]

            # Initialize video writer on first frame
            if self.writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(
                    self.output_file,
                    fourcc,
                    self.fps,
                    (width, height)
                )
                self.start_time = self.get_clock().now()
                self.recording = True
                self.get_logger().info(f"Recording started! Resolution: {width}x{height}")

            # Write frame
            if self.recording:
                self.writer.write(cv_image)
                self.frame_count += 1

                # Show preview with recording indicator
                if self.show_preview:
                    preview = cv_image.copy()

                    # Add recording indicator (red circle)
                    cv2.circle(preview, (30, 30), 15, (0, 0, 255), -1)
                    cv2.putText(preview, "REC", (50, 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                    # Add frame count
                    cv2.putText(preview, f"Frame: {self.frame_count}", (10, height - 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    # Add elapsed time
                    elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                    cv2.putText(preview, f"Time: {elapsed:.1f}s", (10, height - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    cv2.imshow('Recording Preview', preview)
                    key = cv2.waitKey(1)
                    if key == ord('q'):
                        self.get_logger().info("User pressed 'q' - stopping recording")
                        self.stop_recording()
                        rclpy.shutdown()

                # Check duration
                if self.duration:
                    elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                    if elapsed >= self.duration:
                        self.get_logger().info(f"Duration reached ({self.duration}s) - stopping")
                        self.stop_recording()
                        rclpy.shutdown()

                # Progress logging
                if self.frame_count % (self.fps * 5) == 0:  # Every 5 seconds
                    elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                    self.get_logger().info(
                        f"Recording... {self.frame_count} frames, {elapsed:.1f}s"
                    )

        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def stop_recording(self):
        if self.recording:
            self.recording = False

            if self.writer:
                self.writer.release()

            if self.show_preview:
                cv2.destroyAllWindows()

            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

            self.get_logger().info("=" * 50)
            self.get_logger().info("Recording Complete!")
            self.get_logger().info(f"  Output file: {self.output_file}")
            self.get_logger().info(f"  Total frames: {self.frame_count}")
            self.get_logger().info(f"  Duration: {elapsed:.2f} seconds")
            self.get_logger().info(f"  Average FPS: {self.frame_count / elapsed:.2f}")

            # Get file size
            if os.path.exists(self.output_file):
                size_mb = os.path.getsize(self.output_file) / (1024 * 1024)
                self.get_logger().info(f"  File size: {size_mb:.2f} MB")

            self.get_logger().info("=" * 50)


def list_image_topics():
    """List available image topics"""
    print("\nScanning for image topics...")
    os.system("ros2 topic list | grep -E '(image|camera)' || echo 'No image topics found'")
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Record ROS2 camera topic to MP4 video',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --topic /camera/image_raw --duration 30
  %(prog)s --topic /robotis_op3/camera/image_raw --output match.mp4
  %(prog)s --list
  %(prog)s --topic /vision/yolo/debug --fps 15 --no-preview
        """
    )

    parser.add_argument('--topic', '-t', type=str,
                       help='Camera topic to record (e.g., /camera/image_raw)')
    parser.add_argument('--output', '-o', type=str,
                       help='Output MP4 file (default: recording_YYYYMMDD_HHMMSS.mp4)')
    parser.add_argument('--duration', '-d', type=float,
                       help='Recording duration in seconds (default: until Ctrl+C)')
    parser.add_argument('--fps', type=int, default=30,
                       help='Video FPS (default: 30)')
    parser.add_argument('--no-preview', action='store_true',
                       help='Disable preview window')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List available image topics and exit')

    args = parser.parse_args()

    # Handle list option
    if args.list:
        list_image_topics()
        return

    # Interactive mode if no topic specified
    if not args.topic:
        print("\n" + "=" * 60)
        print("ROS2 Camera Recorder - Interactive Mode")
        print("=" * 60)
        list_image_topics()

        args.topic = input("Enter topic name (or press Enter for default): ").strip()
        if not args.topic:
            args.topic = "/robotis_op3/camera/image_raw"
            print(f"Using default: {args.topic}")

        duration_input = input("Recording duration in seconds (or press Enter for manual stop): ").strip()
        if duration_input:
            try:
                args.duration = float(duration_input)
            except ValueError:
                print("Invalid duration, using manual stop")

        print()

    # Generate output filename if not specified
    if not args.output:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"recording_{timestamp}.mp4"

    # Ensure .mp4 extension
    if not args.output.endswith('.mp4'):
        args.output += '.mp4'

    # Initialize ROS2
    rclpy.init()

    # Create recorder node
    recorder = CameraRecorder(
        topic_name=args.topic,
        output_file=args.output,
        duration=args.duration,
        fps=args.fps,
        show_preview=not args.no_preview
    )

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n\nCtrl+C detected - stopping recording...")
        recorder.stop_recording()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Start recording
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop_recording()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
