#!/usr/bin/env bash
set -euo pipefail

require_command() {
  local name="$1"
  if ! command -v "$name" >/dev/null 2>&1; then
    echo "Error: '$name' is not installed." >&2
    exit 1
  fi
}

read_with_default() {
  local prompt="$1"
  local default_value="$2"
  local user_value
  read -r -p "$prompt [$default_value]: " user_value
  if [[ -z "$user_value" ]]; then
    echo "$default_value"
  else
    echo "$user_value"
  fi
}

print_header() {
  echo "========================================"
  echo " Webcam Recorder (FFmpeg + V4L2)"
  echo "========================================"
}

select_device() {
  local devices=()
  mapfile -t devices < <(ls /dev/video* 2>/dev/null || true)

  if [[ ${#devices[@]} -eq 0 ]]; then
    echo "Error: no webcam devices found under /dev/video*." >&2
    exit 1
  fi

  echo
  echo "Available camera devices:"
  local i=1
  for dev in "${devices[@]}"; do
    echo "  $i) $dev"
    ((i++))
  done

  local choice
  read -r -p "Select device [1]: " choice
  choice="${choice:-1}"

  if ! [[ "$choice" =~ ^[0-9]+$ ]] || ((choice < 1 || choice > ${#devices[@]})); then
    echo "Invalid selection. Using ${devices[0]}."
    choice=1
  fi

  CAMERA_DEVICE="${devices[$((choice - 1))]}"
}

show_formats() {
  if ! command -v v4l2-ctl >/dev/null 2>&1; then
    return
  fi

  local ans
  read -r -p "Show camera-supported formats/resolutions? [y/N]: " ans
  if [[ "$ans" =~ ^[Yy]$ ]]; then
    echo
    v4l2-ctl --device="$CAMERA_DEVICE" --list-formats-ext
    echo
  fi
}

select_profile() {
  echo
  echo "Recording profiles:"
  echo "  1) Recommended: MJPEG copy to MKV (fast, your known working mode)"
  echo "  2) MP4 compatible: MJPEG -> H.264 MP4 (smaller files)"
  echo "  3) MJPEG copy to AVI (large files, broad legacy support)"
  echo "  4) Raw YUYV -> H.264 MP4 (if MJPEG input fails)"

  local profile_choice
  read -r -p "Select profile [1]: " profile_choice
  profile_choice="${profile_choice:-1}"

  case "$profile_choice" in
    1)
      PROFILE_NAME="mjpeg_mkv"
      INPUT_FORMAT="mjpeg"
      OUTPUT_EXT="mkv"
      VIDEO_CODEC_ARGS=(-c:v copy)
      ;;
    2)
      PROFILE_NAME="mp4_h264"
      INPUT_FORMAT="mjpeg"
      OUTPUT_EXT="mp4"
      VIDEO_CODEC_ARGS=(-c:v libx264 -profile:v high -level 3.1 -pix_fmt yuv420p -movflags +faststart)
      ;;
    3)
      PROFILE_NAME="mjpeg_avi"
      INPUT_FORMAT="mjpeg"
      OUTPUT_EXT="avi"
      VIDEO_CODEC_ARGS=(-c:v copy)
      ;;
    4)
      PROFILE_NAME="raw_to_h264"
      INPUT_FORMAT="yuyv422"
      OUTPUT_EXT="mp4"
      VIDEO_CODEC_ARGS=(-c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p -movflags +faststart)
      ;;
    *)
      echo "Invalid selection. Using profile 1."
      PROFILE_NAME="mjpeg_mkv"
      INPUT_FORMAT="mjpeg"
      OUTPUT_EXT="mkv"
      VIDEO_CODEC_ARGS=(-c:v copy)
      ;;
  esac
}

select_video_params() {
  echo
  echo "Resolution presets:"
  echo "  1) 1280x720"
  echo "  2) 1920x1080"
  echo "  3) 640x480"
  echo "  4) Custom"

  local res_choice
  read -r -p "Select resolution [1]: " res_choice
  res_choice="${res_choice:-1}"

  case "$res_choice" in
    1) VIDEO_SIZE="1280x720" ;;
    2) VIDEO_SIZE="1920x1080" ;;
    3) VIDEO_SIZE="640x480" ;;
    4)
      VIDEO_SIZE="$(read_with_default "Enter custom resolution (e.g. 960x720)" "1280x720")"
      ;;
    *)
      echo "Invalid selection. Using 1280x720."
      VIDEO_SIZE="1280x720"
      ;;
  esac

  FPS="$(read_with_default "FPS" "30")"

  if ! [[ "$FPS" =~ ^[0-9]+$ ]] || ((FPS <= 0)); then
    echo "Invalid FPS. Using 30."
    FPS="30"
  fi
}

select_duration_and_output() {
  local default_name="webcam_$(date +%Y%m%d_%H%M%S).${OUTPUT_EXT}"

  echo
  read -r -p "Duration in seconds (blank = press q to stop): " DURATION_SECONDS
  OUTPUT_FILE="$(read_with_default "Output filename" "$default_name")"

  if [[ -e "$OUTPUT_FILE" ]]; then
    local overwrite
    read -r -p "File exists. Overwrite? [y/N]: " overwrite
    if [[ ! "$overwrite" =~ ^[Yy]$ ]]; then
      echo "Aborted: not overwriting '$OUTPUT_FILE'."
      exit 0
    fi
    OVERWRITE_FLAG="-y"
  else
    OVERWRITE_FLAG="-n"
  fi
}

run_recording() {
  local cmd
  cmd=(ffmpeg "$OVERWRITE_FLAG" -f v4l2 -input_format "$INPUT_FORMAT" -framerate "$FPS" -video_size "$VIDEO_SIZE" -i "$CAMERA_DEVICE")

  if [[ -n "${DURATION_SECONDS:-}" ]]; then
    if [[ "$DURATION_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
      cmd+=(-t "$DURATION_SECONDS")
    else
      echo "Warning: invalid duration '$DURATION_SECONDS'; recording until manual stop."
    fi
  fi

  cmd+=("${VIDEO_CODEC_ARGS[@]}" "$OUTPUT_FILE")

  echo
  echo "Profile: $PROFILE_NAME"
  echo "Device: $CAMERA_DEVICE"
  echo "Input format: $INPUT_FORMAT"
  echo "Resolution: $VIDEO_SIZE"
  echo "FPS: $FPS"
  echo "Output: $OUTPUT_FILE"
  echo
  echo "Command:"
  printf '  %q' "${cmd[@]}"
  echo
  echo
  echo "Recording started. Press 'q' in ffmpeg to stop cleanly."
  echo

  "${cmd[@]}"
}

main() {
  require_command ffmpeg
  print_header
  select_device
  show_formats
  select_profile
  select_video_params
  select_duration_and_output
  run_recording
}

main "$@"
