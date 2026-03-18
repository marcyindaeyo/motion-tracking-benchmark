import os
import copy
import argparse
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import mediapipe as mp  # type:ignore
from typing import List, Any, Dict, Tuple, Union
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from utils import CvFpsCalc
from utils.download_file import download_file
import csv
from datetime import datetime

# Shared state for callback communication
latest_result = None
latest_depth_image = None
latest_color_image = None
latest_timestamp = 0
latest_debug_image = None
state = ""


output_dir = "CameraLog"
os.makedirs(output_dir, exist_ok=True)
timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

# CSV setup
# position tracking
wrist_csv_filename = os.path.join(output_dir, f"camera_tracking_{timestamp_str}.csv")
wrist_csv_file = open(wrist_csv_filename, mode='w', newline='')
wrist_csv_writer = csv.writer(wrist_csv_file)

landmark_indices_to_record = {
    11: "LEFT_SHOULDER",
    12: "RIGHT_SHOULDER",
    13: "LEFT_ELBOW",
    14: "RIGHT_ELBOW",
    15: "LEFT_WRIST",
    16: "RIGHT_WRIST"
}
header = ["Time"]
for name in landmark_indices_to_record.values():
    header += [f"{name}_X", f"{name}_Y", f"{name}_Z"]
wrist_csv_writer.writerow(header)

# event logging
timestamp_csv_filename = os.path.join(output_dir, f"timestamp_recording_{timestamp_str}.csv")
timestamp_csv_file = open(timestamp_csv_filename, mode='w', newline='')
timestamp_csv_writer = csv.writer(timestamp_csv_file)
timestamp_csv_writer.writerow(["Event", "Timestamp"])

def result_callback(
    result: mp.tasks.vision.PoseLandmarkerResult,
    output_image: mp.Image,
    timestamp_ms: int,
):

    global latest_result, latest_timestamp
    latest_result = result
    latest_timestamp = timestamp_ms

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=int, choices=[0, 1, 2], default=0)
    return parser.parse_args()

def main():
    global latest_result, latest_depth_image, latest_color_image, latest_debug_image, state

    args = get_args()

    model: int = args.model

    # Predefined local model paths for 0 (lite), 1 (full), 2 (heavy)
    local_model_paths = [
        "model/pose_landmarker_lite.task",
        "model/pose_landmarker_full.task",
        "model/pose_landmarker_heavy.task"
    ]


    # Select the path according to --model index
    model_path: str = local_model_paths[model]

    BaseOptions = mp.tasks.BaseOptions
    PoseLandmarker = mp.tasks.vision.PoseLandmarker
    PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
    PoseLandmarkerResult = mp.tasks.vision.PoseLandmarkerResult
    VisionRunningMode = mp.tasks.vision.RunningMode

    options = PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        output_segmentation_masks=False,
        running_mode=VisionRunningMode.LIVE_STREAM,
        result_callback=result_callback
    )
    detector = PoseLandmarker.create_from_options(options)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    time.sleep(2)

    align = rs.align(rs.stream.color)
    color_intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    fps_calc = CvFpsCalc(buffer_len=10)

    while True:
        frames = pipeline.wait_for_frames(timeout_ms=1000)
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        latest_depth_image = depth_image
        latest_color_image = color_image

        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)
        timestamp_ms = int(time.time() * 1000)
        detector.detect_async(mp_image, timestamp_ms)

        if latest_result and latest_result.pose_landmarks:
            debug_image = copy.deepcopy(color_image)
            debug_image = draw_debug(debug_image, latest_result, fps_calc.get())
            latest_debug_image = debug_image

            for pose_landmarks in latest_result.pose_landmarks:
                landmark_3d_dict = {}
                for idx in landmark_indices_to_record.keys():
                    if idx < len(pose_landmarks):
                        landmark = pose_landmarks[idx]
                        pixel_x = int(landmark.x * color_image.shape[1])
                        pixel_y = int(landmark.y * color_image.shape[0])
                        if 0 <= pixel_x < color_image.shape[1] and 0 <= pixel_y < color_image.shape[0]:
                            depth_in_m = depth_image[pixel_y, pixel_x] * depth_scale
                            if 0 < depth_in_m < 10:
                                point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [pixel_x, pixel_y], depth_in_m)
                                landmark_3d_dict[idx] = point_3d

                row = [time.time()]
                for idx in landmark_indices_to_record:
                    point = landmark_3d_dict.get(idx)
                    if point is not None:
                        row += [point[0], point[1], point[2]]
                    else:
                        row += [float('nan'), float('nan'), float('nan')]
                wrist_csv_writer.writerow(row)


        cv2.imshow("Pose Landmarks", latest_debug_image if latest_debug_image is not None else color_image)
        gui_image = create_gui(700, 300, state)
        cv2.imshow("GUI", gui_image)

        key = cv2.waitKey(1)
        if key == 27:
            break
        elif key == ord('s'):
            state = "Start"
            timestamp_csv_writer.writerow(["Start", time.time()])
        elif key == ord('e'):
            state = "End"
            timestamp_csv_writer.writerow(["End", time.time()])

    pipeline.stop()
    wrist_csv_file.close()
    timestamp_csv_file.close()
    cv2.destroyAllWindows()
    print("Saved:", wrist_csv_filename, timestamp_csv_filename)

# GUI
def create_gui(window_width: int, window_height: int, state: str) -> np.ndarray:
    """ GUI function to display timestamp, status (Start/End), and buttons """
    font = cv2.FONT_HERSHEY_SIMPLEX
    gui_image = np.ones((window_height, window_width, 3), dtype=np.uint8) * 255  

    # Display Timestamp
    timestamp = time.time()
    cv2.putText(gui_image, f"Timestamp: {timestamp:.6f}s", (20, 50), font, 0.7, (0, 0, 0), 2)

    # Display Status (Start/End)
    if state == "Start":
        cv2.putText(gui_image, "Start", (20, 100), font, 1, (0, 255, 0), 2)  # Green
    elif state == "End":
        cv2.putText(gui_image, "End", (20, 100), font, 1, (0, 0, 255), 2)  # Red

    button_width = 200
    button_height = 80
    button_font_scale = 1
    cv2.rectangle(gui_image, (50, 200), (50 + button_width, 200 + button_height), (0, 255, 0), -1)  # Green
    cv2.putText(gui_image, "'s' to Start", (55, 240), font, button_font_scale, (255, 255, 255), 2)
    cv2.rectangle(gui_image, (300, 200), (300 + button_width, 200 + button_height), (0, 0, 255), -1)  # Red
    cv2.putText(gui_image, "'e' to End", (305, 240), font, button_font_scale, (255, 255, 255), 2)

    return gui_image


def blur_face(image: np.ndarray, landmarks: List[Any]) -> np.ndarray:
    face_indices = [0, 1, 2, 3, 4, 5, 6]
    points = []
    image_width, image_height = image.shape[1], image.shape[0]
    
    for idx in face_indices:
        lm = landmarks[idx]
        x = int(lm.x * image_width)
        y = int(lm.y * image_height)
        points.append((x, y))

    if points:
        xs, ys = zip(*points)
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        pad = 20
        roi = image[max(0, min_y - pad):min(image.shape[0], max_y + pad),
                    max(0, min_x - pad):min(image.shape[1], max_x + pad)]

        if roi.size > 0:
            blurred_roi = cv2.GaussianBlur(roi, (51, 51), 30)
            image[max(0, min_y - pad):min(image.shape[0], max_y + pad),
                  max(0, min_x - pad):min(image.shape[1], max_x + pad)] = blurred_roi

    return image


def draw_debug(
    image: np.ndarray,
    detection_result: vision.HandLandmarkerResult,  # type:ignore
    display_fps: float,
) -> np.ndarray:
    image_width, image_height = image.shape[1], image.shape[0]

    landmark_draw_info: Dict[
        int,
        Dict[str, Union[str, Tuple[int, int, int]]],
    ] = {
        0: {  # 鼻
            'name': 'NOSE',
            'color': (0, 255, 0)  # 緑
        },
        1: {  # 左目（内側）
            'name': 'LEFT_EYE_INNER',
            'color': (255, 0, 0)  # 赤
        },
        2: {  # 左目
            'name': 'LEFT_EYE',
            'color': (0, 0, 255)  # 青
        },
        3: {  # 左目（外側）
            'name': 'LEFT_EYE_OUTER',
            'color': (255, 255, 0)  # 黄
        },
        4: {  # 右目（内側）
            'name': 'RIGHT_EYE_INNER',
            'color': (0, 255, 255)  # シアン
        },
        5: {  # 右目
            'name': 'RIGHT_EYE',
            'color': (255, 0, 255)  # マゼンタ
        },
        6: {  # 右目（外側）
            'name': 'RIGHT_EYE_OUTER',
            'color': (128, 128, 128)  # グレー
        },
        7: {  # 左耳
            'name': 'LEFT_EAR',
            'color': (255, 128, 0)  # オレンジ
        },
        8: {  # 右耳
            'name': 'RIGHT_EAR',
            'color': (128, 0, 255)  # 紫
        },
        9: {  # 口（左）
            'name': 'MOUTH_LEFT',
            'color': (0, 128, 255)  # ライトブルー
        },
        10: {  # 口（右）
            'name': 'MOUTH_RIGHT',
            'color': (128, 255, 0)  # ライム
        },
        11: {  # 左肩
            'name': 'LEFT_SHOULDER',
            'color': (255, 128, 128)  # ライトレッド
        },
        12: {  # 右肩
            'name': 'RIGHT_SHOULDER',
            'color': (128, 128, 0)  # オリーブ
        },
        13: {  # 左肘
            'name': 'LEFT_ELBOW',
            'color': (0, 128, 128)  # ティール
        },
        14: {  # 右肘
            'name': 'RIGHT_ELBOW',
            'color': (128, 0, 128)  # マルーン
        },
        15: {  # 左手首
            'name': 'LEFT_WRIST',
            'color': (64, 64, 64)  # ダークグレー
        },
        16: {  # 右手首
            'name': 'RIGHT_WRIST',
            'color': (192, 192, 192)  # シルバー
        },
        17: {  # 左小指
            'name': 'LEFT_PINKY',
            'color': (255, 69, 0)  # レッドオレンジ
        },
        18: {  # 右小指
            'name': 'RIGHT_PINKY',
            'color': (75, 0, 130)  # インディゴ
        },
        19: {  # 左人差し指
            'name': 'LEFT_INDEX',
            'color': (173, 255, 47)  # グリーンイエロー
        },
        20: {  # 右人差し指
            'name': 'RIGHT_INDEX',
            'color': (220, 20, 60)  # クリムゾン
        },
        21: {  # 左親指
            'name': 'LEFT_THUMB',
            'color': (255, 0, 0)  # 赤
        },
        22: {  # 右親指
            'name': 'RIGHT_THUMB',
            'color': (0, 0, 255)  # 青
        },
        23: {  # 左腰
            'name': 'LEFT_HIP',
            'color': (0, 255, 0)  # 緑
        },
        24: {  # 右腰
            'name': 'RIGHT_HIP',
            'color': (255, 255, 0)  # 黄
        }
    }
    line_info_list: List[List[int]] = [

        [11, 12],  # Left shoulder to right shoulder
        [11, 13],  # Left shoulder to left elbow
        [13, 15],  # Left elbow to left wrist
        [12, 14],  # Right shoulder to right elbow
        [14, 16],  # Right elbow to right wrist
        [23, 24],  # Left hip to right hip     
        [11, 23],  # Left shoulder to left hip
        [12, 24]  # Right shoulder to right hip
    ]



    for pose_landmarks, _ in zip(
            detection_result.pose_landmarks,
            detection_result.pose_world_landmarks,
    ):
        image = blur_face(image, pose_landmarks)
        # Organize landmark info
        landmark_dict: Dict[int, List[Union[int, float]]] = {}
        for index, landmark in enumerate(pose_landmarks):
            if landmark.visibility < 0 or landmark.presence < 0:
                continue
            landmark_x: int = min(int(landmark.x * image_width),
                                  image_width - 1)
            landmark_y: int = min(int(landmark.y * image_height),
                                  image_height - 1)
            landmark_dict[index] = [landmark_x, landmark_y, landmark.z]

        # Draw connection lines
        for line_info in line_info_list:
            if line_info[0] in landmark_dict and line_info[1] in landmark_dict:
                cv2.line(image, tuple(landmark_dict[line_info[0]][:2]),
                         tuple(landmark_dict[line_info[1]][:2]), (220, 220, 220),
                         3, cv2.LINE_AA)  # type:ignore

        # Draw each landmark
        for index, landmark in landmark_dict.items():
            if index < 11 or index > 24 or (16 < index < 23):
                continue
            cv2.circle(image, (landmark[0], landmark[1]), 5,
                       landmark_draw_info[index]['color'], -1,
                       cv2.LINE_AA)

    # FPS
    if detection_result.segmentation_masks is None:
        color = (0, 255, 0)
    else:
        color = (255, 255, 255)
    cv2.putText(
        image,
        "FPS:" + str(display_fps),
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        color,
        2,
        cv2.LINE_AA,
    )

    return image



if __name__ == '__main__':
    main()
