import cv2
import imutils
import mediapipe as mp
import imageio
import threading
import datetime, time
import numpy as np
import math
import yaml, os, json, subprocess
from collections import deque
import textwrap
import depthai as dai

# config file.
curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)


class OpencvFuncs():
    def __init__(self, project_path, base_ctrl):
        self.base_ctrl = base_ctrl
        self.cv_event = threading.Event()
        self.cv_event.clear()
        self.cv_mode = f['code']['cv_none']
        self.detection_reaction_mode = f['code']['re_none']
        
        self.this_path = project_path
        self.photo_path = self.this_path + '/templates/pictures/'
        self.video_path = self.this_path + '/templates/videos/'
        self.frame_scale = 1
        self.picture_capture_flag = False
        self.set_video_record_flag = False
        self.video_record_status_flag = False
        self.writer = None
        self.overlay = None
        self.scale_rate = 1
        self.video_quality = f['video']['default_quality']

        # cv ctrl info
        self.cv_light_mode = 0
        self.pan_angle = 0
        self.tilt_angle = 0
        self.video_fps = 0
        self.fps_start_time = time.time()
        self.fps_count = 0
        self.cv_movtion_lock = True
        self.aimed_error = f['cv']['aimed_error']
        self.track_spd_rate = f['cv']['track_spd_rate']
        self.track_acc_rate = f['cv']['track_acc_rate']
        self.CMD_GIMBAL = f['cmd_config']['cmd_gimbal_ctrl']
        self.sampling_rad = f['cv']['sampling_rad']

        # reaction
        self.last_frame_capture_time = datetime.datetime.now()
        self.last_movtion_captured = datetime.datetime.now()

        # base data
        self.show_base_info_flag = False
        self.recv_deque = deque(maxlen=20)

        # info update
        self.show_info_flag = True
        self.info_update_time = time.time()
        self.info_deque = deque(maxlen=10)
        self.info_scale = 270 / 480
        self.info_bg_color = (0, 0, 0)
        self.info_show_time = 10
        self.recv_line_max = 26

        # mission funcs
        self.mission_flag = False

        # osd settings
        self.add_osd = f['base_config']['add_osd']

        # camera type detection
        self.usb_camera_connected = True

        # FPS calculation
        self.fps_start_time = time.time()
        self.fps_count = 0
        self.video_fps = 0

        # OSD settings
        self.add_osd = f['base_config']['add_osd']

        # OAK-D-Lite initialization
        self.oak_pipeline = self.create_oak_pipeline()
        self.oak_device = dai.Device(self.oak_pipeline)
        self.oak_rgb_queue = self.oak_device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.oak_depth_queue = self.oak_device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        # Add these lines for depth processing
        self.min_depth = 100  # minimum depth in millimeters
        self.max_depth = 10000  # maximum depth in millimeters

    def create_oak_pipeline(self):
        pipeline = dai.Pipeline()

        # RGB camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Mono cameras for depth
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Stereo Depth
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(640, 480)

        # Linking
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Create outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")

        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)

        return pipeline

    def create_depth_colormap(self):
        # Create a gradual colormap from red (close) to blue (far)
        colormap = np.zeros((256, 1, 3), dtype=np.uint8)
        for i in range(256):
            if i < 128:
                # Red to yellow
                colormap[i] = [0, int(i*2), 255]
            else:
                # Yellow to blue
                colormap[i] = [int((i-128)*2), 255, max(255 - int((i-128)*2), 0)]
        return colormap

    def frame_process(self):
        try:
            # Get RGB frame
            in_rgb = self.oak_rgb_queue.get()
            rgb_frame = in_rgb.getCvFrame()

            # Get depth frame
            in_depth = self.oak_depth_queue.get()
            depth_frame = in_depth.getFrame()

            # Clip depth values to our desired range
            depth_frame = np.clip(depth_frame, self.min_depth, self.max_depth)

            # Normalize the depth to 0-255 range
            depth_norm = ((depth_frame - self.min_depth) / (self.max_depth - self.min_depth) * 255).astype(np.uint8)

            # Apply gaussian blur to smooth the depth map
            depth_norm = cv2.GaussianBlur(depth_norm, (5, 5), 0)

            # Create a custom colormap
            colormap = self.create_depth_colormap()

            # Apply the colormap to the depth image
            depth_colormap = cv2.applyColorMap(depth_norm, colormap)

            # Ensure both frames are the same size
            depth_colormap = cv2.resize(depth_colormap, (rgb_frame.shape[1], rgb_frame.shape[0]))

            # Create a mask for areas with valid depth data
            mask = (depth_frame > self.min_depth) & (depth_frame < self.max_depth)

            # Create the blended frame
            blended_frame = rgb_frame.copy()
            blended_frame[mask] = cv2.addWeighted(rgb_frame[mask], 0.7, depth_colormap[mask], 0.3, 0)

            # Add OSD if enabled
            if self.add_osd:
                blended_frame = self.osd_render(blended_frame)

            # Capture frame if flag is set
            if self.picture_capture_flag:
                self.capture_picture(blended_frame)

            # Record video if flag is set
            if self.set_video_record_flag and self.video_record_status_flag:
                self.record_video(blended_frame)

            # Encode frame
            ret, buffer = cv2.imencode('.jpg', blended_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
            frame = buffer.tobytes()

            # Update FPS
            self.update_fps()

            return frame
        except Exception as e:
            print(f"[cv_ctrl.frame_process] error: {e}")
            # Return a blank frame or error message frame
            error_frame = 255 * np.ones((480, 640, 3), dtype=np.uint8)
            cv2.putText(error_frame, f"Camera read failed... {e}", 
                        (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            ret, buffer = cv2.imencode('.jpg', error_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
            return buffer.tobytes()
        

    def update_fps(self):
        self.fps_count += 1
        if time.time() - self.fps_start_time >= 2:
            self.video_fps = self.fps_count / 2
            self.fps_count = 0
            self.fps_start_time = time.time()
    
    def record_video(self, frame):
        if self.writer is None:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            video_filename = f'{self.video_path}video_{current_time}.mp4'
            self.writer = imageio.get_writer(video_filename, fps=30, codec='libx264')
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.writer.append_data(rgb_frame)
    
    def capture_picture(self, frame):
        current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        photo_filename = f'{self.photo_path}photo_{current_time}.jpg'
        try:
            cv2.imwrite(photo_filename, frame)
            self.picture_capture_flag = False
            print(f"Picture captured: {photo_filename}")
        except Exception as e:
            print(f"Failed to capture picture: {e}")


    def usb_camera_detection(self):
        lsusb_output = subprocess.check_output(["lsusb"]).decode("utf-8")
        if "Camera" in lsusb_output:
            print("USB Camera connected")
            return True
        else:
            print("USB Camera not connected")
            return False


    def osd_render(self, osd_frame):
        if not self.add_osd:
            return osd_frame

        # render lidar data
        lidar_points = []
        for lidar_angle, lidar_distance in zip(self.base_ctrl.rl.lidar_angles_show, self.base_ctrl.rl.lidar_distances_show):
            lidar_x = int(lidar_distance * np.cos(lidar_angle) * 0.05) + 320
            lidar_y = int(lidar_distance * np.sin(lidar_angle) * 0.05) + 240
            lidar_points.append((lidar_x, lidar_y))

        for lidar_point in lidar_points:
            cv2.circle(osd_frame, lidar_point, 3, (255, 0, 0), -1)

        # render sensor data
        sensor_index = 0
        for sensor_line in self.base_ctrl.rl.sensor_data:
            # sensor_line = sensor_line[:-2]
            cv2.putText(osd_frame, sensor_line,
                        (100, 50 + sensor_index * 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            sensor_index = sensor_index + 1


        return osd_frame

    def picture_capture(self):
        self.picture_capture_flag = True

    def video_record(self, input_cmd):
        if input_cmd:
            self.set_video_record_flag = True
        else:
            self.set_video_record_flag = False

    def scale_ctrl(self, input_rate):
        if input_rate < 1:
            self.scale_rate = 1
        else:
            self.scale_rate = input_rate

    def set_video_quality(self, input_quality):
        if input_quality < 1:
            self.video_quality = 1
        elif input_quality > 100:
            self.video_quality = 100
        else:
            self.video_quality = int(input_quality)

    def set_cv_mode(self, input_mode):
        self.cv_mode = input_mode
        if self.cv_mode == f['code']['cv_none']:
            self.set_video_record_flag = False


    def info_update(self, megs, color, size):
        if megs == -1:
            self.info_update_time = time.time()
            self.show_info_flag = True
            return
        wrapped_lines = textwrap.wrap(megs, self.recv_line_max)
        for line in wrapped_lines:
            self.info_deque.appendleft({'text':line,'color':color,'size':size})
        self.info_update_time = time.time()
        self.show_info_flag = True

    def commandline_ctrl(self, args_str):
        return

    def show_recv_info(self, input_cmd):
        if input_cmd == True:
            self.show_base_info_flag = True
        else:
            self.show_base_info_flag = False
        print(self.show_base_info_flag)

    def format_json_numbers(self, obj):
        if isinstance(obj, dict):
            return {k: self.format_json_numbers(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.format_json_numbers(elem) for elem in obj]
        elif isinstance(obj, float):
            return round(obj, 2)
        return obj

    def update_base_data(self, input_data):
        if not input_data:
            return
        try:
            if self.show_base_info_flag:
                self.recv_deque.appendleft(json.dumps(self.format_json_numbers(input_data)))
            if input_data['T'] == 1003:
                self.info_deque.appendleft({'text':json.dumps(input_data['mac']),'color':(16,64,255),'size':0.5})
                wrapped_lines = textwrap.wrap(json.dumps(input_data['megs']), self.recv_line_max)
                for line in wrapped_lines:
                    self.info_deque.appendleft({'text':line,'color':(255,255,255),'size':0.5})
                self.info_update_time = time.time()
                self.show_info_flag = True
        except Exception as e:
            print(f"[cv_ctrl.update_base_data] error: {e}")


    def head_light_ctrl(self, input_mode):
        self.cv_light_mode = input_mode
        if input_mode == 0:
            self.base_ctrl.lights_ctrl(self.base_ctrl.base_light_status, 0)
            self.cv_light_mode = input_mode
        elif input_mode == 2:
            self.base_ctrl.lights_ctrl(self.base_ctrl.base_light_status, 255)
            self.cv_light_mode = input_mode
        elif input_mode == 3:
            if self.cv_light_mode == 1:
                return
            elif self.base_ctrl.head_light_status == 0:
                self.cv_light_mode = 2
                self.base_ctrl.lights_ctrl(self.base_ctrl.base_light_status, 255)
            elif self.base_ctrl.head_light_status != 0:
                self.cv_light_mode = 0
                self.base_ctrl.lights_ctrl(self.base_ctrl.base_light_status, 0)