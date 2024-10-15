import cv2
import imageio
import threading
import datetime, time
import numpy as np
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

        # usb camera init
        self.camera = cv2.VideoCapture(-1)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, f['video']['default_res_w'])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, f['video']['default_res_h'])

    def frame_process(self):
        try:
            success, input_frame = self.camera.read()
            if not success:
                self.camera.release()
                time.sleep(1)
                self.camera = cv2.VideoCapture(0)
        except Exception as e:
            print(f"[cv_ctrl.frame_process] error: {e}")
            input_frame = 255 * np.ones((480, 640, 3), dtype=np.uint8)
            cv2.putText(input_frame, f"camera read failed... \n{e}", 
                        (round(0.05*640), round(0.1*640 + 5 * 13)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.369, (0, 0, 0), 1)
            ret, buffer = cv2.imencode('.jpg', input_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
            input_frame = buffer.tobytes()
            return input_frame

        # opencv funcs
        if self.cv_mode != f['code']['cv_none']:
            if not self.cv_event.is_set():
                self.cv_event.set()
                self.opencv_threading(input_frame)
            try:
                mask = self.overlay.astype(bool)
                input_frame[mask] = self.overlay[mask]
                cv2.addWeighted(self.overlay, 1, input_frame, 1, 0, input_frame)
            except Exception as e:
                    print("An error occurred:", e)
        elif self.show_info_flag:
            if time.time() - self.info_update_time > self.info_show_time:
                self.show_info_flag = False
            try:
                self.overlay = input_frame.copy()
                cv2.rectangle(self.overlay,  (round((self.info_scale-0.005)*640), round((0.33)*480)), 
                                        (round(0.98*640), round((0.78)*480)), 
                                        self.info_bg_color, -1)
                cv2.addWeighted(self.overlay, 0.5, input_frame, 0.5, 0, input_frame)
            except Exception as e:
                print(f"[cv_ctrl.frame_process] error: {e}")

            # info_deque.appendleft(time.time())
            for i in range(0, len(self.info_deque)):
                cv2.putText(input_frame, str(self.info_deque[i]['text']), 
                            (round(self.info_scale*640), round(self.info_scale*640 - i * 20)), 
                            cv2.FONT_HERSHEY_SIMPLEX, self.info_deque[i]['size'], self.info_deque[i]['color'], 1)

        if self.show_base_info_flag:
            for i in range(0, len(self.recv_deque)):
                cv2.putText(input_frame, str(self.recv_deque[i]), 
                        (round(0.05*640), round(0.1*640 + i * 13)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.369, (255, 255, 255), 1)

        # render osd
        input_frame = self.osd_render(input_frame)

        # capture frame
        if self.picture_capture_flag:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            photo_filename = f'{self.photo_path}photo_{current_time}.jpg'
            try:
                cv2.imwrite(photo_filename, input_frame)
                self.picture_capture_flag = False
                print(photo_filename)
            except:
                pass

        # record video
        if not self.set_video_record_flag and not self.video_record_status_flag:
            pass
        elif self.set_video_record_flag and not self.video_record_status_flag:
            current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            video_filename = f'{self.video_path}video_{current_time}.mp4'
            self.writer = imageio.get_writer(video_filename, fps=30, codec='libx264')
            self.video_record_status_flag = True
        elif self.set_video_record_flag and self.video_record_status_flag:
            cv2.circle(input_frame, (15, 15), 5, (64, 64, 255), -1)
            rgb_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGRA2RGB)
            rgb_frame_3d = np.expand_dims(rgb_frame, axis=0)
            self.writer.append_data(rgb_frame_3d)
            # self.writer.append_data(np.array(cv2.cvtColor(input_frame, cv2.COLOR_BGRA2RGB)))
        elif not self.set_video_record_flag and self.video_record_status_flag:
            self.video_record_status_flag = False
            self.writer.close()

        # frame scale
        if self.scale_rate == 1:
            pass
        else:
            img_height, img_width = input_frame.shape[:2]
            img_width_d2  = img_width/2
            img_height_d2 = img_height/2
            x_start = int(img_width_d2 - (img_width_d2//self.scale_rate))
            x_end   = int(img_width_d2 + (img_width_d2//self.scale_rate))
            y_start = int(img_height_d2 - (img_height_d2//self.scale_rate))
            y_end   = int(img_height_d2 + (img_height_d2//self.scale_rate))
            input_frame = input_frame[y_start:y_end, x_start:x_end]

        # encode frame
        try:
            ret, buffer = cv2.imencode('.jpg', input_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
            input_frame = buffer.tobytes()
        except:
            pass

        # get fps
        self.fps_count += 1
        if time.time() - self.fps_start_time >= 2:
            self.video_fps = self.fps_count/2
            self.fps_count = 0
            self.fps_start_time = time.time()

        # output frame
        return input_frame


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