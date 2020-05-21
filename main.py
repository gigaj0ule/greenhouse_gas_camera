#!/usr/bin/env python3

import tkinter as tk
import cv2
import numpy as np
import datetime

from PIL import ImageTk, Image

import queue, time, threading, os

import pickle
import copy

from camcmds import axsys_control

# Camera hardware info
mwir_camera_video_index = 0
mwir_camera_serial_port = "/dev/ttyUSB0"
mwir_camera_type = 1

use_camera = True #False

# Color map
#colormap = np.array([[[0,0,4],[1,0,5],[1,1,6],[1,1,8],[2,1,10],[2,2,12],[2,2,14],[3,2,16],[4,3,18],[4,3,20],[5,4,23],[6,4,25],[7,5,27],[8,5,29],[9,6,31],[10,7,34],[11,7,36],[12,8,38],[13,8,41],[14,9,43],[16,9,45],[17,10,48],[18,10,50],[20,11,52],[21,11,55],[22,11,57],[24,12,60],[25,12,62],[27,12,65],[28,12,67],[30,12,69],[31,12,72],[33,12,74],[35,12,76],[36,12,79],[38,12,81],[40,11,83],[41,11,85],[43,11,87],[45,11,89],[47,10,91],[49,10,92],[50,10,94],[52,10,95],[54,9,97],[56,9,98],[57,9,99],[59,9,100],[61,9,101],[62,9,102],[64,10,103],[66,10,104],[68,10,104],[69,10,105],[71,11,106],[73,11,106],[74,12,107],[76,12,107],[77,13,108],[79,13,108],[81,14,108],[82,14,109],[84,15,109],[85,15,109],[87,16,110],[89,16,110],[90,17,110],[92,18,110],[93,18,110],[95,19,110],[97,19,110],[98,20,110],[100,21,110],[101,21,110],[103,22,110],[105,22,110],[106,23,110],[108,24,110],[109,24,110],[111,25,110],[113,25,110],[114,26,110],[116,26,110],[117,27,110],[119,28,109],[120,28,109],[122,29,109],[124,29,109],[125,30,109],[127,30,108],[128,31,108],[130,32,108],[132,32,107],[133,33,107],[135,33,107],[136,34,106],[138,34,106],[140,35,105],[141,35,105],[143,36,105],[144,37,104],[146,37,104],[147,38,103],[149,38,103],[151,39,102],[152,39,102],[154,40,101],[155,41,100],[157,41,100],[159,42,99],[160,42,99],[162,43,98],[163,44,97],[165,44,96],[166,45,96],[168,46,95],[169,46,94],[171,47,94],[173,48,93],[174,48,92],[176,49,91],[177,50,90],[179,50,90],[180,51,89],[182,52,88],[183,53,87],[185,53,86],[186,54,85],[188,55,84],[189,56,83],[191,57,82],[192,58,81],[193,58,80],[195,59,79],[196,60,78],[198,61,77],[199,62,76],[200,63,75],[202,64,74],[203,65,73],[204,66,72],[206,67,71],[207,68,70],[208,69,69],[210,70,68],[211,71,67],[212,72,66],[213,74,65],[215,75,63],[216,76,62],[217,77,61],[218,78,60],[219,80,59],[221,81,58],[222,82,56],[223,83,55],[224,85,54],[225,86,53],[226,87,52],[227,89,51],[228,90,49],[229,92,48],[230,93,47],[231,94,46],[232,96,45],[233,97,43],[234,99,42],[235,100,41],[235,102,40],[236,103,38],[237,105,37],[238,106,36],[239,108,35],[239,110,33],[240,111,32],[241,113,31],[241,115,29],[242,116,28],[243,118,27],[243,120,25],[244,121,24],[245,123,23],[245,125,21],[246,126,20],[246,128,19],[247,130,18],[247,132,16],[248,133,15],[248,135,14],[248,137,12],[249,139,11],[249,140,10],[249,142,9],[250,144,8],[250,146,7],[250,148,7],[251,150,6],[251,151,6],[251,153,6],[251,155,6],[251,157,7],[252,159,7],[252,161,8],[252,163,9],[252,165,10],[252,166,12],[252,168,13],[252,170,15],[252,172,17],[252,174,18],[252,176,20],[252,178,22],[252,180,24],[251,182,26],[251,184,29],[251,186,31],[251,188,33],[251,190,35],[250,192,38],[250,194,40],[250,196,42],[250,198,45],[249,199,47],[249,201,50],[249,203,53],[248,205,55],[248,207,58],[247,209,61],[247,211,64],[246,213,67],[246,215,70],[245,217,73],[245,219,76],[244,221,79],[244,223,83],[244,225,86],[243,227,90],[243,229,93],[242,230,97],[242,232,101],[242,234,105],[241,236,109],[241,237,113],[241,239,117],[241,241,121],[242,242,125],[242,244,130],[243,245,134],[243,246,138],[244,248,142],[245,249,146],[246,250,150],[248,251,154],[249,252,157],[250,253,161],[252,255,164]]])


# =================================================================================
if use_camera is not False:
    # Try to open camera video stream
    try:
        mwir_camera = cv2.VideoCapture(int(mwir_camera_video_index))
    except:
        print("Could not open video stream {}".format(mwir_camera_video_index))
        mwir_camera = None
else:
    mwir_camera = None

# Set capture size
if mwir_camera is not None:
    print(mwir_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 320))
    print(mwir_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240))

    os.system("v4l2-ctl -d /dev/video"+str(mwir_camera_video_index)+" -i 0 -s 0 --set-fmt-video=width=320,height=240,pixelformat=4")

mwir_camera_cmd = axsys_control(mwir_camera_serial_port, camera_type=mwir_camera_type)


# =================================================================================
# This thread will implement saving a video
def ir_capture(application_shutdown_signal, ir_frames, is_recording, video_shape, video_fps):

    # Output dir
    video_dir = f"{os.path.dirname(os.path.realpath(__file__))}/videos"

    try:
        os.mkdir(video_dir)
    except Exception as e:
        print(f"ERROR {str(e)}: {video_dir}")

    while not application_shutdown_signal.is_set():
        
        # Wait until we started recording
        is_recording.wait()

        # We started recording!

        # Create an output file with name as present datetime
        out_time = datetime.datetime.now().strftime("%m-%d-%Y-%H-%M-%S")
        out_file = f"{video_dir}/{out_time}"
        out_file_raw = f"{out_file}_raw.mp4"
        out_file_processed = f"{out_file}_processed.mp4"

        # Debug
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        # Start Recording
        print(f"Starting recording to {out_file_raw}")
        raw_writer = cv2.VideoWriter(out_file_raw, fourcc, video_fps, (video_shape[1], video_shape[0]), True)

        print(f"Starting recording to {out_file_processed}")
        processed_writer = cv2.VideoWriter(out_file_processed, fourcc, video_fps, (video_shape[1], video_shape[0]), True)

        # Get new frames and write them to disk
        while is_recording.is_set():
            try:
                new_frame = ir_frames.get(block=True, timeout=1)

                # Save raw frame
                new_raw_frame = new_frame[0].astype(np.uint8)
                new_raw_frame_rgb = cv2.cvtColor(new_raw_frame, cv2.COLOR_BGR2RGB)
                raw_writer.write(new_raw_frame_rgb)

                # Save processed frame
                new_processed_frame = new_frame[1].astype(np.uint8)
                new_processed_frame_rgb = cv2.cvtColor(new_processed_frame, cv2.COLOR_BGR2RGB)
                processed_writer.write(new_processed_frame_rgb)

            except Exception as e:
                print(f"No new frames to write... {str(e)}")

        # Release recorders
        raw_writer.release()
        print(f"Finished recording to {out_file_raw}")

        processed_writer.release()
        print(f"Finished recording to {out_file_processed}")


# This thread will implement saving a snapshot
def ir_snapshot(application_shutdown_signal, ir_frames):

    # Output dir
    snapshot_dir = f"{os.path.dirname(os.path.realpath(__file__))}/snapshots"

    try:
        os.mkdir(snapshot_dir)
    except Exception as e:
        print(f"ERROR {str(e)}: {snapshot_dir}")

    while not application_shutdown_signal.is_set():
       
        try:     
            new_frames = ir_frames.get(block=True, timeout=1)
            
            out_time = datetime.datetime.now().strftime("%m-%d-%Y-%H-%M-%S")
            out_file = f"{snapshot_dir}/{out_time}"
            out_file_raw = f"{out_file}_raw.png"
            out_file_processed = f"{out_file}_processed.png"
            
            new_frame = new_frames[0]
            new_frame_rgb = cv2.cvtColor(new_frame, cv2.COLOR_BGR2RGB)

            cv2.imwrite(out_file_raw, new_frame_rgb)
            print(f"Saved snapshot to {out_file_raw}")

            new_processed_frame = new_frames[1]
            new_processed_frame_rgb = cv2.cvtColor(new_processed_frame, cv2.COLOR_BGR2RGB)

            cv2.imwrite(out_file_processed, new_processed_frame_rgb)
            print(f"Saved processed snapshot to {out_file_processed}")

        except Exception as e:
            pass
            #print(f"No new frames to write... {str(e)}")

# =================================================================================
# Post Processing
OVERLAY_HUE = 0 # red
OVERLAY_BRIGTNESS = 0.5
EDGES_CONTRAST = 0.3
EDGES_THRESH1 = 40
EDGES_THRESH2 = 80


# =================================================================================
# User interface
class DualCamera():
    def __init__(self):
        self.root = tk.Tk()
        #self.root.attributes('-fullscreen', True)
        self.root.wm_title("Pollution Watch")

        # Sepecial modes
        self.saving = False
        self.overlay = False
        self.snapshot = False

        # Internal settings / state
        self.SATURATION = 8
        self.FILTER_K = 2

        # Videos
        self.mwir_cam_image = ImageTk.PhotoImage(image=Image.fromarray(np.uint8(256 * np.random.rand(480, 740))))
        self.mwir_cam_panel = tk.Label(image = self.mwir_cam_image)
        self.mwir_cam_panel.grid(row = 0, rowspan=10, column = 6, columnspan=4)

        # Separator
        self.vertical_line = tk.Label(text = '      ')
        self.vertical_line.grid(row = 0, column = 5)
        
        # Buttons
        row_actions = 0

        self.actions_label = tk.Label(text = 'Actions')
        self.actions_label.grid(row = row_actions, column = 0)

        self.overlay_button = tk.Button(width=10, height=2, text='Motion', command=self.toggle_overlay)
        self.overlay_button.grid(row = row_actions, column = 1)

        self.close_button = tk.Button(width = 10, height = 2, text = 'Close', command=self.quit)
        self.close_button.grid(row = row_actions, column = 2)

        # Buttons
        row_recording = row_actions + 1

        self.actions_label = tk.Label(text = 'Save')
        self.actions_label.grid(row = row_recording, column = 0)

        self.record_button = tk.Button(width = 10, height = 2, text = 'Record', command=self.save)
        self.record_button.grid(row = row_recording, column = 1)

        self.snapshot_button = tk.Button(width=10, height=2, text='Snapshot', command=self.take_snapshot)
        self.snapshot_button.grid(row = row_recording, column = 2)

        # Calibration
        row_calibration = row_recording + 1
        self.calibrate_label = tk.Label(text = 'Calibration')
        self.calibrate_label.grid(row = row_calibration, column = 0)

        self.calibrate_button = tk.Button(width = 10, height = 2, text = 'Flat Field', command=self.SET_calibrate)
        self.calibrate_button.grid(row = row_calibration, column = 1)

        self.autocal_off_button = tk.Button(width = 10, height = 2, text = 'Autocal Off', command=self.SET_cam_autocal_off)
        self.autocal_off_button.grid(row = row_calibration, column = 2)

        self.autocal_on_button = tk.Button(width = 10, height = 2, text = 'Autocal On', command=self.SET_cam_autocal_on)
        self.autocal_on_button.grid(row = row_calibration, column = 3)


        # Sensitivity
        row_sensitivity = row_calibration + 1

        self.sensitivity_label = tk.Label(text = 'Sensitivty')
        self.sensitivity_label.grid(row = row_sensitivity, column = 0)

        self.high_sensitivity_button = tk.Button(width = 10, height = 2, text = 'High', command=self.SET_cam_sensitivity_high)
        self.high_sensitivity_button.grid(row = row_sensitivity, column = 1)

        self.low_sensitivity_button = tk.Button(width = 10, height = 2, text = 'Low', command=self.SET_cam_sensitivity_low)
        self.low_sensitivity_button.grid(row = row_sensitivity, column = 2)

        # Lens Buttons
        row_lens = row_sensitivity + 1

        self.lens_label = tk.Label(self.root, text="Lens")
        self.lens_label.grid(row = row_lens, column = 0)

        self.fov_n_button = tk.Button(width = 10, height = 2, text = 'FOV N', command=self.SET_cam_fovn)
        self.fov_n_button.grid(row = row_lens, column = 1)

        self.fov_w_button = tk.Button(width = 10, height = 2, text = 'FOV M', command=self.SET_cam_fovm)
        self.fov_w_button.grid(row = row_lens, column = 2)

        self.fov_w_button = tk.Button(width = 10, height = 2, text = 'FOV W', command=self.SET_cam_fovw)
        self.fov_w_button.grid(row = row_lens, column = 3)

        # Focus Buttons
        row_focus = row_lens + 1

        self.focus_label = tk.Label(self.root, text="Focus")
        self.focus_label.grid(row = row_focus, column = 0)

        self.focus_far_button = tk.Button(width = 10, height = 2, text = 'FOCUS Far', command=self.SET_cam_focusin)
        self.focus_far_button.grid(row = row_focus, column = 1)

        self.focus_near_button = tk.Button(width = 10, height = 2, text = 'FOCUS Near', command=self.SET_cam_focusnear)
        self.focus_near_button.grid(row = row_focus, column = 2)

        self.focus_auto_button = tk.Button(width = 10, height = 2, text = 'AUTOFOCUS', command=self.SET_cam_autofocus)
        self.focus_auto_button.grid(row = row_focus, column = 3)

        # AGC
        row_agc = row_focus + 1

        self.agc_label = tk.Label(self.root, text="A.G.C.")
        self.agc_label.grid(row = row_agc, column = 0)

        self.agc_on_button = tk.Button(width = 10, height = 2, text = 'AGC On', command=self.SET_agc_on)
        self.agc_on_button.grid(row = row_agc, column = 1)

        self.agc_off_button = tk.Button(width = 10, height = 2, text = 'AGC Off', command=self.SET_agc_off)
        self.agc_off_button.grid(row = row_agc, column = 2)

        # Gain Slider
        row_gain = 10

        self.mwir_camera_gain = tk.IntVar()
        self.gain_label = tk.Label(self.root, text="Gain")
        self.gain_label.grid(row=row_gain, column=0) 
        self.gain_slider = tk.Scale(self.root, from_=0, to=4096, length=1200, variable=self.mwir_camera_gain, orient=tk.HORIZONTAL, command=self.SET_gain)
        self.gain_slider.grid(row = row_gain, rowspan = 1, column = 1, columnspan=9)

        # Bias slider
        row_bias = row_gain + 1
        self.mwir_camera_bias = tk.IntVar()
        self.bias_label = tk.Label(self.root, text="Bias")
        self.bias_label.grid(row=row_bias, column=0)
        self.bias_slider = tk.Scale(self.root, from_=0, to=4096, length=1200, variable=self.mwir_camera_bias, orient=tk.HORIZONTAL, command=self.SET_bias)
        self.bias_slider.grid(row = row_bias, rowspan =1, column = 1, columnspan=9)


        # Overlay slider
        row_overlay_saturation = row_bias + 1
        self.mwir_camera_bias = tk.IntVar()
        self.saturation_label = tk.Label(self.root, text="Motion")
        self.saturation_label.grid(row=row_overlay_saturation, column=0)
        self.saturation_slider = tk.Scale(self.root, from_=0, to=10, length=1200, variable=self.SATURATION, orient=tk.HORIZONTAL, command=self.SET_saturation)
        self.saturation_slider.grid(row = row_overlay_saturation, rowspan =1, column = 1, columnspan=9)
        self.saturation_slider.set(self.SATURATION)

        # Filter k slider
        row_filter_k = row_overlay_saturation + 1

        self.filter_k_label = tk.Label(self.root, text="Filter K")
        self.filter_k_label.grid(row=row_filter_k, column=0)
        self.filter_k_slider = tk.Scale(self.root, from_=1, to=10, length=1200, variable=self.FILTER_K, orient=tk.HORIZONTAL, command=self.SET_filter_k)
        self.filter_k_slider.grid(row = row_filter_k, rowspan=1, column=1, columnspan=9)
        self.filter_k_slider.set(self.FILTER_K)
        



        """
        # set default positions
        self.cm_slider.set(0x0C)
        self.clock_slider.set(0x15)
        self.bias_slider.set(0x05)
        """

        # App kill switch
        self.application_shutdown_signal = threading.Event()

        # Grab an image from the camera so we know what the stream looks like
        if mwir_camera is not None:
            ret, mwir_frame = mwir_camera.read()

            # New frames
            self.new_mwir_frame = None
            self.new_post_processed_mwir_frame = None

            # Rescale video for display
            self.new_video_height = 660
            self.orig_video_height = mwir_frame.shape[0]
            self.orig_video_width = mwir_frame.shape[1]
            video_scale_factor =  self.new_video_height / self.orig_video_height
            self.new_video_width = int(video_scale_factor * mwir_frame.shape[1])
            self.video_display_size = (self.new_video_width, self.new_video_height)
            
            # Save video shape
            self.orig_video_shape = mwir_frame.shape
            self.mwir_camera_fps = mwir_camera.get(cv2.CAP_PROP_FPS)

            # Low pass frame
            self.low_pass_frame = np.zeros((self.orig_video_shape[0], self.orig_video_shape[1]), np.float32)

            # thread for reading from sensor hardware intro an image queue           
            self.ir_frames = queue.Queue()
            self.is_recording_signal = threading.Event()

            self.capture_thread = threading.Thread(
                target=ir_capture,
                name="recording_thread",
                args=[  self.application_shutdown_signal, 
                        self.ir_frames, 
                        self.is_recording_signal, 
                        self.orig_video_shape, 
                        self.mwir_camera_fps
                    ]
                )

            self.capture_thread.start()

            # Thread for saving snapshots
            self.snapshot_frames = queue.Queue()

            self.snapshot_thread = threading.Thread(
                target=ir_snapshot,
                name="snapshot_thread",
                args=[  self.application_shutdown_signal, 
                        self.snapshot_frames
                    ]
                )
            
            self.snapshot_thread.daemon = True
            self.snapshot_thread.start()

        # Communication thread polls BRICON
        self.comms_thread = threading.Thread(target=self.comms_thread_fn,
            name="comms_thread",
            args=[]
        )

        self.comms_thread.daemon = True
        self.comms_thread.start()

        # Let's roll!
        self.ticktock()
        self.root.mainloop()


    def ticktock(self):

        # grab an image from the camera
        if mwir_camera is not None:
            ret, mwir_frame = mwir_camera.read()

            if mwir_frame is not None:
                #frame = imutils.resize(frame, height=240)
                new_mwir_frame_int = cv2.cvtColor(mwir_frame, cv2.COLOR_BGR2GRAY)
                self.new_mwir_frame = new_mwir_frame_int.astype(np.float32)
                
                # RGB
                """
                differenceFrame = np.clip((self.low_pass_frame - self.new_mwir_frame) * self.SATURATION, 0, 255).astype(np.float32)
                edges = (255 - cv2.Canny(self.new_mwir_frame.astype(np.uint8), EDGES_THRESH1, EDGES_THRESH2) * EDGES_CONTRAST).astype(np.uint8)
                
                # Highlight edges
                egdes_frame = self.new_mwir_frame.astype(np.float32)
                egdes_frame[edges != 255] = [0]

                # Overlay Motion
                weighted_difference_channel = cv2.addWeighted(egdes_frame, 0.4, differenceFrame, 0.6, 1)

                # Color format seems to be BRG
                self.overlay_frame = cv2.merge((weighted_difference_channel, egdes_frame, egdes_frame))
                
                # Export frame
                window_preview_frame = self.overlay_frame.astype(np.uint8)
                
                #cv2.cvtColor(self.overlay_frame, cv2.COLOR_HSV2BGR)
                #window_preview_frame = self.new_mwir_frame
                """

                # HSV
                difference_frame = np.clip((self.low_pass_frame - self.new_mwir_frame) * self.SATURATION, 0, 255).astype(np.uint8)
                edges_frame = cv2.Canny(new_mwir_frame_int, EDGES_THRESH1, EDGES_THRESH2)

                # Combine edges with a lightened version of the original image to make a background reference image
                reference_channel = cv2.addWeighted(edges_frame, -EDGES_CONTRAST, new_mwir_frame_int, 1 - OVERLAY_BRIGTNESS, OVERLAY_BRIGTNESS * 256)

                # Set up an HSV output image to combine background as brightness and difference as color saturation
                hsv_overlay_frame = np.zeros((self.low_pass_frame.shape[0], self.low_pass_frame.shape[1], 3), np.uint8)
                hsv_overlay_frame[...,0] = OVERLAY_HUE

                # Difference determines color saturation/intensity
                hsv_overlay_frame[...,1] = difference_frame

                # Reference determines value/brightness
                hsv_overlay_frame[...,2] = reference_channel
                #hsv_overlay_frame[...,2] = cv2.addWeighted(reference_channel, 0.4, difference_frame, 0.6, 1)

                self.new_post_processed_mwir_frame = cv2.cvtColor(hsv_overlay_frame, cv2.COLOR_HSV2RGB)

                # What does preview show?
                if self.overlay:
                    # Color format is RGB
                    window_preview_frame = self.new_post_processed_mwir_frame
                else:
                    window_preview_frame = self.new_mwir_frame.astype(np.uint8)
                
                # Scale 
                scaled_mwir_frame = cv2.resize(window_preview_frame, self.video_display_size, interpolation = cv2.INTER_NEAREST)

                # Create image from video
                image = Image.fromarray(scaled_mwir_frame)

                #if not self.overlay:
                self.mwir_cam_image = ImageTk.PhotoImage(image=image)
                self.mwir_cam_panel.configure(image=self.mwir_cam_image)

            # Are we saving a video?
            if self.is_recording_signal.is_set():
                self.ir_frames.put([self.new_mwir_frame, self.new_post_processed_mwir_frame])

            # Is this a snapshot?
            if self.snapshot is True:
                self.snapshot = False
                print("Taking snapshot...")
                self.snapshot_frames.put([self.new_mwir_frame, self.new_post_processed_mwir_frame])

        # Get feedback from camera port
        feedback = mwir_camera_cmd.readPort()

        # Anything in the port?
        if(feedback):
            print(feedback.decode('latin-1'))

        # Tick
        self.root.after(10, self.ticktock)

        # Save old frame
        if self.low_pass_frame is None:
            self.low_pass_frame = self.new_mwir_frame
        else:
            self.low_pass_frame += (self.FILTER_K / 10) * (self.new_mwir_frame - self.low_pass_frame)


    # Update brightness and contrast sliders
    def comms_thread_fn(self):
        while not self.application_shutdown_signal.is_set():
            if(mwir_camera_cmd):
                self.GET_bricon()
            time.sleep(1)

    # Program operations
    def quit(self):
        self.application_shutdown_signal.set()
        self.is_recording_signal.clear()
        self.root.quit()

    # Recording operations
    def save(self):
        self.record_button.configure(text = 'Stop Recording', command=self.stop_save)
        self.is_recording_signal.set()

    def stop_save(self):
        self.record_button.configure(text = 'Record', command=self.save)
        self.is_recording_signal.clear()

    def take_snapshot(self):
        print("Set snapshot to true")
        self.snapshot = True

    # AGC
    def SET_agc_on(self):
        cmd = "AGC:1"
        mwir_camera_cmd.camCmd(cmd)

    def SET_agc_off(self):
        cmd = "AGC:0"
        mwir_camera_cmd.camCmd(cmd)

    # Flat field calibration
    def SET_calibrate(self):
        cmd = "CAL1:"
        mwir_camera_cmd.camCmd(cmd)

    # Get settings
    def GET_bricon(self):
        cmd = "BRICON?:"
        response = mwir_camera_cmd.camCmdWithResponse(cmd)
        
        if(response is not None):
            response_ints = [int(i) for i in response.split() if i.isdigit()] 

            try:
                print(response_ints)
                brightness = int(response_ints[0])
                contrast = int(response_ints[1])

                self.mwir_camera_bias.set(brightness)
                self.mwir_camera_gain.set(contrast)
            except Exception as e:
                print(str(e))

    # Lens
    def SET_cam_fovn(self):
        cmd = "FOVN:"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_fovm(self):
        cmd = "FOVM:"
        mwir_camera_cmd.camCmd(cmd)
    
    def SET_cam_fovw(self):
        cmd = "FOVW:"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_focusin(self):
        cmd = "FOCUSI:"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_focusnear(self):
        cmd = "FOCUSN:"
        mwir_camera_cmd.camCmd(cmd)
        
    def SET_cam_autofocus(self):
        cmd = "AF:"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_autocal_off(self):
        cmd = "AUTOCALOFF:"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_autocal_on(self):
        cmd = "AUTOCALON:"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_sensitivity_low(self):
        cmd = "SENSTOG:0"
        mwir_camera_cmd.camCmd(cmd)

    def SET_cam_sensitivity_high(self):
        cmd = "SENSTOG:1"
        mwir_camera_cmd.camCmd(cmd)

    def SET_bias(self, val):
        val = int(val)
        cmd = "MOC:{:d}".format(val)
        mwir_camera_cmd.camCmd(cmd)

    def SET_gain(self, val):
        val = int(val)
        cmd = "MGC:{:d}".format(val)
        mwir_camera_cmd.camCmd(cmd)

    def SET_saturation(self, val):
        self.SATURATION = int(val)

    def SET_filter_k(self, val):
        self.FILTER_K = int(val)

    # Motion overlay
    def toggle_overlay(self):
        if self.overlay:
            self.overlay = False
            self.overlay_button.configure(text = 'Motion')
        else:
            self.overlay = True
            self.overlay_button.configure(text = 'No Motion')

app = DualCamera()
