
import csv
import cv2
import os
import shlex
import subprocess
import tempfile
import olympe

from random import random

from datetime import datetime as dtm

class StreamProcessing:

    def __init__(self, drone, processing_fun, format_h264=True, mp4_folderpath="Documents/recorded-runs"):
        """"""
        self.drone = drone
        self.processing_fun = processing_fun

        unique_id = dtm.strftime(dtm.now(), "%Y%m%d_%H%M%S")
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_")
        self.finald = "{}/{}".format(os.path.expanduser("~"), mp4_folderpath)

        print("Olympe streaming temporal output dir: {}".format(self.tempd))
        print("Olympe streaming final output dir: {}".format(self.tempd))        
        
        # You can record the video stream from the drone if you plan to do some
        # post processing.
        self.h264_data_fname = "{}_h264_data.264".format(unique_id)
        self.h264_meta_fname = "{}_h264_metadata.json".format(unique_id)
        self.h264_mp4_fname = "{}_h264_data.mp4".format(unique_id)
        self.h264_stats_fname = "{}_h264_stats.csv".format(unique_id)

        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.finald, self.h264_stats_fname), 'w+')
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()

        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.tempd, self.h264_data_fname),
            h264_meta_file=os.path.join(self.tempd, self.h264_meta_fname),
        )

        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self._yuv_frame_cb,
            h264_cb=self._h264_frame_cb
        )
        
    def _yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded frame.
        :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful informations
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        if random() > 0.75:
            cv2.imwrite(os.path.join(self.finald, "test_picture.png"), cv2frame)

        # cv2frame = self.processing_fun(cv2frame)

        # Use OpenCV to show this frame
        cv2.imshow("Olympe Anafi Streaming", cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...

    def _h264_frame_cb(self, h264_frame):
        """
        More info at:
        https://developer.parrot.com/docs/olympe/olympeapi.html?highlight=set_streaming_callbacks#olympe.Drone.set_streaming_callbacks
        
        This function will be called by Olympe for each new h264 frame.
        :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

        # Do some image processing on the frame
        h264_frame.as_ndarray()  # 1-D non-owning numpy
    
    def start(self):
        """Starts the video streaming"""
        self.drone.connection()
        self.drone.start_video_streaming()

    def stop(self):
        """Stops the video streaming and stop collecting the video metadata"""
        self.drone.stop_video_streaming()
        self.h264_stats_file.close()

    def store(self):
        """
        Saves the video in the given filepath in .mp4 format
        """
        h264_filepath = os.path.join(self.tempd, self.h264_data_fname)
        mp4_filepath = os.path.join(self.finald, self.h264_mp4_fname)

        subprocess.run(
            shlex.split('ffmpeg -i {} -c:v copy {}'.format(
                h264_filepath, mp4_filepath)),
            check=True
        )    

    def open_video(self):
        """Plays the stored video in the default application"""
        subprocess.run(
            shlex.split('xdg-open {}'.format(self.mp4_filepath)),
            check=True
        )

    def open_folder(self):
        """Opens the video folder in the default application"""
        subprocess.run(
            shlex.split('xdg-open {}'.format(self.finald)),
            check=True
        )

# Desired functionality
# stream = StreamProcessing(
#     drone=drone,
#     format_h264=True,
#     save_path="",
#     processing_function=lambda x: x + 1,  # this should publish the results in an async way
# )
# Could use a "store" like in previous project