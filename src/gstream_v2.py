#!/usr/bin/env python3

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstVideo', '1.0')
gi.require_version('GdkPixbuf', '2.0')
from gi.repository import Gst, GLib, GObject, GdkPixbuf

import cv2
import numpy as np
import time
import math
import rospy
import signal
from geometry_msgs.msg import Twist

Gst.init(None)

def create_pixbuf_from_cv2_image(image):
    height, width, channels = image.shape
    stride = width * channels
    return GdkPixbuf.Pixbuf.new_from_data(
        image.tobytes(), GdkPixbuf.Colorspace.RGB, True, 8, width, height, stride
    )

class VideoOverlay:
    def __init__(self, rtsp_url, second_video_url):
        rospy.init_node("gstream", anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.rtsp_url = rtsp_url
        self.second_video_url = second_video_url

        self.pipeline = Gst.parse_launch(f"""
            videomixer name=mix ! videoconvert ! autovideosink sync=false
            rtspsrc location={self.rtsp_url} latency=0 buffer-mode=auto ! queue ! decodebin ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! mix.sink_0
            rtspsrc location={self.second_video_url} latency=0 buffer-mode=auto ! queue ! decodebin ! videoconvert ! videoscale ! video/x-raw,width=320,height=240 ! mix.sink_1
            gdkpixbufoverlay name=overlay_time_date ! gdkpixbufoverlay name=overlay_lines ! mix.sink_2
        """)

        self.overlay_time_date = self.pipeline.get_by_name("overlay_time_date")
        self.overlay_lines = self.pipeline.get_by_name("overlay_lines")
        self.loop = GLib.MainLoop()

        self.length = 359    # pixels

        self.theta1 = 0
        self.theta2 = 0
        self.theta  = 0
        self.wb = 200
        self.tw = 160

        self.start_point1 = (159, 359)
        self.start_point2 = (319, 359)

        self.end_point1 = (159, 0)
        self.end_point2 = (319, 0)

        self.r1 = 0
        self.r2 = 0
        self.r = 0
        self.running = True  # Add this line to initialize self.running
        self.update_overlays()

    def cmd_vel_callback(self, msg):
        self.theta = msg.angular.z

        self.theta1 = math.atan2((self.wb * math.tan(self.theta)) , (self.wb - 0.5 * self.tw * math.tan(self.theta)))
        self.theta2 = math.atan2((self.wb * math.tan(self.theta)) , (self.wb + 0.5 * self.tw * math.tan(self.theta)))

        self.end_point1 = (
            int(self.start_point1[0] + self.length * math.cos(math.pi / 2 + self.theta1)),
            int(self.start_point1[1] - self.length * math.sin(math.pi / 2 + self.theta1)) 
        )
        self.end_point2 = (
            int(self.start_point2[0] + self.length * math.cos(math.pi / 2 + self.theta2)),
            int(self.start_point2[1] - self.length * math.sin(math.pi / 2 + self.theta2)) 
        )

        if  math.tan(self.theta1) == 0 or math.tan(self.theta2) == 0:
            self.r = float('inf')
        else:
            self.r = (self.wb) / math.tan(self.theta)

        if math.sin(self.theta1) == 0:
            self.r1 = float('inf')
        else:
            self.r1 = int(math.sqrt((self.tw/2-self.r)**2 + self.wb**2))

        if math.sin(self.theta2) == 0:
            self.r2 = float('inf')
        else:
            self.r2 = int(math.sqrt((-self.tw/2-self.r)**2 + (self.wb)**2))

        self.update_overlays()

    def create_time_date_image(self):
        image = np.zeros((360, 480, 4), dtype=np.uint8)
        current_time = time.strftime("%H:%M:%S")
        current_date = time.strftime("%d/%m/%Y")
        
        # Draw time and date on the image
        cv2.putText(image, current_time, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0, 255), 2)
        cv2.putText(image, current_date, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0, 255), 2)
        
        return create_pixbuf_from_cv2_image(image)

    def create_lines_image(self):
        image = np.zeros((360, 480, 4), dtype=np.uint8)

        cv2.line(image, self.start_point1, (159, 0), (0, 0, 255, 100), 40)
        cv2.line(image, self.start_point2, (319, 0), (0, 0, 255, 100), 40)

        cv2.line(image, self.start_point1, self.end_point1, (255, 0, 0, 255), 20)
        cv2.line(image, self.start_point2, self.end_point2, (255, 0, 0, 255), 20)
    
        if math.isinf(self.r) and math.isinf(self.r1) and math.isinf(self.r2) :
            cv2.line(image, self.start_point1, (159, 0), (255, 255, 0, 100), 20)
            cv2.line(image, self.start_point2, (319, 0), (255, 255, 0, 100), 20)
        else:
            cv2.ellipse(image, (int(239-self.r), int(self.wb+359)), (int(self.r1), int(self.r1)), 0, 0, 360, (255, 255, 0, 255), 20)      #Left wheel
            cv2.ellipse(image, (int(239-self.r), int(self.wb+359)), (int(self.r2), int(self.r2 )), 0, 0, 360, (255, 255, 0, 255), 20)     #Right wheel            

        pts1 = np.float32([[159,359],[319,359],[159,0],[319,0]])
        pts2 = np.float32([[159,359],[319,359],[199,179],[289,179]])

        M = cv2.getPerspectiveTransform(pts1,pts2)
        image = cv2.warpPerspective(image, M, (480, 360))

        return create_pixbuf_from_cv2_image(image)

    def update_overlays(self):
        if not self.running:
            return False

        pixbuf_time_date = self.create_time_date_image()
        pixbuf_lines = self.create_lines_image()

        self.overlay_time_date.set_property('pixbuf', pixbuf_time_date)
        self.overlay_lines.set_property('pixbuf', pixbuf_lines)
        
        GLib.timeout_add_seconds(1, self.update_overlays)
        return True

    def start(self):
        self.pipeline.set_state(Gst.State.PLAYING)
        GLib.timeout_add_seconds(1, self.update_overlays)
        self.loop.run()

    def stop(self):
        self.running = False
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()

def signal_handler(sig, frame):
    video_overlay.stop()
    rospy.signal_shutdown('Shutting down')
    print("Shutting down...")

if __name__ == "__main__":
    rtsp_url = "rtspt://localhost:8554/my_camera"
    second_video_url = "rtspt://localhost:8554/my_second_camera"
    video_overlay = VideoOverlay(rtsp_url, second_video_url)
    signal.signal(signal.SIGINT, signal_handler)
    try:
        video_overlay.start()
    except KeyboardInterrupt:
        video_overlay.stop()
        rospy.signal_shutdown('Shutting down')
