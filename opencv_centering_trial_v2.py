#!/usr/bin/env python

import cv2 as cv
import gi
import numpy as np
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
	
def nothing(x):
    pass

if __name__ == '__main__':
    
    ccx = rospy.Publisher('cc/x', Float32, queue_size=10) #ccx = camera center x position
    ccy = rospy.Publisher('cc/y', Float32, queue_size=10) #ccy = camera center y position
    tcx = rospy.Publisher('tc/x', Float32, queue_size=10) #tcx = target center x position
    tcy = rospy.Publisher('tc/y', Float32, queue_size=10) #tcy = target center y position
    ctr = rospy.Publisher('ctr', Bool, queue_size=10) #ctr  = centered boolean
    cntr = rospy.Publisher('cntr', Bool, queue_size=10) #cntr = contour boolean

    rospy.init_node('offb_pid')
    rate = rospy.Rate(50)

    # Add port= if is necessary to use a different one
    video = Video()
    cv.namedWindow('centering')
    cv.resizeWindow('centering',400, 300)

    # 0 0 0 177 245 255 Gedung 1 dan 3
    # 0 0 0 162 245 255 Gedung 1 dan 2
    cv.createTrackbar('hl', 'centering', 0, 179, nothing)
    cv.createTrackbar('sl', 'centering', 0, 255, nothing)
    cv.createTrackbar('vl', 'centering', 0, 255, nothing)
    cv.createTrackbar('hu', 'centering', 177, 179, nothing)
    cv.createTrackbar('su', 'centering', 245, 255, nothing)
    cv.createTrackbar('vu', 'centering', 255, 255, nothing)

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()
        (a, b) = frame.shape[:2]
        frame2 = cv.cvtColor(frame, cv.COLOR_BGR2HSV)	
        h_l = cv.getTrackbarPos('hl', 'centering')	
        s_l = cv.getTrackbarPos('sl', 'centering')	
        v_l = cv.getTrackbarPos('vl', 'centering')	
        h_u = cv.getTrackbarPos('hu', 'centering')	
        s_u = cv.getTrackbarPos('su', 'centering')	
        v_u = cv.getTrackbarPos('vu', 'centering')
        
        mask = cv.inRange(frame2, (h_l, s_l, v_l), (h_u, s_u, v_u))

        frame3 = cv.bitwise_and(frame2, frame2, mask = mask)
        frame4 = cv.cvtColor(frame3, cv.COLOR_BGR2GRAY)
        frame5 = cv.GaussianBlur(frame4, (5,5), cv.BORDER_DEFAULT)
        
        ret2, thresh = cv.threshold(frame5, 0, 255, cv.THRESH_BINARY_INV)
        
        cv.imshow('lol', thresh)

        cv.circle(frame, (int(b/2), int(a/2)), 20, (0, 0, 255), 2)
        #cv.circle(frame, (b/2-20, a/2), 1, (255,0,0), 5)
        #cv.circle(frame, (b/2+20, a/2), 1, (255,0,0), 5)
        #cv.circle(frame, (b/2, a/2-20), 1, (255,0,0), 5)
        #cv.circle(frame, (b/2, a/2+20), 1, (255,0,0), 5)
        hierarchy, contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cntr.publish(False)

        for cnt in contours:
            if cv.contourArea(cnt) > 4000:
                cv.drawContours(frame, cnt, -1, (255, 0, 0), 2)
                x, y, w, h = cv.boundingRect(cnt)
                cv.rectangle(frame, (x, y), (x+w, y+h), (61, 251, 41), 3)
                x1 = x + int(w/2)
                y1 = y + int(h/2)
                cv.circle(frame, (x1, y1), 1, (255, 0, 0), 5)
                # if x1 > int(b/2)-20 and x1 < int(b/2)+20 and y1 > int(a/2)-20 and y1 < int(a/2)+20:
                #     cv.putText(frame, "Centered (" + str(x1) + ", " + str(y1) + ")", (200,50), cv.FONT_HERSHEY_DUPLEX, 1, (0,0,0), 2)
                jarakx = int(x1 - b/2)
                jaraky = int(y1 - a/2)
                jarak = math.sqrt(jarakx**2 + jaraky**2)

                ccx.publish(float(b/2))
                ccy.publish(float(a/2))
                tcx.publish(float(x1))
                tcy.publish(float(y1))
                ctr.publish(False)
                cntr.publish(True)

                if jarak < 20:
                    cv.putText(frame, "Centered (" + str(x1) + ", " + str(y1) + ")", (200,50), cv.FONT_HERSHEY_DUPLEX, 1, (0,0,0), 2)
                    ctr.publish(True)

        cv.imshow('niggs1', frame)

        k = cv.waitKey(1)
        if k == 27:
            break
        rate.sleep()
