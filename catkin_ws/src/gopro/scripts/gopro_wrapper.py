import unicodedata
import requests
import rospy
import socket
import time

from lxml.html import parse

from gopro.msg import Status

from gopro_responses import status_matrix

# attempt imports for image() function
import cv2
from cv_bridge import CvBridge, CvBridgeError
import subprocess as sp
from sensor_msgs.msg import Image

class GoProWrapper:

    def __init__(self, ip, password):
        self.ip = ip
        self.password = password
        self.base_url = 'http://' + self.ip + '/'
        self.cv2_bridge = CvBridge()

    """
    Wrapped to do an HTTP request
    """
    def do_http_request(self, url):
        try:
            return requests.get('http://' + self.ip + url, timeout=5)
        except requests.ConnectionError as exception:
            rospy.logerr(exception.message)

        return False


    """
    Launches the preview. keep_alive_preview must be call at least every 2 seconds
    """
    def start_preview(self):
        self.do_http_request('/gp/gpExec?p1=gpStreamA9&c1=restart')

    """
    Sends a packet to the camera to keep alive the preview
    """
    def keep_alive_preview(self):
        sckt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.loginfo('Sending keep alive packet to preview')
        sckt.sendto("_GPHD_:0:0:2:0\n", (self.ip, 8554))

    """
    Returns a photo from the live feed of the gopro with ffmpeg
    """
    def picture_from_preview_ff(self):
        cv2.namedWindow("GoPro",cv2.CV_WINDOW_AUTOSIZE)

        sp.Popen(
            [
                'ffmpeg',
                '-i', 'udp://@' + self.ip + ':8554/',
                'r', '1',
                '-f', 'image2',
                '-vframes', '1',
                '-vcodec', 'mjpeg',
                'captured.jpg',
            ], stdin=sp.PIPE, stdout=sp.PIPE)

        return Image('captured.jpg')

    """
    Returns a photo from the live feed of the gopro with opencv
    """
    def picture_from_preview_cv(self):
        # use OpenCV to capture a frame

        rospy.loginfo('Capturing preview')
        capture = cv2.VideoCapture('udp://@' + self.ip + ':8554/')
        #capture = cv2.VideoCapture(0) # webcam

        if capture.isOpened():
            rospy.loginfo('Capturing is opened')
            success, picture = capture.read()

            capture.release()

            if success:
                return self.cv2_bridge.cv2_to_imgmsg(picture, encoding="passthrough")
            else:
                rospy.logerr('Was not successful retrieving the image')
        else:
            rospy.logerr('Was not able to open the video stream')

    """
    Takes a picture and retrieves it from HTTP
    """
    def picture(self):
        # Sets the resolution to the minimum one
        self.do_http_request('/gp/gpControl/setting/2/13')

        # Mode to photo
        self.do_http_request('/gp/gpControl/command/mode?p=1')

        # Takes the photo
        self.do_http_request('/gp/gpControl/command/shutter?p=1')

        time.sleep(.5)

        # Crawl the web page
        parsed = parse('http://' + self.ip + '/videos/DCIM/100GOPRO/')
        elements = parsed.findall('.//a')

        length = len(elements)

        # Get the last picture added
        for i in reversed(range(length)):

            if elements[i].text_content().find('.JPG') != -1:
                capture = cv2.VideoCapture('http://' + self.ip + '/videos/DCIM/100GOPRO/' + elements[i].text_content())
                
                if capture.isOpened():
                    success, picture = capture.read()
                    capture.release()

                    if success:
                        rospy.loginfo('Now removing ' + elements[i].text_content())
                        self.do_http_request('/gp/gpControl/command/storage/delete?p=/100GOPRO/' + elements[i].text_content())
                        return self.cv2_bridge.cv2_to_imgmsg(picture, encoding="passthrough")
                    else:
                        rospy.logerr('Reading the stream has encountered an error')
                else:
                    rospy.logerr('Was not able to open the VideoCapture')

    """
    Splits by control characters =D
    """
    @staticmethod
    def _split_by_control_characters(val):
        # extract non-control characters
        output = []
        s = ''
        for c in unicode(val):
            if unicodedata.category(c)[0] == 'C':
                if len(s) > 0:
                    # start a new string if we found a control character
                    output.append(str(s))
                    s = ''
            else:
                s += c

        # clean up any left over string
        if len(s) > 0:
            output.append(str(s))

        # return extracts strings
        return output

    """
    Create the status of the camera
    """
    def status(self):
        status = Status()

        response = self.do_http_request(self.__get_url("/camera/cv"))

        if response is not False:
            response = response.content

            parts = GoProWrapper._split_by_control_characters(response)

            if len(parts) > 0:
                # everything except the first two chunks of 'HD4.02.01.02.00'
                status.cv.firmware = '.'.join(parts[0].split('.')[2:])
                status.cv.model = '.'.join(parts[0].split('.')[0:2])
                status.cv.name = parts[1]

            for command in status_matrix:
                response = self.do_http_request(self.__get_url('/' + command)).content.encode('hex')
                commandParts = command.split('/')

                for item in status_matrix[command]:
                    args = status_matrix[command][item]

                    if 'first' in args and 'second' in args:
                        part = response[args['first']:args['second']]
                    else:
                        part = response

                    o = getattr(status, commandParts[1])
                    value = part

                    if 'translate' in args:
                        value = self.__translate(args['translate'], part)

                    setattr(o, item, value)

            return status
        return False

    """
        Enables or disables the camera
    """
    def switch(self, on=True):
        value = "1"
        if on is False:
            value = "0"

        return self.do_http_request('/gp/gpControl/setting/10/' + value)

    """
    Constructs an URL with the given command and value
    """
    def __get_url(self, command, value=None):
        url = command + '?t=' + self.password

        if value is not None:
            url += '?p=%' + value

        return url

    """
    Transforms a hexa to a decimal
    """
    def hex_to_dec(self, val):
        return int(val, 16)

    """
    Transforms a data received from the GoPro to a convenient value
    """
    def __translate(self, config, value):
        if isinstance(config, dict):
            # use a lookup dictionary
            if value in config:
                return config[value]
            else:
                return 'translate error: {} not found'.format(value)
        else:
            # use an internal function
            if hasattr(self, config):
                return getattr(self, config)(value)
            else:
                return 'translate error: {} not a function'.format(config)
