import unicodedata
import requests
import rospy
import socket

from lxml.html import parse

from gopro.msg import Status
from sensor_msgs.msg import Image as ROSImage

# Numpy and scipy
import numpy as np

from gopro_responses import status_matrix
from gopro_responses import command_matrix

# attempt imports for image() function
import cv2
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image


class GoProWrapper:

    def __init__(self, ip, password):
        self.ip = ip
        self.password = password
        self.base_url = 'http://' + self.ip + '/'

    """
    Launches the preview. keep_alive_preview must be call at least every 2 seconds
    """
    def preview(self):
        requests.get('http://' + self.ip + '/gp/gpExec?p1=gpStreamA9&c1=restart')

    """
    Sends a packet to the camera to keep alive the preview
    """
    def keep_alive_preview(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto("_GPHD_:0:0:2:0\n", (self.ip, 8554))

    def picture(self):
        # Mode to photo
        requests.get('http://' + self.ip + '/gp/gpControl/command/mode?p=1')

        # Takes the photo
        requests.get('http://' + self.ip + '/gp/gpControl/command/shutter?p=1')

        # Crawl the web page
        parsed = parse('http://' + self.ip + '/videos/DCIM/100GOPRO/')
        elements = parsed.findall('.//a')

        length = len(elements)

        rospy.logerr('Size of elements ' + str(length))

        for i in reversed(range(length)):
            rospy.logerr("Range" + str(i))

            rospy.logerr(elements[i].text_content())

            if elements[i].text_content().find('.JPG') != -1:
                cap = cv2.VideoCapture('http://' + self.ip + '/videos/DCIM/100GOPRO/' + elements[i].text_content())
                if cap.isOpened():
                    success, picture = cap.read()
                    cap.release()

                    if success:
                        return CvBridge().cv2_to_imgmsg(picture, encoding="passthrough")
                    else:
                        rospy.logerr('Reading the stream has encountered an error')
                else:
                    rospy.logerr('Was not able to open the VideoCapture')


    """
    Returns a photo from the live feed of the gopro
    """
    def picture_from_preview(self):
        # restart the live feed
        try:
            # use OpenCV to capture a frame and store it in a numpy array
            stream = cv2.VideoCapture(filename='udp://@' + self.ip + ':8554')

            if stream.isOpened():
                success, numpy_image = stream.read()

                stream.release()
                cv2.destroyAllWindows()

                if success:
                    image = Image.fromarray(numpy_image)

                    return ROSImage(data=image.data)
                else:
                    rospy.logerr('Was not successful retrieving the image')
            else:
                    rospy.logerr('Was not able to open the video stream')
        except requests.exceptions.RequestException as exception:
            rospy.logerr(exception.message)


    """
    Splits by control characters =D
    """
    def _split_by_control_characters(self, val):
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

        response = requests.get(self.__get_url("camera/cv")).content
        parts = self._split_by_control_characters(response)

        if len(parts) > 0:
            # everything except the first two chunks of 'HD4.02.01.02.00'
            status.cv.firmware = '.'.join(parts[0].split('.')[2:])
            status.cv.model = '.'.join(parts[0].split('.')[0:2])
            status.cv.name = parts[1]

        for command in status_matrix:
            response = requests.get(self.__get_url(command)).content.encode('hex')
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

    """
        Enables or disables the camera
    """
    def switch(self, on=True):
        value = "01"
        if on is False:
            value = "00"

        return requests.get(self.__get_url("bacpac/PW", value))

    """
    Constructs an URL with the given command and value
    """
    def __get_url(self, command, value=None):
        url = self.base_url + command + '?t=' + self.password

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
