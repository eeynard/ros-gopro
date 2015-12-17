import unicodedata
import requests

from gopro.msg import Status
from sensor_msgs.msg import Image as ROSImage

# Numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

from gopro_responses import status_matrix
from gopro_responses import command_matrix

# attempt imports for image() function
try:
    import cv2
    from PIL import Image
    import StringIO
    import base64
except ImportError:
    pass


class GoProWrapper:

    def __init__(self, ip, password):
        self.ip = ip
        self.password = password
        self.base_url = 'http://' + self.ip + '/'

    """
    Returns a photo from the live feed of the gopro
    """
    def image(self):
        # use OpenCV to capture a frame and store it in a numpy array
        stream = cv2.VideoCapture('http://' + self.ip + ':8080/live/amba.m3u8')
        success, numpy_image = stream.read()

        if success:
            image = Image.fromarray(numpy_image)

            return ROSImage(data=image.getdata(), width=image.width, heigth=image.height)

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
