import unicodedata
import requests
from gopro.msg import Status

class GoProWrapper:

    def __init__(self, ip, password):
        self.ip = ip
        self.password = password
        self.base_url = 'http://' + self.ip + '/'

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
        response = requests.get(self.__get_url("camera/cv")).content.encode('hex')
        status = Status()

        parts = self._split_by_control_characters(response.decode('hex'))

        if len(parts) > 0:
            # everything except the first two chunks of 'HD4.02.01.02.00'
            status.cv.firmware = '.'.join(parts[0].split('.')[2:])
            status.cv.model = '.'.join(parts[0].split('.')[0:2])
            status.cv.name = parts[1]
        else:
            return None

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
    Returns a photo
    """
    def photo(self):
        response = requests.get(self.__get_url("camera/CM", "01"))
        return response.text()

    """
    Constructs an URL with the given command and value
    """
    def __get_url(self, command, value=None):
        url = self.base_url + command + '?t=' + self.password

        if value is not None:
            url += '?p=%' + value

        return url
