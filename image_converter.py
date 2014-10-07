__author__ = 'rbtying'
try:
    from cStringIO import StringIO, InputType, OutputType
    from StringIO import StringIO as pyStringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, (InputType, OutputType, pyStringIO))
except ImportError:
    from StringIO import StringIO

    def _checkIsStringIO(obj):
        return isinstance(obj, StringIO)

import pygame

# ROS specific imports
import sensor_msgs.msg


class ImageConverter(object):
    """
    Convert images/compressedimages to and from ROS
    """

    _ENCODINGMAP_PY_TO_ROS = {'L': 'mono8', 'RGB': 'rgb8',
                              'RGBA': 'rgba8', 'YCbCr': 'yuv422'}
    _ENCODINGMAP_ROS_TO_PY = {'mono8': 'L', 'rgb8': 'RGB',
                              'rgba8': 'RGBA', 'yuv422': 'YCbCr'}
    _PIL_MODE_CHANNELS = {'L': 1, 'RGB': 3, 'RGBA': 4, 'YCbCr': 3}

    @staticmethod
    def to_ros(img):
        """
        Convert a PIL/pygame image to a ROS compatible message (sensor_msgs.Image).
        """

        # Everything ok, convert PIL.Image to ROS and return it
        if img.mode == 'P':
            img = img.convert('RGB')

        rosimage = sensor_msgs.msg.Image()
        rosimage.encoding = ImageConverter._ENCODINGMAP_PY_TO_ROS[img.mode]
        (rosimage.width, rosimage.height) = img.size
        rosimage.step = (ImageConverter._PIL_MODE_CHANNELS[img.mode]
                         * rosimage.width)
        rosimage.data = img.tostring()
        return rosimage

    @classmethod
    def from_ros(cls, rosMsg):
        """
        Converts a ROS sensor_msgs.Image or sensor_msgs.CompressedImage to a pygame Surface
        :param rosMsg: The message to convert
        :return: an alpha-converted pygame Surface
        """
        pyimg = None
        if isinstance(rosMsg, sensor_msgs.msg.Image):
            pyimg = pygame.image.fromstring(rosMsg.data, (rosMsg.width, rosMsg.height),
                                            cls._ENCODINGMAP_ROS_TO_PY[rosMsg.encoding])
        elif isinstance(rosMsg, sensor_msgs.msg.CompressedImage):
            pyimg = pygame.image.load(StringIO(rosMsg.data))

        if not pyimg:
            raise TypeError('rosMsg is not an Image or CompressedImage!')

        return pyimg.convert_alpha()