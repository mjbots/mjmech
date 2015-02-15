#!/usr/bin/env python

# Camera calibration which uses opencv
import cv
import cv2
import numpy
import os
import yaml
import re

DEFAULT_CAL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'cal-20141109.yaml')

class CameraCalibration(object):
    """This class handles camera calibration -- it translates between
    projection point (pixel coordinates on the image) and
    camera-referenced coordinates (X/Z and Y/Z fractions)
    """

    # Default diagonal field-of-view, in degrees
    _DEFAULT_FOV_DIAG = 78.0   # From C920 camera datasheet
    # Camera aspect ratio
    _DEFAULT_ASPECT = (16, 9)

    # camera metadata for describe()
    # For Logitech C920 --> 1/3" 3 MP HD sensor (according to some forum)
    _DEFAULT_IMAGE_SIZE = (1928, 1080)   # pixels
    _DEFAULT_SENSOR_SIZE = (6.0, 4.8)    # mm

    def __init__(self):
        """Create an object. Use default camera matrix"""
        # (x, y) tuple -- size of image used for calibration, pixels
        self.image_size = None
        self.camera_matrix = None
        self.dist_coeffs = None

        self.setup_yaml(DEFAULT_CAL)

    def setup_yaml(self, fname):
        """Load a yaml file produced by opencv calibration procedure"""
        # The file is in FileStorage class, which does not have full python API.
        # Load member-by-member as recommended here:
        # http://stackoverflow.com/questions/11025477/error-loading-opencv-xml-file-with-python
        self.camera_matrix = numpy.asarray(
            cv.Load(fname, cv.CreateMemStorage(), 'camera_matrix'))
        self.dist_coeffs = numpy.asarray(
            cv.Load(fname, cv.CreateMemStorage(), 'distortion_coefficients'))


        with open(fname, 'r') as f:
            data_str = f.read()
        # Clean up invalid directive yaml
        data_str = re.sub('^%.*$', '', data_str, count=1, flags=re.MULTILINE)
        # Remove class tags
        data_str = re.sub(': !!.*$', ':', data_str, flags=re.MULTILINE)
        data = yaml.safe_load(data_str)

        self.image_size = (data['image_width'], data['image_height'])

    def describe(self):
        """Describe calibration info as a string. Use human-understandable
        metrics."""
        image_size = (self.image_size or self._DEFAULT_IMAGE_SIZE)
        aperture = self._DEFAULT_SENSOR_SIZE
        assert self.camera_matrix is not None, 'Calibration not loaded'

        # Center coordinates -- in percent, with (0, 0) being image center
        c_x = (self.camera_matrix[0, 2] *1.0 / image_size[0] - 0.5) * 100.0
        c_y = (self.camera_matrix[1, 2] *1.0 / image_size[1] - 0.5) * 100.0

        fov_x, fov_y, focal_len, principal, aspect = \
            cv2.calibrationMatrixValues(self.camera_matrix, image_size,
                                        aperture[0], aperture[0])

        return ("FOV(deg)=({fov_x:.1f}/{fov_y:.1f}) "
                "principal=({principal[0]:.1f}/{principal[1]:.1f}) "
                "center=({c_x:.1f},{c_y:.1f}) "
                "focal_len={focal_len:.6f} aspect={aspect:.3f}"
                ).format(**locals())

    def to_world2d(self, uv_pos, image_size=None):
        """Given a list of pixel positions (u, v) (correspond to camera_x and
        camera_y), rectify and return world 2d coordinates (x_p=X/Z and
        y_p=Y/Z, assuming origin is at camera center, and Z axis is along
        camera optical axis)
        """
        if image_size is None:
            image_size = self.image_size
        # Scale to image size during calibration
        cam_pts = [
            [[pos_x * self.image_size[0] * 1.0 / image_size[0],
              pos_y * self.image_size[1] * 1.0 / image_size[1]]]
            for (pos_x, pos_y) in uv_pos]
        cam_pts_np = numpy.array(cam_pts, dtype=numpy.float32)
        world_pts = cv2.undistortPoints(cam_pts_np,
                                        self.camera_matrix, self.dist_coeffs)
        #print 'undistort debug:', cam_pts, world_pts
        return [(itm[0][0], itm[0][1]) for itm in world_pts]

    def from_world2d(self, ptlist, image_size=None):
        world_pts = numpy.array(
            [ (x, y, 1) for (x, y) in ptlist ])

        # No rotation or translation
        rvec = tvec = (0, 0, 0)
        img_pts, jacobian = cv2.projectPoints(
            world_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)

        if image_size is None:
            scale_x, scale_y = 1.0, 1.0
        else:
            scale_x = 1.0 * image_size[0] / self.image_size[0]
            scale_y = 1.0 * image_size[1] / self.image_size[1]
        return [(itm[0][0] * scale_x, itm[0][1] * scale_y) for itm in img_pts]

if __name__ == '__main__':
    cc = CameraCalibration()
    import sys
    if len(sys.argv) > 1:
        cc.setup_yaml(sys.argv[1])
    print cc.describe()
    orig = ((0, 0),
            (cc.image_size[0] * 0.5, cc.image_size[1] * 0.5),
            cc.image_size)
    distorted = cc.to_world2d(orig)
    back = cc.from_world2d(distorted)
    print 'transform test:'
    for (orig_i, distorted_i, back_i) in zip(orig, distorted, back):
        print ' ', orig_i, '->', distorted_i, '->', back_i
