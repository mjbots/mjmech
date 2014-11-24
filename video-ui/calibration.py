import math
import re
import yaml

from math import degrees, radians

def scale_tuple(data, factor):
    return tuple(x * factor for x in data)

class CameraCalibration(object):
    """This class handles camera calibration -- it translates between
    projection point (pixel coordinates on the image) and
    camera-referenced coordinates (X/Z and Y/Z fractions)
    """

    # Default diagonal field-of-view, in degrees
    _DEFAULT_FOV_DIAG = 78.0   # From C920 camera datasheet
    # Camera aspect ratio
    _DEFAULT_ASPECT = (16, 9)

    def __init__(self):
        """Create an object. Use default camera matrix"""
        # (x, y) tuple -- size of image used for calibration, pixels
        self.image_size = None

        # (c_x, c_y) tuple -- principal point intrinsic paramter, pixels
        # A point at the center of optical axis will be projected to (c_x, c_y)
        self.principal = None

        # (f_x, f_y) tuple -- focal length intrinsic parameter, pixels
        # A point which is 45deg off optical axis will be projected to
        # (c_x + f_x, c_y + f_y)
        self.focal_length = None

        # Distortion coefficiens. None or 4/5/8 numbers
        self.distortion = None

        self.setup_fov(self._DEFAULT_FOV_DIAG)

    def setup_fov(self, fov_diag, aspect_tuple=None):
        """Use information from ideal camera with given field of view
        (degrees) and aspect ratio
        """
        if aspect_tuple is None:
            aspect_tuple = self._DEFAULT_ASPECT
        asp_x, asp_y = aspect_tuple

        # Arbitrary image size
        self.image_size = scale_tuple((asp_x, asp_y), 120.0)
        # Principal at center
        self.principal = scale_tuple(self.image_size, 0.5)

        # Field of view illustration:
        # http://therandomlab.blogspot.com/2013/03/logitech-c920-and-c910-fields-of-view.html

        # half-diagonal in pixels (from image size)
        dy = (self.principal[0]**2 + self.principal[1]**2)**0.5
        # Calculate focal length, assuming it is the same for both coords
        f_len = dy / math.tan(radians(fov_diag) / 2.0)
        self.focal_length = (f_len, f_len)

        # Note: should expect HFOV=70.42 and VFOV=43.30 when DFOV=78.0

        # No distortion
        self.distortion = None

    def setup_yaml(self, fname):
        """Load a yaml file produced by opencv calibration procedure"""
        with open(fname, 'r') as f:
            data_str = f.read()
        # Clean up non-standart/unsafe yaml
        data_str = re.sub('^%.*$', '', data_str, flags=re.MULTILINE)
        data_str = re.sub(': !!.*$', ':', data_str, flags=re.MULTILINE)
        data = yaml.safe_load(data_str)

        self.image_size = (data['image_width'], data['image_height'])

        assert (data['camera_matrix']['rows'] == 3)
        assert (data['camera_matrix']['cols'] == 3)

        cmat = data['camera_matrix']['data']
        self.principal = (cmat[2], cmat[5])
        self.focal_length = (cmat[0], cmat[4])
        assert (cmat[1], cmat[3], cmat[6], cmat[7], cmat[8]) == (0, 0, 0, 0, 1)


        self.distortion = tuple(data['distortion_coefficients']['data'])
        assert len(self.distortion) in [4, 5, 8]
        # NOT IMPLEMENTED
        self.distortion = None

    def to_world2d(self, uv_pos, image_size=None):
        """Given a pixel position (u, v) (correspond to camera_x and camera_y),
        rectify and return world 2d coordinates (x_p=X/Z and y_p=Y/Z, assuming
        origin is at camera center, and Z axis is along camera optical axis)
        """
        if image_size is None:
            # Assume native image size
            u, v = uv_pos
        else:
            # Rescale to proper image size
            u = uv_pos[0] * 1.0 * self.image_size[0] / image_size[0]
            v = uv_pos[1] * 1.0 * self.image_size[1] / image_size[1]

        x_pp = (u * 1.0 - self.principal[0]) / self.focal_length[0]
        y_pp = (v * 1.0 - self.principal[1]) / self.focal_length[1]
        if not self.distortion:
            # No distortion
            return (x_pp, y_pp)

        k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6 = \
            (self.distortion + (0, 0, 0, 0))[:8]

        raise NotImplementedError()

    def describe(self):
        """Describe calibration info as a string. Use human-understandable
        metrics."""
        s_x, s_y = self.image_size
        c_x, c_y = self.principal
        f_x, f_y = self.focal_length

        # Field-of-view in horizontal and vertical directions, degrees.
        # Calculated by adding angle from optical center to two sides of
        # the image.
        # NOTE: when distortions work, the FOV should be calculated by probing
        # image in multiple places along top and bottom axis, and returning
        # min and max vertical FOV -- thus for each direction, we have a range.
        fov_x = degrees(-math.atan(self.to_world2d((0, c_y))[0]) +
                         math.atan(self.to_world2d((s_x, c_y))[0]))

        fov_y = degrees(-math.atan(self.to_world2d((c_x, 0))[1]) +
                        math.atan(self.to_world2d((c_x, s_y))[1]))

        # For diagonal field-of-view, assume principal is in the center and
        # pixels are square (focal lengths are equal)
        s_d = (s_x**2 + s_y**2) ** 0.5
        f_d = ((f_x**2 + f_y**2) / 2) ** 0.5
        fov_d = degrees(2 * math.atan2(s_d / 2.0, f_d))

        # Position of image center, degrees. This will be (0, 0) for ideal
        # camera.
        world2d = self.to_world2d(scale_tuple(self.image_size, 0.5))
        cent_x = degrees(math.atan(world2d[0]))
        cent_y = degrees(math.atan(world2d[1]))

        if self.distortion is None:
            dist_str = 'none'
        else:
            dist_str = '(present)'

        # degree mark
        deg = '_d'

        return ("FOV{deg}=({fov_x:.1f}/{fov_y:.1f})={fov_d:.1f} "
                "center{deg}=({cent_x:.1f}/{cent_y:.1f}) dist={dist_str}"
                ).format(**locals())



if __name__ == '__main__':
    cc = CameraCalibration()
    import sys
    if len(sys.argv) > 1:
        cc.setup_yaml(sys.argv[1])
    print cc.describe()
