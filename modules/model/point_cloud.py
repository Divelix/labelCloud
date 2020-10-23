import ctypes

import numpy as np
import OpenGL.GL as gl

# Get size of float (4 bytes) for VBOs
from modules.control import config_parser

SIZE_OF_FLOAT = ctypes.sizeof(ctypes.c_float)


# Creates an array buffer in a VBO
def create_buffer(attributes):
    bufferdata = (ctypes.c_float * len(attributes))(*attributes)  # float buffer
    buffersize = len(attributes) * SIZE_OF_FLOAT  # buffer size in bytes

    vbo = gl.glGenBuffers(1)  # manufactoring
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, buffersize, bufferdata, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    return vbo


class PointCloud:
    POINT_SIZE = config_parser.get_pointcloud_settings("POINT_SIZE")
    COLOR_FOR_COLORLESS_PCD = config_parser.get_pointcloud_settings("COLORLESS_COLOR")

    def __init__(self, path):
        self.path_to_pointcloud = path
        self.points = None
        self.colors = None
        self.colorless = None
        self.vbo = None
        self.center = (0, 0, 0)
        self.pcd_mins = None
        self.pcd_maxs = None
        self.init_translation = (0, 0, 0)

        # Point cloud transformations
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.trans_x = 0.0
        self.trans_y = 0.0
        self.trans_z = 0.0

    # GETTERS AND SETTERS
    def get_no_of_points(self):
        return len(self.points)

    def get_no_of_colors(self):
        return len(self.colors)

    def get_rotations(self):
        return np.round([self.rot_x, self.rot_y, self.rot_z], 1)

    def get_translations(self):
        return np.round([self.trans_x, self.trans_y, self.trans_z], 4)

    def get_mins_maxs(self):
        return self.pcd_mins, self.pcd_maxs

    def set_mins_maxs(self):
        self.pcd_mins = np.amin(self.points, axis=0)
        self.pcd_maxs = np.amax(self.points, axis=0)
        print("Mins: " + str(self.pcd_mins))
        print("Maxs: " + str(self.pcd_maxs))

    def set_rot_x(self, angle):
        self.rot_x = angle % 360

    def set_rot_y(self, angle):
        self.rot_y = angle % 360

    def set_rot_z(self, angle):
        self.rot_z = angle % 360

    def set_trans_x(self, val):
        self.trans_x = val

    def set_trans_y(self, val):
        self.trans_y = val

    def set_trans_z(self, val):
        self.trans_z = val

    # MANIPULATORS

    def transform_data(self):
        if self.colorless:
            attributes = self.points
        else:
            # Merge coordinates and colors in alternating order
            attributes = np.concatenate((self.points, self.colors), axis=1)

        return attributes.flatten()  # flatten to single list

    def write_vbo(self):
        v_array = self.transform_data()
        # if bool(gl.glGenBuffers):  # GenBuffer must first be set by OpenGL widget
        self.vbo = create_buffer(v_array)
        print("Wrote VBO!")

    def draw_pointcloud(self):
        gl.glTranslate(self.trans_x, self.trans_y, self.trans_z)  # third, pcd translation

        rot_trans = np.add(self.pcd_mins, (np.subtract(self.pcd_maxs, self.pcd_mins)/2))
        gl.glTranslate(*rot_trans)              # move point cloud back

        gl.glRotate(self.rot_x, 1.0, 0.0, 0.0)
        gl.glRotate(self.rot_y, 0.0, 1.0, 0.0)  # second, pcd rotation
        gl.glRotate(self.rot_z, 0.0, 0.0, 1.0)

        gl.glTranslate(*(rot_trans * -1))       # move point cloud to center for rotation

        gl.glPointSize(PointCloud.POINT_SIZE)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)

        if self.colorless:
            stride = 3 * SIZE_OF_FLOAT  # (12 bytes) : [x, y, z] * sizeof(float)
            gl.glPointSize(1)
            gl.glColor3d(*PointCloud.COLOR_FOR_COLORLESS_PCD)  # IDEA: Color by (height) position
        else:
            stride = 6 * SIZE_OF_FLOAT  # (24 bytes) : [x, y, z, r, g, b] * sizeof(float)

        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, stride, None)

        if not self.colorless:
            gl.glEnableClientState(gl.GL_COLOR_ARRAY)
            offset = 3 * SIZE_OF_FLOAT  # (12 bytes) : the rgb color starts after the 3 coordinates x, y, z
            gl.glColorPointer(3, gl.GL_FLOAT, stride, ctypes.c_void_p(offset))
        gl.glDrawArrays(gl.GL_POINTS, 0, self.get_no_of_points())  # Draw the points

        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        if not self.colorless:
            gl.glDisableClientState(gl.GL_COLOR_ARRAY)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def reset_translation(self):
        self.trans_x, self.trans_y, self.trans_z = self.init_translation

    def print_details(self):
        print("Point Cloud Center:\t%s" % np.round(self.center, 2))
        print("Point Cloud Minimums:\t%s" % np.round(self.pcd_mins, 2))
        print("Point Cloud Maximums:\t%s" % np.round(self.pcd_maxs, 2))
        print("Initial Translation:\t%s" % np.round(self.init_translation, 2))