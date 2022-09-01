import numpy as np
import cv2

table_height = 1

class Camera:
    def __init__(self, cx, cy, focalx, focaly, pix_to_meter):
        self.cx = cx
        self.cy = cy
        self.focalx = focalx
        self.focaly = focaly
        self.pix_to_meter = pix_to_meter

    def convert_from_uvd(self, u, v, d):
        """
        Borrowed from: https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f
        :param self:
        :param u:
        :param v:
        :param d:
        :return:
        """
        d *= self.pix_to_meter
        x_over_z = (self.cx - u) / self.focalx
        y_over_z = (self.cy - v) / self.focaly
        z = d / np.sqrt(1. + x_over_z**2 + y_over_z**2)
        x = x_over_z * z
        y = y_over_z * z
        return x, y, z


def detect_contour(img):
    mask = np.where(img[:, :, 3] < table_height)  # Where the height is above the table
    img_mask = cv2.bitwise_and(img, mask)

def calc_bounding_box(mask, img, cam: Camera):

    img_masked = cv2.bitwise_and(img, mask)
    height = np.max(img_masked) - table_height

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    bounding_rect = cv2.boundingRect(contours[0])

    xyz_rect = []
    for coord in bounding_rect:
        xyz = cam.convert_from_uvd(coord[0], coord[1], height)
        xyz_rect.append(xyz)

    return xyz_rect






