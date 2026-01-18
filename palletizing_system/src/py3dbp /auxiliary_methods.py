from decimal import Decimal
from .constants import Axis
import numpy as np

class RPY:
    def __init__(self, angle, threshold=1e-10):
        '''
        Rotation order : x -> y -> z, default threshold = 1e-10
        you can change it like eg)RPY[angle, 2e-15]
        '''
        if angle is None:
            angle = [0, 0, 0]
        self.roll = angle[0]
        self.pitch = angle[1]
        self.yaw = angle[2]
        self.threshold = threshold
        self.quaternion = self.calculate_quaternion()
        self.rotation_matrix = self.calculate_rotation_matrix()

    def calculate_quaternion(self):
        """
        주어진 롤, 피치, 요 각도로 쿼터니언을 계산하는 함수
        :return: 쿼터니언 [w, x, y, z]
        """
        roll_half = self.roll / 2
        pitch_half = self.pitch / 2
        yaw_half = self.yaw / 2
        w = np.cos(roll_half) * np.cos(pitch_half) * np.cos(yaw_half) + np.sin(roll_half) * np.sin(pitch_half) * np.sin(yaw_half)
        x = np.sin(roll_half) * np.cos(pitch_half) * np.cos(yaw_half) - np.cos(roll_half) * np.sin(pitch_half) * np.sin(yaw_half)
        y = np.cos(roll_half) * np.sin(pitch_half) * np.cos(yaw_half) + np.sin(roll_half) * np.cos(pitch_half) * np.sin(yaw_half)
        z = np.cos(roll_half) * np.cos(pitch_half) * np.sin(yaw_half) - np.sin(roll_half) * np.sin(pitch_half) * np.cos(yaw_half)
        return [w, x, y, z]

    def calculate_rotation_matrix(self):
        """
        주어진 롤, 피치, 요 각도로 3차원 회전 변환 행렬을 계산하는 함수
        :return: 회전 변환 행렬
        """
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(self.roll), -np.sin(self.roll)],
                        [0, np.sin(self.roll), np.cos(self.roll)]])

        R_y = np.array([[np.cos(self.pitch), 0, np.sin(self.pitch)],
                        [0, 1, 0],
                        [-np.sin(self.pitch), 0, np.cos(self.pitch)]])

        R_z = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0],
                        [np.sin(self.yaw), np.cos(self.yaw), 0],
                        [0, 0, 1]])
        R = np.dot(R_z, np.dot(R_y, R_x))
        R[np.abs(R) < self.threshold] = 0
        return R


def rectIntersect(item1, item2, x, y):
    d1 = item1.getDimension()
    d2 = item2.getDimension()

    cx1 = item1.position[x] + d1[x]/2
    cy1 = item1.position[y] + d1[y]/2
    cx2 = item2.position[x] + d2[x]/2
    cy2 = item2.position[y] + d2[y]/2

    ix = max(cx1, cx2) - min(cx1, cx2)
    iy = max(cy1, cy2) - min(cy1, cy2)

    return ix < (d1[x]+d2[x])/2 and iy < (d1[y]+d2[y])/2


def intersect(item1, item2):
    return (
        rectIntersect(item1, item2, Axis.WIDTH, Axis.HEIGHT) and
        rectIntersect(item1, item2, Axis.HEIGHT, Axis.DEPTH) and
        rectIntersect(item1, item2, Axis.WIDTH, Axis.DEPTH)
    )


def getLimitNumberOfDecimals(number_of_decimals):
    return Decimal('1.{}'.format('0' * number_of_decimals))


def set2Decimal(value, number_of_decimals=0):
    number_of_decimals = getLimitNumberOfDecimals(number_of_decimals)

    return Decimal(value).quantize(number_of_decimals)
