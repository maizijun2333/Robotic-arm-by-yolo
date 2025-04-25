import numpy as np
import math
import time
import logging
from math import *

# CRITICAL, ERROR, WARNING, INFO, DEBUG
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

#用户自定义
# 摄像头相对于机械臂云台投影中心的偏移量（单位：cm）
CAMERA_OFFSET_X = 0  # 摄像头左右方向上的偏移（假设为 0）
CAMERA_OFFSET_Y = 4  # 摄像头前后方向上的偏移
CAMERA_OFFSET_Z = 1 # 摄像头在机械臂上方 6 cm

def pixel_to_arm_coords(x1, y1, x2, y2, depth_cm, img_width=1280, img_height=720, focal_length=1030, theta_cam=0):
    """
    将像素坐标转换为机械臂坐标系下的 3D 坐标（单位：cm），采用直角坐标系
    :param x1, y1, x2, y2: 目标框的像素坐标
    :param depth_cm: 目标深度（摄像头到目标的距离，单位：cm）
    :param img_width: 图像宽度（像素）
    :param img_height: 图像高度（像素）
    :param focal_length: 摄像头焦距（像素）
    :param theta_cam: 摄像头俯仰角（单位：度），正值表示向下倾斜
    :return: 机械臂坐标系下的坐标 (X_arm, Y_arm, Z_arm)，单位：cm
    """
    # 计算目标中心点的像素坐标
    x_pixel = (x1 + x2) / 2
    y_pixel = (y1 + y2) / 2

    # 相机主点（光轴交点）
    C_x = img_width / 2
    C_y = img_height / 2

    # 计算摄像头坐标系下的坐标
    X_camera = ((x_pixel - C_x) * depth_cm) / focal_length  # 左右方向（水平向右为正）
    Y_camera = ((y_pixel - C_y) * depth_cm) / focal_length  # 上下方向（向下为正）
    Z_camera = depth_cm  # 摄像头正前方

    # 处理摄像头俯仰角
    if theta_cam != 0:
        theta_rad = np.radians(theta_cam)  # 角度转换为弧度
        # 进行绕 X 轴的旋转变换
        Z_camera_rot = Z_camera * np.cos(theta_rad) - Y_camera * np.sin(theta_rad)
        Y_camera_rot = Z_camera * np.sin(theta_rad) + Y_camera * np.cos(theta_rad)
    else:
        Z_camera_rot = Z_camera
        Y_camera_rot = Y_camera

    # **将摄像头坐标转换到机械臂坐标系**
    X_arm = X_camera + CAMERA_OFFSET_X  # 左右方向（水平向右为正）
    Y_arm = Z_camera_rot + CAMERA_OFFSET_Y  # **前后方向（摄像头前方 = 机械臂前方）**
    if Y_arm >= 20:
        Z_arm = -Y_camera_rot + CAMERA_OFFSET_Z + 3  # **垂直方向（摄像头下方 = 机械臂上方）**

    return X_arm, Y_arm, Z_arm

class IK:
    # 舵机从下往上数
    # 公用参数，即4自由度机械臂的连杆参数
    l1 = 6.10    #机械臂底盘中心到第二个舵机中心轴的距离6.10cm
    l2 = 10.16   #第二个舵机到第三个舵机的距离10.16cm
    l3 = 9.64    #第三个舵机到第四个舵机的距离9.64cm
    l4 = 16.65  #第四个舵机到机械臂末端的距离16.6cm， 机械臂末端是指爪子完全闭合时

    def setLinkLength(self, L1=l1, L2=l2, L3=l3, L4=l4):
        # 更改机械臂的连杆长度，为了适配相同结构不同长度的机械臂
        self.l1 = L1
        self.l2 = L2
        self.l3 = L3
        self.l4 = L4

    def getLinkLength(self):
        # 获取当前设置的连杆长度
        return {"L1":self.l1, "L2":self.l2, "L3":self.l3, "L4":self.l4}

    def getRotationAngle(self, coordinate_data, Alpha):
        # 给定指定坐标和俯仰角，返回每个关节应该旋转的角度，如果无解返回False
        # coordinate_data为夹持器末端坐标，坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        # Alpha为夹持器与水平面的夹角，单位度

        # 设夹持器末端为P(X, Y, Z), 坐标原点为O, 原点为云台中心在地面的投影， P点在地面的投影为P_
        # l1与l2的交点为A, l2与l3的交点为B，l3与l4的交点为C
        # CD与PD垂直，CD与z轴垂直，则俯仰角Alpha为DC与PC的夹角, AE垂直DP_， 且E在DP_上， CF垂直AE，且F在AE上
        # 夹角表示：例如AB和BC的夹角表示为ABC
        X, Y, Z = coordinate_data
        #求底座旋转角度
        theta6 = (180 - math.degrees(math.atan2(Y, X))) % 240
        P_O = sqrt(X*X + Y*Y) #P_到原点O距离
        CD = self.l4 * cos(radians(Alpha))
        PD = self.l4 * sin(radians(Alpha)) #当俯仰角为正时，PD为正，当俯仰角为负时，PD为负
        AF = P_O - CD
        CF = Z - self.l1 - PD
        AC = sqrt(pow(AF, 2) + pow(CF, 2))
        print('P_o:',P_O,'CD:',CD,'PD:',PD,'AF:',AF,'CF:',CF,'AC:',AC)
        if round(CF, 4) < -self.l1:
            print('高度低于0, CF(%s)<l1(%s)', CF, -self.l1)
            return False
        if self.l2 + self.l3 < round(AC, 4): #两边之和小于第三边
            print('不能构成连杆结构, l2(%s) + l3(%s) < AC(%s)', self.l2, self.l3, AC)
            return False

        #求theat4
        cos_ABC = round(-(pow(AC, 2)- pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*self.l3), 4) #余弦定理
        if abs(cos_ABC) > 1:
            print('不能构成连杆结构, abs(cos_ABC(%s)) > 1', cos_ABC)
            return False
        ABC = acos(cos_ABC) #反三角算出弧度
        print(ABC)
        theta4 = 180.0 - degrees(ABC)

        #求theta5
        CAF = acos(AF / AC)
        cos_BAC = round((pow(AC, 2) + pow(self.l2, 2) - pow(self.l3, 2))/(2*self.l2*AC), 4) #余弦定理
        if abs(cos_BAC) > 1:
            print('不能构成连杆结构, abs(cos_BAC(%s)) > 1', cos_BAC)
            return False
        if CF < 0:
            zf_flag = -1
        else:
            zf_flag = 1
        theta5 = degrees(CAF * zf_flag + acos(cos_BAC))

        #求theta3
        theta3 = Alpha - theta5 + theta4

        return {"theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6} # 有解时返回角度字典
        
class ArmIK:
    servo3Range = (0, 1000.0, 0, 240.0) #脉宽， 角度
    servo4Range = (0, 1000.0, 0, 240.0)
    servo5Range = (0, 1000.0, 0, 240.0)
    servo6Range = (0, 1000.0, 0, 240.0)

    def __init__(self):
        self.setServoRange()
        self.ik = IK()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range, servo6_Range=servo6Range):
        # 适配不同的舵机
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        #将逆运动学算出的角度转换为舵机对应的脉宽值
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0])/2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0] + 60:
            logger.info('servo3(%s)超出范围(%s, %s)', servo3, self.servo3Range[0] + 60, self.servo3Range[1])
            return False

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0])/2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            logger.info('servo4(%s)超出范围(%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0])/2 - (90.0 - theta5) * self.servo5Param))
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0])/2 + 90*self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0])/2 - 90*self.servo5Param):
            logger.info('servo5(%s)超出范围(%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False
        
        if theta6 < -(self.servo6Range[3] - self.servo6Range[2])/2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2])/2 - (90 - theta6)) * self.servo6Param))
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            logger.info('servo6(%s)超出范围(%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da = 1):
        #给定坐标coordinate_data和俯仰角的范围alpha1，alpha2, 自动在范围内寻找到的合适的解
        #如果无解返回False,否则返回对应舵机角度,俯仰角
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        #da为俯仰角遍历时每次增加的角度
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):#遍历求解
            result = self.ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha

        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2):
        #给定坐标coordinate_data和俯仰角alpha,以及俯仰角范围的范围alpha1, alpha2，自动寻找最接近给定俯仰角的解，并转到目标位置
        #如果无解返回False,否则返回舵机角度、俯仰角、运行时间
        #坐标单位cm， 以元组形式传入，例如(0, 5, 10)
        #alpha为给定俯仰角
        #alpha1和alpha2为俯仰角的取值范围
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]

        return servos,alpha

# if __name__ == '__main__':
#     ik = IK()   
#     #设置连杆长度
#     l1 = ik.l1 + 0.75
#     l4 = ik.l4 - 0.15
#     ik.setLinkLength(L1=l1, L4=l4)

#     x1, y1, x2, y2 = 126, 73, 1277, 484  # 目标框
#     depth_cm = 15  # 单目测距计算的深度（厘米）
#     X_arm, Y_arm, Z_arm = pixel_to_arm_coords(x1, y1, x2, y2, depth_cm)
#     print("机械臂坐标系下的坐标：", X_arm, Y_arm, Z_arm)
    
#     AK = ArmIK()
#     servos,alpha = AK.setPitchRangeMoving((X_arm, Y_arm, Z_arm), -70, -90, -45)
#     print(servos['servo3'])
#     print(servos)