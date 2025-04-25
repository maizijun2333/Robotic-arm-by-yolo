from get_IK import IK,ArmIK,pixel_to_arm_coords
from get_cmxyz import DetectionSystem,_calculate_distance
from arm_send import setBusDefault,setBusServoPulse,action

CLASS_WIDTHS = {
    "ear": 9.0,    # 单位：cm
    "green": 9.0,
    "yellow": 14.0
}

def main():
    detector = DetectionSystem()
    
    # 获取检测结果（自动采集20帧）
    # result = detector.get_average_detections()

    target = input(f"请输入要检测的类别 ({'/'.join(CLASS_WIDTHS.keys())}): ").strip().lower()

    # 执行检测
    result = detector.detect_specific_class(target)

    if result:
        for product, info in result.items():
            print(f"商品类别: {product}")
            print(f"▸ 平均坐标: {info['bbox']}")
            print(f"▸ 平均距离: {info['distance']}cm")

            ik = IK()
            #设置连杆长度
            l1 = ik.l1 + 0.75
            l4 = ik.l4 - 0.15
            ik.setLinkLength(L1=l1, L4=l4)

            x1, y1, x2, y2 = 126, 73, 1277, 484  # 目标框
            depth_cm = _calculate_distance("green",x2-x1)  # 单目测距计算的深度（厘米）
            X_arm, Y_arm, Z_arm = pixel_to_arm_coords(x1, y1, x2, y2, depth_cm)
            #x1, y1, x2, y2 = info['bbox']
            #X_arm, Y_arm, Z_arm = pixel_to_arm_coords(x1, y1, x2, y2, info['distance'])
            print("机械臂坐标系下的坐标：", X_arm, Y_arm, Z_arm)
            
            AK = ArmIK()
            AK.ik = ik
            servos,alpha = AK.setPitchRangeMoving((X_arm, Y_arm, Z_arm), -70, -90, -45)
            print(servos['servo3'])
            print(servos,alpha)
            action(servos['servo3'],servos['servo4'],servos['servo5'],servos['servo6'])

    else:
        print("未检测到有效目标")


if __name__ == '__main__':
    #main()
    x1, y1, x2, y2 = 126, 73, 1277, 484  # 目标框
    depth_cm = _calculate_distance("green",x2-x1)  # 单目测距计算的深度（厘米）
    X_arm, Y_arm, Z_arm = pixel_to_arm_coords(x1, y1, x2, y2, depth_cm)