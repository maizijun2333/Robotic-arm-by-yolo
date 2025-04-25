import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)

#单个舵机控制
def serial_serro_wirte_cmd(id=None,dat1=None, dat2=None):
    buf = bytearray(b'\x55\x55')  # 帧头
    buf.append(8)
    buf.append(3)  # 指令
    # 写数据
    buf.append(1)
    hexnumber = hex(dat2)
    low_hex=hexnumber[-2:]     #int值小于256时十六进制没有高位，手动填充0
    buf.append(int(low_hex,16))
    high_hex=''
    rest = hexnumber[-3::-1]
    for char in rest:
        if char != 'x':
            high_hex += char
        else:
            break
    if high_hex == '':
        buf.append(0)
    else:
        buf.append(int(high_hex,16))
    buf.append(id)
    hexnumber = hex(dat1)
    low_hex = hexnumber[-2:]
    buf.append(int(low_hex,16))
    high_hex = ''
    rest = hexnumber[-3::-1]
    for char in rest:
        if char != 'x':
            high_hex += char
        else:
            break
    if high_hex == '':
        buf.append(0)
    else:
        buf.append(int(high_hex, 16))
    #print(buf)
    #发送
    if ser.isOpen():
        ser.write(buf)
        print(f"{id}action success")
        time.sleep(0.1)
    else:
        print("open failed")

#id:舵机编号 pulse:转动到的位置 use_time:转动时间 该函数防止输入数据超出机械臂转动角度
def setBusServoPulse(id, pulse, use_time):
    pulse = 0 if pulse < 0 else pulse
    pulse = 1000 if pulse > 1000 else pulse
    use_time = 0 if use_time < 0 else use_time
    use_time = 30000 if use_time > 30000 else use_time
    serial_serro_wirte_cmd(id, pulse, use_time)

def setBusDefault():
    setBusServoPulse(1,50,1000)
    setBusServoPulse(2,500,1000)
    setBusServoPulse(3,450,1000)
    setBusServoPulse(4,1000,1000)
    setBusServoPulse(5,700,1000)
    setBusServoPulse(6,500,1000)

#机械臂前伸抓取，6号舵机左转90度，前伸放下
def action(theta3,theta4,theta5,theta6):
    time.sleep(1)    #延时，舵机完成动作需要1000毫秒  机械臂延展长度17cm
    setBusServoPulse(1,50,1000)
    setBusServoPulse(2,100,1000)
    time.sleep(1)
    setBusServoPulse(3,theta3,1000)
    setBusServoPulse(4,theta4,1000)
    setBusServoPulse(5,theta5,1000)
    setBusServoPulse(6,theta6,1000)
    time.sleep(1)
    setBusServoPulse(1,600,1000)
    time.sleep(1)
    setBusServoPulse(2,500,1000)
    setBusServoPulse(3,450,1000)
    setBusServoPulse(4,1000,1000)
    setBusServoPulse(5,700,1000)
    setBusServoPulse(6,500,1000)
    time.sleep(1)
    setBusServoPulse(1,200,1000)

def get_action():
    time.sleep(1)    #延时，舵机完成动作需要1000毫秒  机械臂延展长度17cm
    setBusServoPulse(1,250,1000)
    setBusServoPulse(2,130,1000)
    setBusServoPulse(3,150,1000)
    setBusServoPulse(4,600,1000)
    setBusServoPulse(5,300,1000)
    setBusServoPulse(6,500,1000)
    time.sleep(1)
    setBusServoPulse(1,600,1000)


# 测试语句
#setBusServoPulse(2,900,1000)
#setBusServoPulse(4,500,1000)
#setBusDefault()
#time.sleep(1.5)
#action() 
#get_action()
#time.sleep(1.5)
#setBusDefault()


# ser.close()
