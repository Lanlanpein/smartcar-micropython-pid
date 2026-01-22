"""
这版环岛和速度都可以稳定通过，速度-45，
"""
from machine import *
from smartcar import *
from display import *
from seekfree import *
from smartcar import ticker
from seekfree import IMU963RX
import gc
import time
import math

#########################硬件初始化模块################################
#上电测试
beep    = Pin('D24', Pin.OUT, pull = Pin.PULL_UP_47K, value = False)
led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)
state2  = switch2.value()
#手动跳出中断
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
# 选择学习板上的一号拨码开关作为启动选择开关
boot_select = Pin('C18', Pin.IN, pull=Pin.PULL_UP_47K, value = True)

#无线串口模块初始化
wireless = WIRELESS_UART(460800)

#有线串口模块初始化
uart1 = UART(5)
uart1.init(115200)
uart1.write("Test.\r\n")

#CCD初始化
ccd = TSL1401(10)
ccd.set_resolution(TSL1401.RES_12BIT)

#电机初始化
motor_right = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty = 0, invert = False)
motor_left  = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7, 13000, duty = 0, invert = False)

#编码器初始化
encoder_left1 = encoder("D13","D14")
encoder_right1 = encoder("D15","D16",True)

#toF初始化
# tof = DL1B()

#IMU963初始化
imu =  IMU963RX()

#按键初始化
key = KEY_HANDLER(10)

time.sleep_ms(100)


#**************************全局变量定义区*******************************
speed_errorInt = 0
speed_error1 = 0
speed_error = 0
speed_control = 0
angle_error1 = 0
angle_error = 0
angle_errorInt = 0
angle_control = 0
angular_errorInt = 0
angular_error1 = 0
angular_error = 0
angular_control = 0
location = 0
location_jifen = 0
location_last = 0
turn_output = 0
speed1_last = 0.0
speed2_last = 0.0
PI = 3.1415926
alpha = 0.7
gyro_x = 0
gyro_y = 0
gyro_z = 0
acc_x = 0
acc_y = 0
acc_z = 0
q0 = 1
q1 = 0
q2 = 0
q3 = 0   
exInt = 0
eyInt = 0
ezInt = 0
Kp = 15.5 #比例增益控制加速度计
Ki = 0.006 #积分增益控制陀螺偏差的收敛速度
halfT = 0.005 #采样周期的一半
exInt = 0
eyInt = 0
ezInt = 0
pitch = 0.00
roll = 0.00
yaw = 0.00
data_wave = [0,0,0,0,0,0,0,0]
start_zhili = 0
speed_left = 0
speed_right = 0
angle_left = 0
angle_right = 0
angular_left = 0
angular_right = 0
duty_left = 0
duty_right = 0
motor_left_control = 0
motor_right_control = 0
turn_control = 0
ccd_zhongzhi = 0

#CCD摄像头相关
shreshlod = 0
ccd_data1 = [0] * 128        
ccd_data2 = [0] * 128        
ccd_data1 = ccd.get(0)
ccd_data2 = ccd.get(1)
road_width = 50
left_bound = 0
right_bound = 0
podao_l = 2000
podao_r = 2000
kaishi = 0
begin = 0
podao_State = 0

#与环岛相关
yaw_jifen = 0
tuichu_time = 0
allow_annual_time = 0
annual_dir = 0
annual_state = 0
annual_Detect_flag = 0
annual_yaw_angle = 0
annual_flag = 0
ccd1_road_len = 60 #宽度待测
ccd2_road_len = 40 #宽度待测
annual_delay=0    
ccd1_mid=64       #理论中线
ccd2_mid=60       #理论中线
annual_all_error1=0
annual_ave_error1=0
annual_all_error2=0
annual_ave_error2=0
annual_all_tips=0
ccd1_IllegalEdge=8   #丢线边界待测
ccd2_IllegalEdge=8   #丢线边界待测
ccd1_center=0         #真实中线
ccd2_center=0         #真实中线
ccd1_right_index_last=0
speed_type=0
Stop_delay_time=0

road_width_current = 0
left_bound_last = 0
right_bound_last = 0

road_width = 0
threshold_value1 = 0
bound_l_1 = 0                  # 当前左边界
bound_r_1 = 0                 # 当前右边界
ccd_middle_point1 = 0          # 当前中点
ccd_middle_point1_last = 0     # 上次中点

threshold_value2 = 0
bound_l_2 = 0                 # 当前左边界
bound_r_2 = 0                 # 当前右边界
ccd_middle_point2 = 0          # 当前中点
ccd_middle_point2_last = 0     # 上次中点

b = 1.0  # 光电传感器增益系数
start = 0

circle = 0

#目标速度和目标角度
target_speed =  -50# 目标速度
target_speed1 = 0
target_angle =  51.5# -91.5平衡角度

YJ = 0
#PID控制相关
# Kp_angular = 2000
# Ki_angular = 170
# Kd_angular = 150
# 
# Kp_angle = -0.070
# Ki_angle = -0.000001
# Kd_angle = -0.048
# 
# Kp_speed = -0.015
# Ki_speed = -0.0001
# Kd_speed = 0
# 
# Kp_turn = -4.8
# Kp_turn2 = -1.15
# Ki_turn = 0
# Kd_turn = 0
# Kd_turn2 = 2.3
# 
# state = 0
# b = 1.1
# start = 0

# 缺电
# Kp_angular = 1380
# Ki_angular = 210
# Kd_angular = 71
# 
# Kp_angle = -0.042
# Ki_angle = -0.000001
# Kd_angle = -0.031
# 
# Kp_speed = 0.020
# Ki_speed = 0
# Kd_speed = 0

# 满电
Kp_angular = 1200
Ki_angular = 200
Kd_angular = 166

Kp_angle = -0.052
Ki_angle = -0.000001
Kd_angle = -0.083

Kp_speed = 0.163
Ki_speed = 0
Kd_speed = 0

# Kp_turn = -0
# Kp_turn2 = 0
# Ki_turn = 0
# Kd_turn = 0
# Kd_turn2 = 0

Kp_turn = -60
Kp_turn2 = -10
Ki_turn = 0
Kd_turn = 0.001
Kd_turn2 = -4.5
state = 0
b = 1.1
start = 0

# 状态枚举
class State:
    STRAIGHT = 0
    CROSSROAD = 1  # 只靠左边线行驶
    RIGHT_LINE = 2  # 只靠右边线行驶
    
state = State.STRAIGHT

#************************CCD寻迹、十字路口、环岛********************************
# 提取边界点和中点(差比和法)
def ccd1_get_boundary(ccd_data):
    global threshold_value1
    global bound_l_1                  # 当前左边界
    global bound_r_1                  # 当前右边界
    global ccd_middle_point1          # 当前中点
    global ccd_middle_point1_last     # 上次中点
    global state
    global start

    
    # 计算动态阈值
    left_bound_last = 0
    right_bound_last = 0
    ccd_middle_point1_last = int(ccd_middle_point1)
    left_bound_last = bound_l_1
    right_bound_last = bound_r_1
    threshold_value1 = 13 #楼下：13
    
    ccd_middle_point1_last = int(ccd_middle_point1)
    
    # 搜索左边界点，以赛道中点64作为起搜点（不搜左边界的5个点）
    for i in range(64, 4, -1):
        
        # 计算"差比和"值（上升沿（差比和值为-）是左边界，下降沿（差比和值为+）是右边界）
        value = ((ccd_data[i] - ccd_data[i + 4]) * 100 / (ccd_data[i] + ccd_data[i + 4] + 1))

        # 将"差比和"值和动态阈值比较，得到边界值（左右边界都搜）
        if abs(value) >= threshold_value1 and value > 0:
            bound_r_1 = i
        if abs(value) >= threshold_value1 and value < 0:
            bound_l_1 = i
            break
                
        if i == 5:                # 如果找到1都没找到
            # Beep_on()
            bound_l_1 = 0         # 强制令左边界点为0
            break 
        
    # 搜索右边界点，以赛道中点64作为起搜点(不搜右边界的5个点）
    for i in range(64, 118, 1):
        
        # 计算"差比和"值(上升沿（差比和值为-）是左边界，下降沿（差比和值为+）是右边界）
        value = ((ccd_data[i] - ccd_data[i + 4]) * 100 / (ccd_data[i] + ccd_data[i + 4] + 1))
        
        # 将"差比和"值和动态阈值比较，得到边界值（左右边界都搜）
        if abs(value) >= threshold_value1 and value < 0:
            bound_l_1 = i
        if abs(value) >= threshold_value1 and value > 0:
            bound_r_1 = i
            break
                
        if i == 123:              # 如果找到117都没找到
            # Beep_on()
            bound_r_1 = 127       # 强制令右边界点为127
            break 
        
    # 丢线时补线：丢线的时候（有边界值无法找到，读数为0），根据赛道宽度补出边界值，从而计算出丢线情况下的中点值
    
    if bound_l_1 == 0 and bound_r_1 != 127:     # 左丢线，补上左边界点
        bound_l_1 = bound_r_1 - road_width
    if bound_l_1 != 0 and bound_r_1 == 127:     # 右丢线，补上右边界点
        bound_r_1 = bound_l_1 + road_width
    if (bound_l_1 == 0 and bound_r_1 == 127):     # 左右全丢线，补上两边边界点（都写为设定赛道中点值，这样算的的中点值即设定中点值）
        bound_l_1 = 64
        bound_r_1 = 64
    if bound_l_1 >= 90 and bound_r_1 >= 155:   # 巡线异常
        bound_l_1 = bound_l_1 - 90
        bound_r_1 = bound_r_1 - 90

    if (state == State.STRAIGHT) and (bound_l_1 == 0 and bound_r_1 == 127) or ((sum(ccd.get(1))) > 450000):  # 十字路口
#                 print("crossroad enter")
        state = State.CROSSROAD
        start = time.ticks_ms()
#                 print(start)
        bound_l_1 = 32
        bound_r_1 = 96
#         Beep_on()

    elif state == State.CROSSROAD:
        now_time = time.ticks_ms()
        time_error = time.ticks_diff(now_time, start)
#         print(time_error)
        bound_l_1 = 32
        bound_r_1 = 96
        if time_error > 500:
            state = State.STRAIGHT


    # 计算本次中点
    ccd_middle_point1 = int((bound_l_1 + bound_r_1) / 2)
    road_width_current = bound_r_1 - bound_l_1

                 #转向偏差0              实际中线1          上一次中线2              赛道真实宽度3     左边界     4右边界5    上一次左边界6     上一次有边界7    
    return ccd_middle_point1 - 64, ccd_middle_point1,  ccd_middle_point1_last, road_width_current, bound_l_1, bound_r_1, left_bound_last, right_bound_last

def ccd2_get_boundary(ccd_data):
    global threshold_value2
    global bound_l_2                  # 当前左边界
    global bound_r_2                  # 当前右边界
    global ccd_middle_point2          # 当前中点
    global ccd_middle_point2_last     # 上次中点
    
    # 计算动态阈值
    left_bound_last = 0
    right_bound_last = 0
    threshold_value2 = 13 #楼下：13
    ccd_middle_point2_last = int(ccd_middle_point2)
    left_bound_last = bound_l_2
    right_bound_last = bound_r_2
    
    # 搜索左边界点，以赛道中点64作为起搜点(不搜左边界的5个点） 
    for i in range(64, 4, -1):
        
        # 计算"差比和"值（上升沿（差比和值为-）是左边界，下降沿（差比和值为+）是右边界）
        value = ((ccd_data[i] - ccd_data[i + 4]) * 100 / (ccd_data[i] + ccd_data[i + 4] + 1))
        
        # wireless.send_str("{:>6f},{:>6f}\r\n".format(value,i))   # Debug用
        
        # 将"差比和"值和动态阈值比较，得到边界值（左右边界都搜）
        if abs(value) >= threshold_value2 and value > 0:
            bound_r_2 = i
        if abs(value) >= threshold_value2 and value < 0:
            bound_l_2 = i
            break
                
        if i == 5:                # 如果找到1都没找到
            # Beep_on()
            bound_l_2 = 0         # 强制令左边界点为0
            break 
        
    # 搜索右边界点，以赛道中点64作为起搜点(不搜右边界的5个点） 
    for i in range(64, 118, 1):
        
        # 计算"差比和"值(上升沿（差比和值为-）是左边界，下降沿（差比和值为+）是右边界）
        value = ((ccd_data[i] - ccd_data[i + 4]) * 100 / (ccd_data[i] + ccd_data[i + 4] + 1))
        
        # 将"差比和"值和动态阈值比较，得到边界值（左右边界都搜）
        if abs(value) >= threshold_value2 and value < 0:
            bound_l_2 = i
        if abs(value) >= threshold_value2 and value > 0:
            bound_r_2 = i
            break
                
        if i == 123:                # 如果找到122都没找到
            # Beep_on()
            bound_r_2 = 127         # 强制令右边界点为127
            break 
    
    # 计算本次中点
    ccd_middle_point2 = int((bound_l_2 + bound_r_2) / 2)
    road_width_current = bound_r_2 - bound_l_2
    
                 #转向偏差0              实际中线1          上一次中线2              赛道真实宽度3     左边界4      右边界5    上一次左边界6     上一次有边界7    
    return ccd_middle_point2 - 64, ccd_middle_point2,  ccd_middle_point2_last, road_width_current, bound_l_2, bound_r_2, left_bound_last, right_bound_last

#环岛内检测
def huandao_check(ac_ccd1_road_len, ac_ccd2_road_len, ccd1_left_index, ccd1_right_index, ccd2_left_index, ccd2_right_index, annual_state, annual_Detect_flag):
    global annual_flag
    global ccd1_road_len
    global ccd2_road_len
    global annual_delay
    global ccd1_mid
    global ccd2_mid
    global annual_all_error1
    global annual_ave_error1
    global annual_all_error2
    global annual_ave_error2
    global annual_all_tips
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global ccd1_center
    global ccd2_center
    global ccd1_right_index_last
    global speed_type
    global Stop_delay_time
    global yaw_jifen
    global tuichu_time
    global annual_dir
    #左右圆环判断
    annual_Detect_flag, annual_dir = lefthuan(ac_ccd1_road_len, ac_ccd2_road_len, ccd1_left_index, ccd1_right_index, ccd2_left_index, ccd2_right_index, ccd1_left_index_last, annual_state, annual_Detect_flag)
    annual_Detect_flag, annual_dir = righthuan(ac_ccd1_road_len, ac_ccd2_road_len, ccd1_left_index, ccd1_right_index, ccd2_left_index, ccd2_right_index, ccd1_left_index_last, annual_state, annual_Detect_flag)
    #在普通寻迹状态与环岛方向满足时，进入环岛1状态
    if (annual_state == 0) and (annual_dir == 1):
        annual_state = 1
        yaw_jifen = 0
        
    if (annual_state == 0) and (annual_dir == 2):
        annual_state = -1
        yaw_jifen = 0
        
    if annual_state == 1:
        #检测圆环出口
        yaw_jifen += (imu.get()[5] / 700)
        tuichu_time += 1
        if abs(yaw_jifen) > 200:
            #右侧全部丢失
            if ccd1_right_index >= 127 - 2 * ccd1_IllegalEdge and annual_dir == 1:
                annual_Detect_flag = 0
                annual_state = 2
                tuichu_time = 0
#             #左侧全部丢失
#             if tuichu_time > 10000:
#                 annual_state = 0
#                 tuichu_time = 0
#                 annual_dir = 0
#                 yaw_jifen = 0
                
        if abs(yaw_jifen) <= 200 and tuichu_time > 600:
            annual_state = 0
            tuichu_time = 0
            annual_dir = 0
            
                
    if annual_state == -1:
        #检测圆环出口
        tuichu_time += 1
        yaw_jifen += (imu.get()[5] / 700)
        if abs(yaw_jifen) > 200:
            #右侧全部丢失
#             if ccd1_right_index >= 127 - 2 * ccd1_IllegalEdge and annual_dir == 1:
#                 annual_Detect_flag = 0
#                 annual_state = 2
            #左侧全部丢失
            if ccd1_left_index <= 2 * ccd1_IllegalEdge and annual_dir == 2:
                annual_Detect_flag = 0
                annual_state = -2
                tuichu_time = 0
                yaw_jifen = 0
                
#             if tuichu_time > 10000:
#                 annual_state = 0
#                 tuichu_time = 0
#                 annual_dir = 0
#                 yaw_jifen = 0
                
        if abs(yaw_jifen) <= 200 and tuichu_time > 600:
            annual_state = 0
            tuichu_time = 0
            annual_dir = 0
            
                
                
    #出圆环状态
    if annual_state == 2 and annual_dir == 1:  # 按平均误差走
        #赛宽达到了正常值
        if ac_ccd1_road_len <= 70:  # 判断是否画圆到达圆中
            annual_state = 3
#             annual_all_error1 = 0
#             annual_all_error2 = 0
#             annual_all_tips = 0
#             annual_Detect_flag = 0

    #出圆环状态
    if annual_state == -2 and annual_dir == 2:  # 按平均误差走
        #赛宽达到了正常值
        if ac_ccd1_road_len <= 70:  # 判断是否画圆到达圆中
            annual_state = -3


    #赛宽达到正常值，完美闭环
    if annual_state == 3 and annual_dir == 1:
        tuichu_time += 1
        if tuichu_time > 500:
            annual_state = 0
            tuichu_time = 0
            annual_dir = 0
    
        #赛宽达到正常值，完美闭环
    if annual_state == -3 and annual_dir == 2:
        tuichu_time += 1
        if tuichu_time > 500:
            annual_state = 0
            tuichu_time = 0
            annual_dir = 0
#         if ac_ccd1_road_len < 70 and ac_ccd2_road_len < 50:
#             annual_dir = 0
#             annual_state = 0
#             annual_yaw_angle = 0
#             annual_delay = 0
#             annual_state = 0
    annual_flag = annual_state            
    return annual_flag, annual_state, annual_dir, annual_Detect_flag        
                
#左圆环判断：
def lefthuan(ac_ccd1_road_len, ac_ccd2_road_len, ccd1_left_index, ccd1_right_index, ccd2_left_index, ccd2_right_index, ccd1_left_index_last, annual_state, annual_Detect_flag):
    global annual_flag
    global annual_dir
    global ccd1_road_len
    global ccd2_road_len
    global ccd1_mid
    global ccd2_mid
    global annual_all_error1
    global annual_ave_error1
    global annual_all_error2
    global annual_ave_error2
    global annual_all_tips
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global annual_debug
    global ccd1_center
    global ccd2_center
    global allow_annual_time
    global tuichu_time

    ccd1_half_road_len = ccd1_road_len / 2
    ccd2_half_road_len = ccd2_road_len / 2
    
    if annual_Detect_flag == 0:
        #实际赛宽小于预设赛宽or近端/远端中心与预设中心相差大于10（与进环岛条件相反，退出机制）
        if annual_state > 0 or (ac_ccd1_road_len < ccd1_road_len + 15) or abs(ccd2_center - ccd2_mid) > 10 or abs((ccd1_center - ccd1_mid) > 10):
            annual_Detect_flag = 0
            allow_annual_time = 0
            #如果满足，直接退出函数
            return annual_Detect_flag, annual_dir
#     and ccd1_right_index > (127 - 2 * ccd1_IllegalEdge)
    #    远近两个摄像头赛道宽度都远大于理论赛道宽度，环岛状态都是0，左边缺线，右线也右移动      
    if (ac_ccd2_road_len > ccd2_road_len + 9 and annual_state == 0 and annual_Detect_flag == 0 and ac_ccd1_road_len > ccd1_road_len + 14
        and ccd1_left_index <= 2 * 12 and ccd1_right_index > 70 and YJ < 5):
        #左边界丢失
        if ccd1_left_index <= 2 * 10:
            annual_Detect_flag = 1
    
    if annual_Detect_flag == 1 and annual_flag == 0:
        #当前左边界和上一个左边界做差（左边界逐渐增大）
        tuichu_time += 1
        if ac_ccd1_road_len < 70:
            annual_Detect_flag = 3
            tuichu_time = 0
        
#         if (ccd1_left_index - ccd1_left_index_last >= 1 and annual_state == 0 and ccd1_left_index > ccd1_IllegalEdge):
#             allow_annual_time += 1
#         if (allow_annual_time > 5):
#             allow_annual_time = 0
#             annual_Detect_flag = 2
        if  tuichu_time > 250:
            tuichu_time = 0
            annual_Detect_flag = 0
#     if (annual_Detect_flag == 2 and annual_flag == 0):
#         tuichu_time += 1
#         #当前左边界和上一个左边界做差（左边界逐渐减小）
#         if (ccd1_left_index - ccd1_left_index_last <= -1 and annual_state == 0 ):
#             allow_annual_time += 1
#         if (allow_annual_time > 5):
#             allow_annual_time = 0
#             annual_Detect_flag = 3
#         if  tuichu_time > 250:
#             tuichu_time = 0
#             annual_Detect_flag = 0

    if annual_Detect_flag == 3:
        allow_annual_time += 1
        if (ccd1_left_index < 2 * 8):
            annual_dir = 1
        if allow_annual_time > 250:
            annual_Detect_flag = 0
            allow_annual_time = 0
    return annual_Detect_flag, annual_dir

#右圆环判断：
def righthuan(ac_ccd1_road_len, ac_ccd2_road_len, ccd1_left_index, ccd1_right_index, ccd2_left_index, ccd2_right_index, ccd1_left_index_last, annual_state, annual_Detect_flag):
    global annual_flag
    global annual_dir
    global ccd1_road_len
    global ccd2_road_len
    global ccd1_mid
    global ccd2_mid
    global annual_all_error1
    global annual_ave_error1
    global annual_all_error2
    global annual_ave_error2
    global annual_all_tips
    global ccd1_IllegalEdge
    global ccd2_IllegalEdge
    global annual_debug
    global ccd1_center
    global ccd2_center
    global allow_annual_time
    global tuichu_time

    ccd1_half_road_len = ccd1_road_len / 2
    ccd2_half_road_len = ccd2_road_len / 2
    
    if annual_Detect_flag == 0:
        #实际赛宽小于预设赛宽or近端/远端中心与预设中心相差大于10（与进环岛条件相反，退出机制）
        if annual_state > 0 or (ac_ccd1_road_len < ccd1_road_len + 15) or abs(ccd2_center - ccd2_mid) > 10 or abs((ccd1_center - ccd1_mid) > 10):
            annual_Detect_flag = 0
            allow_annual_time = 0
            #如果满足，直接退出函数
            return annual_Detect_flag, annual_dir
#     and ccd1_left_index < (2 * ccd1_IllegalEdge)
    # 实际赛宽大与预设且环岛状态为0，检测状态为0
    if (ac_ccd2_road_len > ccd2_road_len + 10 and annual_state == 0 and annual_Detect_flag == 0 and ac_ccd1_road_len > ccd1_road_len + 15
        and ccd1_right_index >= (127 - (36)) and ccd1_left_index < 35 and YJ < 6):
        #左边界丢失
        if ccd1_right_index >= (127 - (4 * 9)):
            annual_Detect_flag = -1
    
    if annual_Detect_flag == -1 and annual_flag == 0:
        #当前左边界和上一个左边界做差（左边界逐渐增大）
        tuichu_time += 1
        if ac_ccd1_road_len < 70:
            annual_Detect_flag = -3
            tuichu_time = 0
        
#         if (ccd1_left_index - ccd1_left_index_last >= 1 and annual_state == 0 and ccd1_left_index > ccd1_IllegalEdge):
#             allow_annual_time += 1
#         if (allow_annual_time > 5):
#             allow_annual_time = 0
#             annual_Detect_flag = 2
        if  tuichu_time > 250:
            tuichu_time = 0
            annual_Detect_flag = 0
#     if (annual_Detect_flag == 2 and annual_flag == 0):
#         tuichu_time += 1
#         #当前左边界和上一个左边界做差（左边界逐渐减小）
#         if (ccd1_left_index - ccd1_left_index_last <= -1 and annual_state == 0 ):
#             allow_annual_time += 1
#         if (allow_annual_time > 5):
#             allow_annual_time = 0
#             annual_Detect_flag = 3
#         if  tuichu_time > 250:
#             tuichu_time = 0
#             annual_Detect_flag = 0

    if annual_Detect_flag == -3:
        allow_annual_time += 1
        if (ccd1_right_index > 127 - (3 * ccd1_IllegalEdge)):
            annual_dir = 2
        if allow_annual_time > 250:
            annual_Detect_flag = 0
            allow_annual_time = 0
    return annual_Detect_flag, annual_dir
#**********************编码器、速度限幅、蜂鸣器、OPenart串口**************************
#速度限幅
def motor_input(speed_l,speed_r):
    if speed_l >= 4000:
        speed_l = 4000
    if speed_l <= -4000:
        speed_l = -4000
    if speed_r >= 4000:
        speed_r = 4000
    if speed_r <= -4000:
        speed_r = -4000
    # 输出PWM
    motor_left.duty(speed_l)
    motor_right.duty(speed_r)
    
#蜂鸣器
def Beep_on():
    beep.high()  # 高电平响
    time.sleep_ms(10)  # 响一段时间
    beep.low()  # 低电平停止

#编码器滤波
def get_encoder_speed():
    
    global speed1_last
    global speed2_last
    
    speed1 = 0.0
    speed2 = 0.0
    
    #读取本次编码器数据
    speed1 = encoder_left1.get()
    speed2 = encoder_right1.get()
    
    #低通滤波
    alpha = 0.8
    speed1 = speed1 * alpha + speed1_last * (1 - alpha)
    speed2 = speed2 * alpha + speed2_last * (1 - alpha)
    
    # 记录上次编码器数据
    speed1_last = speed1
    speed2_last = speed2
    
    return speed1, speed2
#********************************角度解算********************************
#对原始数据进行滤波
def Imu963ra_Init():
    Filter_data = [0,0,0]
    imu_data = imu.get()

    # 循环读取 100 次传感器数据，用于计算滤波的初始值
    for i in range(0, 50):
        Filter_data[0] += imu_data[3]
        Filter_data[1] += imu_data[4]
        Filter_data[2] += imu_data[5]

    # 计算并设置滤波的初始值
    Filter_data[0] = float(Filter_data[0] / 50)
    Filter_data[1] = float(Filter_data[0] / 50)
    Filter_data[2] = float(Filter_data[0] / 50)
    after_data = [Filter_data[0], Filter_data[1], Filter_data[2]]
    return after_data   #这里得到的是滤波后的加速度计的值

#加速度计和陀螺仪的6个数据进行标准化量纲
def xin_data():
    global gyro_x
    global gyro_y
    global gyro_z
    global acc_x
    global acc_y
    global acc_z
    imu_data = imu.get()
    gyro_data = Imu963ra_Init()
    gyro_x = round(gyro_data[0], 3) * PI / 180 / 16.4
    gyro_y = round(gyro_data[1], 3) * PI / 180 / 16.4
    gyro_z = round(gyro_data[2], 3) * PI / 180 / 14.4

    # 对加速度数据进行滤波处理
    acc_x = round(((float(imu_data[0]) * alpha) / 4096 + acc_x * (1 - alpha)), 3)
    acc_y = round(((float(imu_data[1]) * alpha) / 4096 + acc_y * (1 - alpha)), 3)
    acc_z = round(((float(imu_data[2]) * alpha) / 4096 + acc_z * (1 - alpha)), 3)
    
    new_data = [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
    return new_data

#四元数解算最终角度
def siyuanshu():
    global q0
    global q1
    global q2
    global q3
    global exInt
    global eyInt
    global ezInt
    global pitch
    global roll
    global yaw

    new_data = xin_data()
    ax = new_data[0]
    ay = new_data[1]
    az = new_data[2]
    gx = new_data[3]
    gy = new_data[4]
    gz = new_data[5]
    
    # 对加速度数据进行归一化处理
    if (ax * ax + ay * ay + az * az) == 0:
        return 0,0,0
    norm = math.sqrt(ax * ax + ay * ay + az * az)

    ax = ax / norm
    ay = ay / norm
    az = az / norm

    # 估计方向的重力
    vx = 2 * (q1 * q3 - q0 * q2)
    vy = 2 * (q0 * q1 + q2 * q3)
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

    # 计算误差的领域和方向传感器测量参考方向之间的交叉乘积的总和
    ex = (ay * vz - az * vy)
    ey = (az * vx - ax * vz)
    ez = (ax * vy - ay * vx)

    # 积分误差比例积分增益
    exInt += ex * Ki
    eyInt += ey * Ki
    ezInt += ez * Ki

    # 调整后的陀螺仪测量
    gx += Kp * ex + exInt
    gy += Kp * ey + eyInt
    gz += Kp * ez + ezInt

    # 整合四元数
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT
    q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT
    q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT
    q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT

    # 对四元数进行归一化处理
    norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 /= norm
    q1 /= norm
    q2 /= norm
    q3 /= norm

    # 根据四元数计算欧拉角 pitch、roll、yaw
    pitch = round(-math.atan2(2 * q2 * q3 + 2 * q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.3, 3)
    roll = round(-math.asin(2 * (q1 * q3 - q0 * q2)) * 57.3, 3)
    if gz > 0.1 or gz < -0.1:
        yaw += gz

    return pitch, roll, yaw

# 定义全局变量存储最近10个yaw的绝对值
yaw_buffer = []

def yaw_abs_sum(new_yaw):
    global yaw_buffer  # 使用引用全局变量
    # 添加新数据的绝对值
    yaw_buffer.append(abs(new_yaw))
    # 保持列表长度不超过10，删除最老数据
    if len(yaw_buffer) > 100:
        yaw_buffer.pop(0)
    
    # 计算绝对值总和
    return sum(yaw_buffer[0:50])    
# *******************************PID控制区域******************************
#速度环
def speed_PID():
    global speed_errorInt  #速度误差积分
    global speed_error1    #上一次的误差
    global speed_error     #当前时刻误差
    global speed_control   #速度环输出结果
    
    rl = get_encoder_speed()[0]
    rr = get_encoder_speed()[1]
    actual_speed = (rl + rr) / 2   #获取实际速度
    xuangua = imu.get()[2]
    
    speed_error1 = speed_error
    speed_error = -target_speed + actual_speed
    
    if xuangua < 500:
        speed_errorInt = 0
    else:
        speed_errorInt += speed_error1
        if speed_errorInt > 3000:
            speed_errorInt = 3000
        elif speed_errorInt < -3000:
            speed_errorInt = -3000

    speed_control = Kp_speed * speed_error + Ki_speed * speed_errorInt + Kd_speed * (speed_error - speed_error1)
    if xuangua < 500:
        return 0
    elif speed_control > 4000:
        return 4000
    elif speed_control < -4000:
        return -4000
    else:
        return speed_control
    
# 角度环
def angle_PD(speed_control):
    global angle_error1
    global angle_error
    global angle_errorInt
    global angle_control
    
    actual_angle = siyuanshu()[0]
    xuangua = imu.get()[2]
    speed_control = speed_PID()
    angle_error1 = angle_error
    angle_error = speed_control + target_angle - actual_angle
    
    if xuangua < 500:
        angle_errorInt = 0
    else:
        angle_errorInt += angle_error1
        if angle_errorInt > 6000:
            angle_errorInt = 6000
        elif angle_errorInt < -6000:
            angle_errorInt = -6000

    angle_control = Ki_angle * angle_errorInt + Kp_angle * angle_error + Kd_angle * (angle_error - angle_error1)

    if angle_control > 6000:
        return 6000
    elif angle_control < -6000:
        return -6000
    else:
        return angle_control

#角速度环
def gyro_PD(angle_control):
    actual_angular = xin_data()[3]
    xuangua = imu.get()[2]
    
    global angular_errorInt
    global angular_error1
    global angular_error
    global angular_control

    angular_error1 = angular_error
    angular_error = angle_control - actual_angular
    
    if xuangua < 500:
        angular_errorInt = 0
    else:
        angular_errorInt += angular_error1
        if angular_errorInt > 10000:
            angular_errorInt = 10000
        elif angular_errorInt < -10000:
            angular_errorInt = -10000

    angular_control = Kp_angular * angular_error + Ki_angular * angular_errorInt + Kd_angular * (angular_error - angular_error1)
    if xuangua < 500 or xuangua > 4100:
        return 0
    elif angular_control > 3000:
        return 3000
    elif angular_control < -3000:
        return -3000
    else:
        return angular_control    

# 转向环
def turn():
    global location
    global location_jifen
    global location_last
    global turn_output
    gz = imu.get()[5]
    xuangua = imu.get()[2]   
    CCD = ccd1_get_boundary(ccd.get(1))[0]
    CCD2 = ccd1_get_boundary(ccd.get(2))[0]
    
    alpha = 0.5
    CCD3 = CCD * alpha + CCD2 * (1 - alpha)
    
#     if ccd1_get_boundary(ccd.get(0))[3] > 95 or ccd2_get_boundary(ccd.get(1))[3] > 90:
#         CCD3 = 0
#     else:


    if state == State.CROSSROAD:
        CCD3 = CCD * alpha + CCD2 * (1 - alpha)
        
    elif state == State.STRAIGHT:
        #左环岛相关
        if annual_Detect_flag > 0:
            CCD3 = (ccd1_get_boundary(ccd.get(0))[5] - 30) - 64
        if annual_state == 1 or annual_state == 2:
            CCD3 = (ccd1_get_boundary(ccd.get(0))[4] + 20) - 64
        if annual_state == 3:
            CCD3 = (ccd1_get_boundary(ccd.get(0))[5] - 25) - 64
            
        #右环岛相关
        if annual_Detect_flag < 0:
            CCD3 = (ccd1_get_boundary(ccd.get(0))[4] + 35) - 64
        if annual_state == -1 or annual_state == -2:
            CCD3 = (ccd1_get_boundary(ccd.get(0))[5] - 25) - 64
        if annual_state == -3:
            CCD3 = (ccd1_get_boundary(ccd.get(0))[4] + 35) - 64
    
    location_last = location
    location = CCD3
    
    location_jifen += location 
    if location_jifen >= 500:
        location_jifen = 0
    elif location_jifen <= -500:
        location_jifen = 0
    
    turn_output = (location * Kp_turn + abs(location) * location * Kp_turn2 +
                   Ki_turn * location_jifen + (location - location_last) * Kd_turn + gz * Kd_turn2)
    if turn_output > 2500:
        turn_output = 2500
    elif turn_output < -2500:
        turn_output = -2500
    
    if xuangua < 500:
        return 0
    else:
        return turn_output    


# ******************************定时器0,10ms*****************************
ticker_flag_0 = False

def time_pit_0_handler(time):
    global ticker_flag_0  
    ticker_flag_0 = True  

pit0 = ticker(0)
pit0.capture_list(key)
pit0.callback(time_pit_0_handler)
pit0.start(5)

# ******************************定时器1,10ms*****************************
ticker_flag_1 = False

def time_pit_1_handler(time):
    global ticker_flag_1  
    ticker_flag_1 = True 
    
pit1 = ticker(1)
pit1.capture_list(ccd, imu, encoder_left1, encoder_right1)
pit1.callback(time_pit_1_handler)
pit1.start(5) 


# ******************************定时器2,50ms*****************************
ticker_flag_2 = False
def time_pit_2_handler(time):
    global ticker_flag_2 
    ticker_flag_2 = True  

pit2 = ticker(2)
pit2.capture_list()
pit2.callback(time_pit_2_handler)
pit2.start(50) 

# ******************************定时器3,50ms*****************************
ticker_flag_3 = False
def time_pit_3_handler(time):
    global ticker_flag_3  
    ticker_flag_3 = True 

pit3 = ticker(3)
pit3.capture_list()
pit3.callback(time_pit_3_handler)
pit3.start(10)

while True:
    if (ticker_flag_0):  
        ticker_flag_0 = False
        # 翻转 C4 LED 电平（确保进入了循环）
        led.toggle()
        #按键检测是否启动
        key_data = key.get()
        if key_data[0]:
            start_zhili = 1
        if key_data[1]:
            start_zhili = 0
        if key_data[2]:
            annual_state = 0
            annual_dir = 0
            
        # 获取 CCD 数据，处理道路相关信息，如道路中心、左右索引等
        ccd1_error, ccd1_center, ccd1_center_last, ac_ccd1_road_len, ccd1_left_index, ccd1_right_index, ccd1_left_index_last, ccd1_right_index_last = ccd1_get_boundary(ccd.get(0))
        ccd2_error, ccd2_center, ccd2_center_last, ac_ccd2_road_len, ccd2_left_index, ccd2_right_index, ccd2_left_index_last, ccd2_right_index_last = ccd2_get_boundary(ccd.get(1))
        
        annual_flag, annual_state, annual_dir, annual_Detect_flag = huandao_check(ac_ccd1_road_len, ac_ccd2_road_len, ccd1_left_index, ccd1_right_index,
                                                                                    ccd2_left_index, ccd2_right_index, annual_state, annual_Detect_flag)
        YJ = yaw_abs_sum(imu.get()[5] / 700)
    if (ticker_flag_1):
        ticker_flag_1 = False
        
        speed_left    =  speed_PID() 
        speed_right   =  speed_PID()
        angle_left    =  angle_PD(speed_left)
        angle_right   =  angle_PD(speed_right)
        angular_left  =  gyro_PD(angle_left)
        angular_right =  gyro_PD(angle_right)
        duty_left     =  angular_left
        duty_right    =  angular_right
        
        turn_control = turn()
    
        if start_zhili == 1:
            motor_left_control = 1 * duty_left + 1 * turn_control #speed = 100, 1.18  speed = 150, 1.02  speed = 250, 1.02
            motor_right_control = 1.20*duty_right - 1 * turn_control

        if start_zhili == 0: 
            motor_left_control = 0
            motor_right_control = 0
            
#         if ((sum(ccd.get(0))) < 256000):
#             motor_left_control = 0
#             motor_right_control = 0

        motor_input(motor_left_control, motor_right_control)
        
        
    if (ticker_flag_2):  
        ticker_flag_2 = False
        
        wireless.send_ccd_image(WIRELESS_UART.CCD1_2_BUFFER_INDEX)
        #调参数
        data_wave[1] = annual_state
        data_wave[2] = yaw_abs_sum(imu.get()[5] / 700)
#         data_wave[2] = yaw_jifen
        data_wave[0] = annual_Detect_flag
#         data_wave[2] = ac_ccd1_road_len
#         data_wave[3] = tuichu_time
#         data_wave[3] = ac_ccd2_road_len
#         data_wave[4] = ccd1_left_index
#         data_wave[5] = ccd1_right_index
# #         data_wave[4] = ccd2_left_index_last
# #         data_wave[5] = ccd2_right_index_last
#         data_wave[6] = ccd2_left_index
#         data_wave[7] = ccd2_right_index
    

        wireless.send_oscilloscope(
            data_wave[0],data_wave[1],data_wave[2],data_wave[3],
            data_wave[4],data_wave[5],data_wave[6],data_wave[7])
#         Kp_angular = data_wave[0]
#         Ki_angular = data_wave[1]
#         Kd_angular = data_wave[2]
#         Kp_angle = data_wave[3]
#         Ki_angle = data_wave[4]
#         Kd_angle = data_wave[5]
#         Kp_speed   = data_wave[6]
#         target_angle = data_wave[7]
# #          
#         Kp_turn2 = data_wave[0]
#         Kp_turn = data_wave[1]
#         Kd_turn2 = data_wave[2]
#         Kp_speed = data_wave[4]
# #         Ki_speed = data_wave[4]
#         target_angle = data_wave[5]
#         target_speed = data_wave[6]
#         

#         Ki_speed = data_wave[4]
#         target_speed = data_wave[4]
#         target_angle = data_wave[5]
#         b = data_wave[7]

        data_flag = wireless.data_analysis()
        for i in range(0,8):
        # 判断哪个通道有数据更新
            if (data_flag[i]):
                # 数据更新到缓冲
                data_wave[i] = wireless.get_data(i)
# #                 
        
    if (ticker_flag_3): 
        ticker_flag_3 = False
        

    if end_switch.value()==0:
        pit1.stop()
        break
    if switch2.value() != state2:#防止程序写错而无法退出的机制
        print("Test program stop.")
        break
    
    


# 回收内存
gc.collect()





