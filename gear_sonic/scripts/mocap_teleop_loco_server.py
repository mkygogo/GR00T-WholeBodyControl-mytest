import time
import math
import threading
import zmq
import os
from dynamixel_sdk import *

# 让 pygame 在无头模式下运行（防止在纯终端环境报错）
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

# 导入项目中已有的 ZMQ 打包工具
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_planner_message, build_command_message

# ================= 1. 舵机与串口配置 =================
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_PRESENT_POSITION   = 132
LEN_PRO_PRESENT_POSITION    = 4
PROTOCOL_VERSION            = 2.0
BAUDRATE                    = 57600
DEVICENAME                  = '/dev/ttyUSB0'  

# 双臂全开：左臂(7-2) + 右臂(14-9)，共 12 个舵机
DXL_IDs = [7, 6, 5, 4, 3, 2, 14, 13, 12, 11, 10, 9] 

# ================= 2. 关节正反向配置 (基于最新双臂校准结果) =================

DIR_MAP = {
     7:  1,  # 左肩 Pitch (前后摆动)
     6: -1,  # 左肩 Roll (侧向平举)
     5: -1,  # 左肩 Yaw (大臂自转)
     4:  1,  # 左肘 (小臂弯曲)
     3:  1,  # 左腕 Roll (手腕旋转)
     2:  1,  # 左腕 Pitch (手腕上下)
    14: -1,  # 右肩 Pitch (前后摆动)
    13:  1,  # 右肩 Roll (侧向平举)
    12: -1,  # 右肩 Yaw (大臂自转)
    11:  1,  # 右肘 (小臂弯曲)
    10:  1,  # 右腕 Roll (手腕旋转)
     9: -1,  # 右腕 Pitch (手腕上下)
}


# ================= 3. 手柄摇杆轴与按键映射 =================
AXIS_LEFT_X  = 0   # 左摇杆 X轴 (控制原地旋转)
AXIS_RIGHT_X = 3   # 右摇杆 X轴 (控制左右平移)
AXIS_RIGHT_Y = 4   # 右摇杆 Y轴 (控制前后移动)
AXIS_LT      = 2   # 左扳机 LT (控制下蹲)

BTN_LB = 4         # 左肩键 (加速)
BTN_RB = 5         # 右肩键 (减速)

JOYSTICK_DEADZONE = 0.15

# ================= 全局状态变量 =================
global_upper_body_pos = [0.0] * 17
global_mode = 0                  
global_movement = [0.0, 0.0, 0.0] 
global_facing = [1.0, 0.0, 0.0]  
global_yaw = 0.0                 
global_speed = -1.0
global_running = True

def map_servo_to_urdf(current_deg, initial_deg, direction=1):
    delta_deg = current_deg - initial_deg
    urdf_deg = delta_deg * direction
    return math.radians(urdf_deg)

# ----------------- 线程 1: 舵机读取 -----------------
def dynamixel_reader_thread():
    global global_upper_body_pos, global_running
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        print(f"[Dynamixel] ❌ 无法打开串口: {DEVICENAME}")
        global_running = False
        return

    for dxl_id in DXL_IDs:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 0)
        groupSyncRead.addParam(dxl_id)

    print("\n" + "="*60)
    print(" ⚠️ 准备标定零点：请把机械臂【垂直向下】自然下垂放置")
    print("="*60 + "\n")
    
    for i in range(3, 0, -1):
        print(f"标定倒计时: {i}...")
        time.sleep(1)
        
    initial_positions_deg = {}
    print("[Dynamixel] 正在抓取零点数据...")
    
    while len(initial_positions_deg) < len(DXL_IDs) and global_running:
        groupSyncRead.txRxPacket()
        for dxl_id in DXL_IDs:
            if dxl_id not in initial_positions_deg and groupSyncRead.isAvailable(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION):
                present_pos = groupSyncRead.getData(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                if present_pos >= 2**31: present_pos -= 2**32
                initial_positions_deg[dxl_id] = (present_pos / 4096.0) * 360.0
        time.sleep(0.05)

    print(f"\n[Dynamixel] ✅ 零点记录完成！\n")

    while global_running:
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            continue 

        positions_deg = {}
        for dxl_id in DXL_IDs:
            if groupSyncRead.isAvailable(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION):
                present_pos = groupSyncRead.getData(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                if present_pos >= 2**31: present_pos -= 2**32
                positions_deg[dxl_id] = (present_pos / 4096.0) * 360.0

        new_pos = [0.0] * 17
        
        # === 核心双臂映射逻辑 ===
        # 左臂
        if 7 in positions_deg: new_pos[3]  = map_servo_to_urdf(positions_deg[7], initial_positions_deg[7], DIR_MAP[7])
        if 6 in positions_deg: new_pos[5]  = map_servo_to_urdf(positions_deg[6], initial_positions_deg[6], DIR_MAP[6])
        if 5 in positions_deg: new_pos[7]  = map_servo_to_urdf(positions_deg[5], initial_positions_deg[5], DIR_MAP[5])
        if 4 in positions_deg: new_pos[9]  = map_servo_to_urdf(positions_deg[4], initial_positions_deg[4], DIR_MAP[4])
        if 3 in positions_deg: new_pos[11] = map_servo_to_urdf(positions_deg[3], initial_positions_deg[3], DIR_MAP[3])
        if 2 in positions_deg: new_pos[13] = map_servo_to_urdf(positions_deg[2], initial_positions_deg[2], DIR_MAP[2])
        # 右臂
        if 14 in positions_deg: new_pos[4]  = map_servo_to_urdf(positions_deg[14], initial_positions_deg[14], DIR_MAP[14])
        if 13 in positions_deg: new_pos[6]  = map_servo_to_urdf(positions_deg[13], initial_positions_deg[13], DIR_MAP[13])
        if 12 in positions_deg: new_pos[8]  = map_servo_to_urdf(positions_deg[12], initial_positions_deg[12], DIR_MAP[12])
        if 11 in positions_deg: new_pos[10] = map_servo_to_urdf(positions_deg[11], initial_positions_deg[11], DIR_MAP[11])
        if 10 in positions_deg: new_pos[12] = map_servo_to_urdf(positions_deg[10], initial_positions_deg[10], DIR_MAP[10])
        if 9  in positions_deg: new_pos[14] = map_servo_to_urdf(positions_deg[9],  initial_positions_deg[9],  DIR_MAP[9]) 
        
        global_upper_body_pos = new_pos
        time.sleep(0.02) 

    groupSyncRead.clearParam()
    portHandler.closePort()

# ----------------- 线程 2: 手柄读取 -----------------
def gamepad_reader_thread():
    global global_mode, global_movement, global_speed, global_facing, global_yaw, global_running

    pygame.init()
    pygame.joystick.init()
    
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[Gamepad] 🎮 已连接手柄: {joystick.get_name()}")
    else:
        print("[Gamepad] ⚠️ 未检测到手柄！底盘将保持静止。")

    dt = 0.02 
    turn_speed_max = 1.5 

    while global_running:
        if joystick:
            pygame.event.pump() 

            lx = joystick.get_axis(AXIS_LEFT_X)
            rx = joystick.get_axis(AXIS_RIGHT_X)
            ry = joystick.get_axis(AXIS_RIGHT_Y)
            lt = joystick.get_axis(AXIS_LT)

            lb_pressed = joystick.get_button(BTN_LB)
            rb_pressed = joystick.get_button(BTN_RB)

            if abs(lx) < JOYSTICK_DEADZONE: lx = 0.0
            if abs(rx) < JOYSTICK_DEADZONE: rx = 0.0
            if abs(ry) < JOYSTICK_DEADZONE: ry = 0.0

            global_yaw += (-lx) * turn_speed_max * dt
            global_facing = [math.cos(global_yaw), math.sin(global_yaw), 0.0]

            if lt > 0.0:  
                global_mode = 4 # IDLE_SQUAT
                global_movement = [0.0, 0.0, 0.0]
                global_speed = -1.0
            else:
                local_move_x = -ry 
                local_move_y = -rx 
                mag = math.hypot(local_move_x, local_move_y)

                if mag > 0.0:
                    global_mode = 2 # WALK

                    gx = local_move_x * math.cos(global_yaw) - local_move_y * math.sin(global_yaw)
                    gy = local_move_x * math.sin(global_yaw) + local_move_y * math.cos(global_yaw)
                    global_movement = [gx, gy, 0.0]

                    base_speed = mag 
                    if lb_pressed:
                        base_speed *= 2.0  
                    elif rb_pressed:
                        base_speed *= 0.5  

                    global_speed = base_speed
                else:
                    global_mode = 0 # IDLE
                    global_movement = [0.0, 0.0, 0.0]
                    global_speed = -1.0

        time.sleep(dt) 

# ----------------- 线程 3: ZMQ 发送 -----------------
def zmq_publisher_thread():
    global global_upper_body_pos, global_mode, global_movement, global_speed, global_facing, global_running
    
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5556")
    time.sleep(0.5)

    try:
        zmq_socket.send(build_command_message(start=True, stop=False, planner=True))
    except Exception as e:
        pass

    print_counter = 0
    while global_running:
        msg = build_planner_message(
            mode=global_mode, 
            movement=global_movement, 
            facing=global_facing, 
            speed=global_speed,
            height=-1.0,
            upper_body_position=global_upper_body_pos 
        )
        zmq_socket.send(msg)
        
        print_counter += 1
        if print_counter % 10 == 0:
            mode_str = "蹲姿" if global_mode == 4 else ("行走" if global_mode == 2 else "站立")
            speed_str = f"{global_speed:.1f}" if global_mode == 2 else "-"
            # 终端打印更新为显示左右肩的 Pitch 角度，方便你同时监控双臂通信状态
            print(f"📡 模式:[{mode_str}] 速度:[{speed_str}] Yaw:[{math.degrees(global_yaw):.0f}°] 动:[{global_movement[0]:.1f},{global_movement[1]:.1f}] L臂:{global_upper_body_pos[3]:.1f} R臂:{global_upper_body_pos[4]:.1f}   ", end='\r')

        time.sleep(0.02)

if __name__ == "__main__":
    t_dynamixel = threading.Thread(target=dynamixel_reader_thread, daemon=True)
    t_gamepad   = threading.Thread(target=gamepad_reader_thread, daemon=True)
    t_zmq       = threading.Thread(target=zmq_publisher_thread, daemon=True)
    
    t_dynamixel.start()
    t_gamepad.start()
    t_zmq.start()
    
    try:
        print("\n========================================================")
        print("  🤖 全功能双臂外骨骼 + 手柄遥操作平台启动")
        print("========================================================\n")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n🛑 正在关闭系统...")
        global_running = False
        time.sleep(0.5)
        print("系统已退出。")