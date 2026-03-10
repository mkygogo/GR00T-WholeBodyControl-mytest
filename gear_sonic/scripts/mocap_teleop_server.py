import time
import math
import threading
import zmq
from dynamixel_sdk import *

# 导入项目中已有的 ZMQ 打包工具
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_planner_message, build_command_message

# ================= 1. 舵机与串口配置 =================
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_PRESENT_POSITION   = 132
LEN_PRO_PRESENT_POSITION    = 4
PROTOCOL_VERSION            = 2.0
BAUDRATE                    = 57600
DEVICENAME                  = '/dev/ttyUSB0'  # ⚠️ 请确认你的 Ubuntu 串口号是否正确！

# 更新为你最新的右臂舵机 ID：从肩部(14)到腕部(9)
DXL_IDs = [14, 13, 12, 11, 10, 9] 

# ================= 2. 关节正反向配置 (极其重要) =================
# 如果标定后，你发现某个关节运动方向反了（比如你往前抬，模型往后抬），
# 只需要在这里把对应的 1 改成 -1 即可，无需改动其他代码！
DIR_MAP = {
    14: 1,  # 右肩 Pitch (前后摆动)
    13: 1,  # 右肩 Roll (侧向平举)
    12: 1,  # 右肩 Yaw (大臂自转)
    11: 1,  # 右肘
    10: 1,  # 右腕 Roll
    9:  1   # 右腕 Pitch
}

# ================= 全局变量 =================
global_upper_body_pos = [0.0] * 17
global_running = True

# 映射函数：计算相对角度并转为弧度
def map_servo_to_urdf(current_deg, initial_deg, direction=1):
    # 用当前角度减去开机标定的初始角度，得到相对变化量
    delta_deg = current_deg - initial_deg
    urdf_deg = delta_deg * direction
    return math.radians(urdf_deg)

def dynamixel_reader_thread():
    global global_upper_body_pos, global_running
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        print(f"[Dynamixel] ❌ 无法打开串口或设置波特率: {DEVICENAME}")
        global_running = False
        return

    # 关闭扭矩，进入拖动示教模式
    for dxl_id in DXL_IDs:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 0)
        groupSyncRead.addParam(dxl_id)

    # ================= 核心：开机自动标定零点 =================
    print("\n" + "="*60)
    print(" ⚠️ 准备标定零点：请把机械臂【垂直向下】自然下垂放置")
    print(" ⚠️ 保持不动... 3秒后将自动记录当前姿态为 0 度起点")
    print("="*60 + "\n")
    
    for i in range(3, 0, -1):
        print(f"标定倒计时: {i}...")
        time.sleep(1)
        
    initial_positions_deg = {}
    print("[Dynamixel] 正在抓取零点数据...")
    
    # 尝试读取初始位置，直到所有舵机都成功读到
    while len(initial_positions_deg) < len(DXL_IDs) and global_running:
        groupSyncRead.txRxPacket()
        for dxl_id in DXL_IDs:
            if dxl_id not in initial_positions_deg and groupSyncRead.isAvailable(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION):
                present_pos = groupSyncRead.getData(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                if present_pos >= 2**31: present_pos -= 2**32
                initial_positions_deg[dxl_id] = (present_pos / 4096.0) * 360.0
        time.sleep(0.05)

    print(f"\n[Dynamixel] ✅ 零点记录完成！初始角度为: ")
    for k in DXL_IDs:
        print(f"  - ID {k}: {initial_positions_deg[k]:.2f} 度")
    print("\n[Dynamixel] 🚀 你现在可以自由转动机械臂了！")

    # ================= 实时读取与映射循环 =================
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
        
        # 将你的 ID 14~9 依次映射到右臂对应的自由度
        if 14 in positions_deg: new_pos[4]  = map_servo_to_urdf(positions_deg[14], initial_positions_deg[14], DIR_MAP[14]) # 右肩 P
        if 13 in positions_deg: new_pos[6]  = map_servo_to_urdf(positions_deg[13], initial_positions_deg[13], DIR_MAP[13]) # 右肩 R
        if 12 in positions_deg: new_pos[8]  = map_servo_to_urdf(positions_deg[12], initial_positions_deg[12], DIR_MAP[12]) # 右肩 Y
        if 11 in positions_deg: new_pos[10] = map_servo_to_urdf(positions_deg[11], initial_positions_deg[11], DIR_MAP[11]) # 右肘
        if 10 in positions_deg: new_pos[12] = map_servo_to_urdf(positions_deg[10], initial_positions_deg[10], DIR_MAP[10]) # 右腕 R
        if 9  in positions_deg: new_pos[14] = map_servo_to_urdf(positions_deg[9],  initial_positions_deg[9],  DIR_MAP[9])  # 右腕 P
        
        global_upper_body_pos = new_pos
        time.sleep(0.02) 

    groupSyncRead.clearParam()
    portHandler.closePort()

def zmq_publisher_thread():
    global global_upper_body_pos, global_running
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
        current_pos = global_upper_body_pos
        msg = build_planner_message(
            mode=0, movement=[0.0, 0.0, 0.0], facing=[1.0, 0.0, 0.0], speed=-1.0,
            upper_body_position=current_pos 
        )
        zmq_socket.send(msg)
        
        print_counter += 1
        if print_counter % 10 == 0:
            # 打印当前右臂发送的弧度，方便观察
            print(f"📡 实时发送弧度: P:{current_pos[4]:.2f} R:{current_pos[6]:.2f} Y:{current_pos[8]:.2f} 肘:{current_pos[10]:.2f} 腕R:{current_pos[12]:.2f} 腕P:{current_pos[14]:.2f}    ", end='\r')

        time.sleep(0.02)

if __name__ == "__main__":
    t_dynamixel = threading.Thread(target=dynamixel_reader_thread)
    t_zmq = threading.Thread(target=zmq_publisher_thread)
    t_dynamixel.start()
    t_zmq.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n🛑 正在关闭系统...")
        global_running = False
        t_dynamixel.join()
        t_zmq.join()
        print("系统已退出。")