import time
import math
import threading
import sys
import zmq
from dynamixel_sdk import *
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_planner_message, build_command_message

# ================= 1. 舵机与串口配置 =================
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_PRESENT_POSITION   = 132
LEN_PRO_PRESENT_POSITION    = 4
PROTOCOL_VERSION            = 2.0
BAUDRATE                    = 57600
DEVICENAME                  = '/dev/ttyUSB0'  # ⚠️ 确认你的串口号

# 左臂(7-2) + 右臂(14-9)，共 12 个舵机
DXL_IDs = [7, 6, 5, 4, 3, 2, 14, 13, 12, 11, 10, 9] 

JOINT_NAMES = {
    # 左臂 (6 DoF)
    7: "左肩 Pitch (前后摆动)",
    6: "左肩 Roll (侧向平举)",
    5: "左肩 Yaw (大臂自转)",
    4: "左肘 (小臂弯曲)",
    3: "左腕 Roll (手腕旋转)",
    2: "左腕 Pitch (手腕上下)",
    
    # 右臂 (6 DoF)
    14: "右肩 Pitch (前后摆动)",
    13: "右肩 Roll (侧向平举)",
    12: "右肩 Yaw (大臂自转)",
    11: "右肘 (小臂弯曲)",
    10: "右腕 Roll (手腕旋转)",
    9:  "右腕 Pitch (手腕上下)"
}

# 动态方向字典（初始全为1）
DIR_MAP = {id: 1 for id in DXL_IDs}

# ================= 全局变量 =================
global_upper_body_pos = [0.0] * 17
global_running = True
current_test_id = None      # 当前正在孤立测试的舵机ID
initial_positions_deg = {}  # 零点标定数据

def map_servo_to_urdf(current_deg, initial_deg, direction=1):
    delta_deg = current_deg - initial_deg
    urdf_deg = delta_deg * direction
    return math.radians(urdf_deg)

def dynamixel_reader_thread():
    global global_upper_body_pos, global_running, initial_positions_deg
    
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

    if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
        print(f"\n[Error] ❌ 无法打开串口: {DEVICENAME}")
        global_running = False
        return

    # 关闭扭矩
    for dxl_id in DXL_IDs:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, 0)
        groupSyncRead.addParam(dxl_id)

    # 尝试读取初始位置 (零点标定)
    while len(initial_positions_deg) < len(DXL_IDs) and global_running:
        groupSyncRead.txRxPacket()
        for dxl_id in DXL_IDs:
            if dxl_id not in initial_positions_deg and groupSyncRead.isAvailable(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION):
                present_pos = groupSyncRead.getData(dxl_id, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
                if present_pos >= 2**31: present_pos -= 2**32
                initial_positions_deg[dxl_id] = (present_pos / 4096.0) * 360.0
        time.sleep(0.05)

    # 实时读取循环
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
        
        # 映射逻辑辅助函数
        def apply_mapping(target_array):
            # 左臂
            if 7 in positions_deg: target_array[3]  = map_servo_to_urdf(positions_deg[7], initial_positions_deg[7], DIR_MAP[7])
            if 6 in positions_deg: target_array[5]  = map_servo_to_urdf(positions_deg[6], initial_positions_deg[6], DIR_MAP[6])
            if 5 in positions_deg: target_array[7]  = map_servo_to_urdf(positions_deg[5], initial_positions_deg[5], DIR_MAP[5])
            if 4 in positions_deg: target_array[9]  = map_servo_to_urdf(positions_deg[4], initial_positions_deg[4], DIR_MAP[4])
            if 3 in positions_deg: target_array[11] = map_servo_to_urdf(positions_deg[3], initial_positions_deg[3], DIR_MAP[3])
            if 2 in positions_deg: target_array[13] = map_servo_to_urdf(positions_deg[2], initial_positions_deg[2], DIR_MAP[2])
            
            # 右臂
            if 14 in positions_deg: target_array[4]  = map_servo_to_urdf(positions_deg[14], initial_positions_deg[14], DIR_MAP[14])
            if 13 in positions_deg: target_array[6]  = map_servo_to_urdf(positions_deg[13], initial_positions_deg[13], DIR_MAP[13])
            if 12 in positions_deg: target_array[8]  = map_servo_to_urdf(positions_deg[12], initial_positions_deg[12], DIR_MAP[12])
            if 11 in positions_deg: target_array[10] = map_servo_to_urdf(positions_deg[11], initial_positions_deg[11], DIR_MAP[11])
            if 10 in positions_deg: target_array[12] = map_servo_to_urdf(positions_deg[10], initial_positions_deg[10], DIR_MAP[10])
            if 9  in positions_deg: target_array[14] = map_servo_to_urdf(positions_deg[9],  initial_positions_deg[9],  DIR_MAP[9])

        # 如果处于单轴测试模式，只让当前被测试的轴更新，其他的死锁为 0.0 (笔直垂下)
        if current_test_id is not None and current_test_id in positions_deg:
            val = map_servo_to_urdf(positions_deg[current_test_id], initial_positions_deg[current_test_id], DIR_MAP[current_test_id])
            # 左臂
            if current_test_id == 7:  new_pos[3] = val
            if current_test_id == 6:  new_pos[5] = val
            if current_test_id == 5:  new_pos[7] = val
            if current_test_id == 4:  new_pos[9] = val
            if current_test_id == 3:  new_pos[11] = val
            if current_test_id == 2:  new_pos[13] = val
            # 右臂
            if current_test_id == 14: new_pos[4] = val
            if current_test_id == 13: new_pos[6] = val
            if current_test_id == 12: new_pos[8] = val
            if current_test_id == 11: new_pos[10] = val
            if current_test_id == 10: new_pos[12] = val
            if current_test_id == 9:  new_pos[14] = val
            
        # 如果测试结束 (current_test_id 为 None)，则全面放开所有轴的控制
        elif current_test_id is None and len(initial_positions_deg) == len(DXL_IDs):
            apply_mapping(new_pos)

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

    while global_running:
        msg = build_planner_message(
            mode=0, movement=[0.0, 0.0, 0.0], facing=[1.0, 0.0, 0.0], speed=-1.0,
            upper_body_position=global_upper_body_pos 
        )
        zmq_socket.send(msg)
        time.sleep(0.02)

def main_interactive_ui():
    global current_test_id, DIR_MAP, global_running

    print("\n" + "="*60)
    print(" 🤖 欢迎使用 GEAR-SONIC 【双臂满配版】外骨骼校准工具 🤖")
    print("="*60)
    print("\n【步骤1】准备零点标定...")
    print("请将物理双臂【垂直向下】自然下垂放置，保持绝对静止！")
    
    for i in range(3, 0, -1):
        print(f"倒计时 {i}...")
        time.sleep(1)
        
    print("等待所有舵机响应...")
    while len(initial_positions_deg) < len(DXL_IDs):
        if not global_running: return
        time.sleep(0.1)
        
    print(f"\n✅ 成功抓取 {len(DXL_IDs)} 个关节的零点！进入单轴隔离测试。\n")

    # 开始逐个轴测试（先左臂后右臂）
    for dxl_id in DXL_IDs:
        current_test_id = dxl_id
        while True:
            print("-" * 55)
            print(f"🎯 正在测试: ID {dxl_id} -> 【{JOINT_NAMES[dxl_id]}】")
            print("👉 请掰动该关节，观察仿真中小人的动作方向：")
            print("  [y] 方向正确，测试下一个关节")
            print("  [r] 方向反了！立即翻转该关节")
            print("  [q] 退出程序")
            
            choice = input("请输入你的选择 (y/r/q): ").strip().lower()
            
            if choice == 'y':
                print(f"✅ ID {dxl_id} 配置已确认。")
                break
            elif choice == 'r':
                DIR_MAP[dxl_id] *= -1
                print(f"🔄 方向已翻转！当前 DIR_MAP[{dxl_id}] = {DIR_MAP[dxl_id]}。请再试一下。")
            elif choice == 'q':
                global_running = False
                return
            else:
                print("无效输入，请重试。")

    print("\n" + "="*60)
    print(" 🎉 12 轴双臂全关节测试完毕！仿真已经放开所有控制。")
    print("="*60)
    
    print("\n⚠️ 你的最终配置文件如下，请将它替换到主运行程序中：\n")
    print("DIR_MAP = {")
    for dxl_id in DXL_IDs:
        print(f"    {dxl_id:2}: {DIR_MAP[dxl_id]:2},  # {JOINT_NAMES[dxl_id]}")
    print("}")
    print("\n(你可以继续自由挥舞双臂。按 Ctrl+C 彻底退出。)")
    
    current_test_id = None 
    
    while global_running:
        time.sleep(1)

if __name__ == "__main__":
    t_dynamixel = threading.Thread(target=dynamixel_reader_thread, daemon=True)
    t_zmq = threading.Thread(target=zmq_publisher_thread, daemon=True)
    t_dynamixel.start()
    t_zmq.start()
    
    try:
        main_interactive_ui()
    except KeyboardInterrupt:
        print("\n\n🛑 强制退出系统...")
        global_running = False
        time.sleep(0.5)