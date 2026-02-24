import asyncio
import websockets
import json
import zmq
import time
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_planner_message, build_command_message

# 1. 初始化 ZMQ 发布者 (模拟原本的 Pico Manager)
context = zmq.Context()
zmq_socket = context.socket(zmq.PUB)
zmq_socket.bind("tcp://*:5556")
time.sleep(0.5)

print("[WebBridge] ZMQ socket bound to tcp://*:5556")

# 2. 发送初始化指令，切换 C++ 接收端到 Planner 模式
try:
    zmq_socket.send(build_command_message(start=True, stop=False, planner=True))
    print("[WebBridge] Sent start command to gear_sonic_deploy")
except Exception as e:
    print(f"ZMQ init error: {e}")

# 注意：适配 websockets 16.0+ 版本，移除了 path 参数
async def handle_ws(websocket):
    print(f"[WebBridge] Web UI Connected from {websocket.remote_address}!")
    try:
        async for message in websocket:
            data = json.loads(message)
            left_arm = data.get("left_arm", [0.0]*7)
            right_arm = data.get("right_arm", [0.0]*7)

            # 3. 组装 17-DoF 的 upper_body_position
            # [腰yaw, 腰roll, 腰pitch, 左肩P, 右肩P, 左肩R, 右肩R, 左肩Y, 右肩Y...]
            upper_body_pos = [
                0.0, 0.0, 0.0,  # 3个腰部自由度 (保持为0)
                left_arm[0], right_arm[0], # Shoulder Pitch
                left_arm[1], right_arm[1], # Shoulder Roll
                left_arm[2], right_arm[2], # Shoulder Yaw
                left_arm[3], right_arm[3], # Elbow
                left_arm[4], right_arm[4], # Wrist Roll
                left_arm[5], right_arm[5], # Wrist Pitch
                left_arm[6], right_arm[6], # Wrist Yaw
            ]

            # 4. 打包并发送给 C++ 底层
            # mode=0 对应 LocomotionMode.IDLE
            msg = build_planner_message(
                mode=0, 
                movement=[0.0, 0.0, 0.0],  # 键盘的移动意图，这里先设为0
                facing=[1.0, 0.0, 0.0],
                speed=-1.0,
                upper_body_position=upper_body_pos
            )
            zmq_socket.send(msg)
            
    except websockets.exceptions.ConnectionClosed:
        print("[WebBridge] Web UI Disconnected.")

# 5. 使用现代的 asyncio 启动模式
async def main():
    print("[WebBridge] WebSocket Server running on ws://0.0.0.0:8765")
    async with websockets.serve(handle_ws, "0.0.0.0", 8765):
        await asyncio.Future()  # 永久运行，不退出

if __name__ == "__main__":
    asyncio.run(main())