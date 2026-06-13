# step2_mission.py
import time
import math
from drone_config import get_connection, send_position

def main():
    master = get_connection()
    # 路径：向东飞 20 米，再向北飞 20 米
    path = [
        [20, 0, -10.0, math.radians(0)],
        [20, 20, -10.0, math.radians(90)],
        [0, 0, -10.0, 0]
    ]

    print("接管控制，执行任务...")
    for pt in path:
        print(f"目标: {pt}")
        while True:
            send_position(master, pt[0], pt[1], pt[2], pt[3])
            # 这里加入你原来的 is_fully_aligned 判断逻辑...
            # 为了演示，我们简化为延时
            time.sleep(0.1)
            # 假设 5 秒后到达下一个点（实际应使用位置闭环判断）
            break 

    print("任务结束，准备降落")
    # 发送降落指令...

if __name__ == "__main__":
    main()