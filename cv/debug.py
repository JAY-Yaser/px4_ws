import cv2

def test_camera_simple():
    # 0 代表默认摄像头，如果你有两个摄像头，可以尝试改成 1
    cap = cv2.VideoCapture(0)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("错误：无法打开摄像头 /dev/video0")
        print("提示：请检查虚拟机设置中是否已将 USB 摄像头连接到 Ubuntu。")
        return

    print("摄像头已启动，按 'q' 键退出...")

    while True:
        # 读取一帧画面
        # ret 是布尔值，表示是否读取成功；frame 是图像数据
        ret, frame = cap.read()
        
        if not ret:
            print("错误：无法接收画面 (帧丢失)")
            break

        # 显示画面，窗口名字叫 'Camera Test'
        cv2.imshow('Camera Test', frame)

        # 等待 1 毫秒，如果按下 'q' 键则退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出")

if __name__ == "__main__":
    test_camera_simple()