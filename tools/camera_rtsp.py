#!/usr/bin/env python3
"""
camera_rtsp.py — ROS2 camera -> UDP RTP stream for QGroundControl

Reads /downward_camera from ROS2, encodes to H.264 RTP over UDP.
QGC connects with:  udp://:5600  (or set in Application Settings > Video)

No external downloads needed — uses GStreamer (already installed).

Usage:
    python3 tools/camera_rtsp.py                          # UDP mode (QGC default)
    python3 tools/camera_rtsp.py --mode tcp --port 5700    # TCP MJPEG (browser)
"""

import argparse
import subprocess
import sys
import time
import os
import struct
import socket
import threading
from io import BytesIO

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraStreamBridge(Node):
    def __init__(self, ros_topic, width, height, fps, mode, port):
        super().__init__('camera_stream_bridge')
        self.bridge = CvBridge()
        self.width = width
        self.height = height
        self.fps = fps
        self.mode = mode
        self.port = port
        self.latest_frame = None
        self._stop = False

        self.sub = self.create_subscription(
            Image, ros_topic, self.image_cb, 10)

        if mode == 'udp':
            self._start_gst_udp()
        elif mode == 'tcp':
            self._start_tcp_server()
        else:
            self.get_logger().error(f'Unknown mode: {mode}')

    def image_cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Frame error: {e}')

    # -------- UDP RTP mode (for QGC) ---------------------------------------
    def _start_gst_udp(self):
        """GStreamer pipeline: raw BGR -> H.264 RTP -> UDP sink."""
        pipeline = (
            f'fdsrc ! videoparse format=bgr width={self.width} '
            f'height={self.height} framerate={self.fps}/1 '
            f'! videoconvert ! x264enc speed-preset=ultrafast '
            f'tune=zerolatency bitrate=800 '
            f'! rtph264pay config-interval=1 pt=96 '
            f'! udpsink host=127.0.0.1 port={self.port} sync=false'
        )
        self.get_logger().info(f'Starting UDP RTP stream on port {self.port}')
        self.get_logger().info(f'QGC: udp://:{self.port}')

        try:
            self.gst_pipe = subprocess.Popen(
                ['gst-launch-1.0', '-e'] + pipeline.split(),
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            self.get_logger().warn('gst-launch-1.0 not found!')
            self.gst_pipe = None

        # Feed frames from spin loop
        self._feed_thread = threading.Thread(target=self._feed_gst, daemon=True)
        self._feed_thread.start()

    def _feed_gst(self):
        """Feed raw BGR frames to GStreamer stdin."""
        while not self._stop and self.gst_pipe and self.gst_pipe.poll() is None:
            if self.latest_frame is not None:
                try:
                    self.gst_pipe.stdin.write(self.latest_frame.tobytes())
                except BrokenPipeError:
                    break
            else:
                time.sleep(0.01)

    # -------- TCP MJPEG mode (browser fallback) ----------------------------
    def _start_tcp_server(self):
        """Simple MJPEG-over-TCP server (connect with browser or VLC)."""
        self._tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._tcp_sock.bind(('0.0.0.0', self.port))
        self._tcp_sock.listen(1)
        self._tcp_sock.settimeout(1.0)
        self.get_logger().info(f'TCP MJPEG server on port {self.port}')
        self.get_logger().info(f'Browser: http://127.0.0.1:{self.port}')
        self._tcp_thread = threading.Thread(target=self._tcp_loop, daemon=True)
        self._tcp_thread.start()

    def _tcp_loop(self):
        while not self._stop:
            try:
                conn, addr = self._tcp_sock.accept()
                self.get_logger().info(f'TCP client: {addr}')
                threading.Thread(target=self._handle_tcp, args=(conn,),
                               daemon=True).start()
            except socket.timeout:
                continue
            except Exception:
                break

    def _handle_tcp(self, conn):
        """Send MJPEG stream over TCP."""
        try:
            conn.sendall(
                b'HTTP/1.1 200 OK\r\n'
                b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n')
            while not self._stop:
                if self.latest_frame is not None:
                    ok, jpg = cv2.imencode('.jpg', self.latest_frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        conn.sendall(
                            b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(jpg)).encode() +
                            b'\r\n\r\n' + jpg.tobytes() + b'\r\n')
                time.sleep(1.0 / self.fps)
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            conn.close()

    def cleanup(self):
        self._stop = True
        if hasattr(self, 'gst_pipe') and self.gst_pipe:
            self.gst_pipe.stdin.close()
            self.gst_pipe.terminate()
            self.gst_pipe.wait(timeout=2)
        if hasattr(self, '_tcp_sock'):
            self._tcp_sock.close()


def main():
    parser = argparse.ArgumentParser(description='ROS2 camera -> QGC video stream')
    parser.add_argument('--ros-topic', default='/downward_camera')
    parser.add_argument('--mode', choices=['udp', 'tcp'], default='udp',
                        help='udp=RTP for QGC, tcp=MJPEG for browser')
    parser.add_argument('--port', type=int, default=5600,
                        help='UDP/TCP port (QGC default UDP: 5600)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    args = parser.parse_args()

    rclpy.init()
    node = CameraStreamBridge(args.ros_topic, args.width, args.height,
                              args.fps, args.mode, args.port)
    if args.mode == 'udp':
        print(f'\n  === QGC Video Setup ===')
        print(f'  In QGC: Application Settings > General > Video')
        print(f'  Video Source: UDP (default)')
        print(f'  Port: {args.port}')
        print(f'  Or: udp://:{args.port}\n')
    elif args.mode == 'tcp':
        print(f'\n  === TCP MJPEG Stream ===')
        print(f'  Browser: http://127.0.0.1:{args.port}\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
