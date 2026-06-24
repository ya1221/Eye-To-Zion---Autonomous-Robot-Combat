import cv2
import numpy as np
import rclpy
import json
import os
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import shared_memory
from rclpy.executors import ExternalShutdownException

os.environ["QT_QPA_PLATFORM"] = "offscreen"

class CVProcessorNode(Node):
    def __init__(self):
        super().__init__('cv_processor')
        cv2.setUseOptimized(True)
        
        # UDP stream from host
        gst_in = "udpsrc port=5000 ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
        self.cap = cv2.VideoCapture(gst_in, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open UDP stream. Is rpicam-vid running?")

        # Initialize Double-Buffered Shared Memory (prevents torn frames)
        ret, frame = self.cap.read()
        self.buf_idx = 0
        self.shms = []
        self.shared_arrays = []
        for i in range(2):
            shm_name = f'vision_shm_{i}'
            try:
                temp_shm = shared_memory.SharedMemory(name=shm_name)
                temp_shm.close()
                temp_shm.unlink()
            except FileNotFoundError:
                pass
            shm = shared_memory.SharedMemory(create=True, size=frame.nbytes, name=shm_name)
            self.shms.append(shm)
            self.shared_arrays.append(np.ndarray(frame.shape, dtype=frame.dtype, buffer=shm.buf))
        
        self.frame_shape = frame.shape
        self.frame_dtype = str(frame.dtype)
        self.publisher_ = self.create_publisher(String, '/camera/metadata', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.capture_frame)
        self.get_logger().info("CV Processor ONLINE.")

    def capture_frame(self):
        if not self.cap.grab(): return
        ret, frame = self.cap.retrieve()
        if not ret: return

        # Write to current buffer, then publish and toggle
        write_idx = self.buf_idx
        np.copyto(self.shared_arrays[write_idx], frame)
        self.buf_idx = 1 - self.buf_idx
        msg = String()
        msg.data = json.dumps({"shm_name": self.shms[write_idx].name, "shape": self.frame_shape, "dtype": self.frame_dtype, "buf_idx": write_idx})
        self.publisher_.publish(msg)

    def cleanup(self):
        self.cap.release()
        for shm in self.shms:
            try:
                shm.close()
                shm.unlink()
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = CVProcessorNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally: node.cleanup(); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()