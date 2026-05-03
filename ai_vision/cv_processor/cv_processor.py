import cv2
import numpy as np
import rclpy
import json
import os
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import shared_memory
from rclpy.executors import ExternalShutdownException

# Force OpenCV to run in headless mode (no GUI attempt)
os.environ["QT_QPA_PLATFORM"] = "offscreen"

class CVProcessorNode(Node):
    def __init__(self):
        super().__init__('cv_processor')
        
        cv2.setUseOptimized(True)
        self.get_logger().info(f"OpenCV Optimization: {cv2.useOptimized()}")

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Camera hardware not found.")
            raise RuntimeError("Hardware Error")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Cannot read initial frame.")
            raise RuntimeError("Stream Error")

        shm_name = 'vision_shm'
        try:
            # Try to create a fresh segment
            self.shm = shared_memory.SharedMemory(create=True, size=frame.nbytes, name=shm_name)
        except FileExistsError:
            # If exists, attach to it, then unlink and create fresh
            self.get_logger().warn("Old SHM found. Cleaning up...")
            temp_shm = shared_memory.SharedMemory(name=shm_name)
            temp_shm.close()
            temp_shm.unlink()
            self.shm = shared_memory.SharedMemory(create=True, size=frame.nbytes, name=shm_name)

        self.shared_array = np.ndarray(frame.shape, dtype=frame.dtype, buffer=self.shm.buf)
        self.publisher_ = self.create_publisher(String, '/camera/metadata', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.capture_frame)
        
        self.skip_frame = 0
        
        self.get_logger().info("CV Processor is now ROBUST and ONLINE.")

    def capture_frame(self):
        # Grab frame without decoding to save CPU cycles
        if not self.cap.grab():
            self.cap.release()
            self.cap = cv2.VideoCapture(0)
            return

        self.skip_frame += 1
        
        # Skip alternating frames at the hardware/decoding level
        if self.skip_frame % 2 != 1:
            self.skip_frame = 0
            return

        # Decode the frame only when we intend to use it
        ret, frame = self.cap.retrieve()
        if not ret:
            return

        np.copyto(self.shared_array, frame)

        metadata = {
            "shm_name": self.shm.name,
            "shape": frame.shape,
            "dtype": str(frame.dtype)
        }
        msg = String()
        msg.data = json.dumps(metadata)
        self.publisher_.publish(msg)

    def cleanup(self):
        self.cap.release()
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass
        print("CV Processor: Clean shutdown.")

def main(args=None):
    rclpy.init(args=args)
    node = CVProcessorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
