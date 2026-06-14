import cv2
import numpy as np
import rclpy
import json
import os
import time
from rclpy.node import Node
from std_msgs.msg import String
from multiprocessing import shared_memory
from rclpy.executors import ExternalShutdownException

os.environ["QT_QPA_PLATFORM"] = "offscreen"

class CVProcessorNode(Node):
    def __init__(self):
        super().__init__('cv_processor')
        cv2.setUseOptimized(True)
        
        # TODO: Change to your Dashboard PC's Tailscale IP
        client_ip = "100.x.y.z"
        udp_port = 5600

        # 1. Listen to local UDP stream from the Host OS
        gst_in = "udpsrc port=5000 ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1"
        self.cap = cv2.VideoCapture(gst_in, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError("Network Error: Cannot open UDP stream on port 5000")

        # 2. Wait patiently for the first frame (up to 5 seconds)
        self.get_logger().info("Waiting for UDP video stream to sync...")
        ret = False
        frame = None
        for _ in range(50):
            ret, frame = self.cap.read()
            if ret and frame is not None:
                break
            time.sleep(0.1)

        if not ret:
            raise RuntimeError("Timeout: Cannot read initial frame. Is rpicam-vid running?")

        # 3. Setup Network Stream to Dashboard (Foxglove)
        gst_out = (
            "appsrc ! videoconvert ! video/x-raw, format=I420 ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=1000 ! "
            f"rtph264pay config-interval=1 pt=96 ! udpsink host={client_ip} port={udp_port} sync=false"
        )
        self.net_stream = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, 30, (frame.shape[1], frame.shape[0]), True)

        # 4. Setup Shared Memory
        shm_name = 'vision_shm'
        try:
            self.shm = shared_memory.SharedMemory(create=True, size=frame.nbytes, name=shm_name)
        except FileExistsError:
            temp_shm = shared_memory.SharedMemory(name=shm_name)
            temp_shm.close()
            temp_shm.unlink()
            self.shm = shared_memory.SharedMemory(create=True, size=frame.nbytes, name=shm_name)

        self.shared_array = np.ndarray(frame.shape, dtype=frame.dtype, buffer=self.shm.buf)
        self.publisher_ = self.create_publisher(String, '/camera/metadata', 10)
        
        self.timer = self.create_timer(1.0 / 30.0, self.capture_frame)
        self.frame_count = 0
        self.get_logger().info("CV Processor is ONLINE and Streaming.")

    def capture_frame(self):
        if not self.cap.grab():
            # Attempt reconnect if stream drops
            self.cap.release()
            gst_in = "udpsrc port=5000 ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1"
            self.cap = cv2.VideoCapture(gst_in, cv2.CAP_GSTREAMER)
            return

        ret, frame = self.cap.retrieve()
        if not ret:
            return

        # A. Publish to Shared Memory
        np.copyto(self.shared_array, frame)
        metadata = {
            "shm_name": self.shm.name,
            "shape": frame.shape,
            "dtype": str(frame.dtype)
        }
        msg = String()
        msg.data = json.dumps(metadata)
        self.publisher_.publish(msg)

        # B. Stream to Network
        #if self.net_stream.isOpened():
        #    self.net_stream.write(frame)
        
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().debug(f"CV Processor: {self.frame_count} frames processed")

    def cleanup(self):
        self.cap.release()
        if hasattr(self, 'net_stream'):
            self.net_stream.release()
        try:
            self.shm.close()
            self.shm.unlink()
        except Exception:
            pass

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
