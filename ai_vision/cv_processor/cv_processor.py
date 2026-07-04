import cv2
import numpy as np
import rclpy
import json
import os
import platform
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from multiprocessing import shared_memory
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from turbojpeg import TurboJPEG, TJPF_BGR

os.environ["QT_QPA_PLATFORM"] = "offscreen"

# Debian/Ubuntu multiarch triplets for libturbojpeg0's install location.
# ctypes.util.find_library() is unreliable in slim containers, so we resolve
# an explicit path per-arch rather than relying on the dynamic linker's search.
_TURBOJPEG_LIB_PATHS = {
    "aarch64": "/usr/lib/aarch64-linux-gnu/libturbojpeg.so.0",
    "arm64": "/usr/lib/aarch64-linux-gnu/libturbojpeg.so.0",
    "x86_64": "/usr/lib/x86_64-linux-gnu/libturbojpeg.so.0",
    "amd64": "/usr/lib/x86_64-linux-gnu/libturbojpeg.so.0",
}

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

        # BEST_EFFORT/VOLATILE: drop-if-late semantics over Zenoh, no resend/replay overhead
        compressed_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.compressed_publisher_ = self.create_publisher(CompressedImage, '/camera/compressed', compressed_qos)

        # Field-tunable without a rebuild: ros2 param set /cv_processor jpeg_quality <n>
        self.declare_parameter('jpeg_quality', 75)

        # Fail fast, same as the cv2.VideoCapture check above: if TurboJPEG can't
        # load here, encoding is bound to fail later, so don't start the node.
        arch = platform.machine()
        lib_path = _TURBOJPEG_LIB_PATHS.get(arch)
        if lib_path is None or not os.path.isfile(lib_path):
            raise RuntimeError(
                f"libturbojpeg not found for architecture '{arch}' (looked for: {lib_path}). "
                "Install the libturbojpeg0 package for this platform."
            )
        try:
            self.jpeg = TurboJPEG(lib_path=lib_path)
        except Exception as e:
            raise RuntimeError(f"Failed to load TurboJPEG from {lib_path}: {e}") from e

        self.compressed_frame_counter = 0

        self.timer = self.create_timer(1.0 / 30.0, self.capture_frame)
        self.get_logger().info("CV Processor ONLINE.")

    def capture_frame(self):
        # A timer callback already queued when SIGINT/SIGTERM arrives can still
        # fire once after shutdown starts tearing down the context - skip it
        # rather than publish into an invalid context.
        if not rclpy.ok(): return
        if not self.cap.grab(): return
        ret, frame = self.cap.retrieve()
        if not ret: return

        stamp = self.get_clock().now().to_msg()

        # Write to current buffer, then publish and toggle
        write_idx = self.buf_idx
        np.copyto(self.shared_arrays[write_idx], frame)
        self.buf_idx = 1 - self.buf_idx
        msg = String()
        msg.data = json.dumps({
            "shm_name": self.shms[write_idx].name,
            "shape": self.frame_shape,
            "dtype": self.frame_dtype,
            "buf_idx": write_idx,
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
        })
        self.publisher_.publish(msg)

        quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        ok = False
        try:
            jpeg_bytes = self.jpeg.encode(frame, quality=quality, pixel_format=TJPF_BGR)
            ok = True
        except Exception as e:
            self.get_logger().warn(f"TurboJPEG encode failed, dropping frame: {e}")

        if ok:
            img_msg = CompressedImage()
            img_msg.header.stamp = stamp
            img_msg.format = 'jpeg'
            img_msg.data = jpeg_bytes
            self.compressed_publisher_.publish(img_msg)

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
    except RCLError as e:
        # Residual race: a queued callback can still hit an invalid context
        # between the rclpy.ok() check and the actual publish call. Benign
        # on shutdown, not a bug to fail loud on like the init-time checks above.
        node.get_logger().warn(f"RCLError during shutdown, ignoring: {e}")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()