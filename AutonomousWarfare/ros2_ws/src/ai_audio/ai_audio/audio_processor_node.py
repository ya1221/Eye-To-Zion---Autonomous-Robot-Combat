import json
import os
import queue
import time

import numpy as np
import librosa
import onnxruntime as ort
import sounddevice as sd

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from .trigger import TriggerCapture

FULL_SCALE = 32768.0


def resolve_device(name_or_index):
    if name_or_index is None:
        return None
    try:
        return int(name_or_index)
    except ValueError:
        pass
    needle = name_or_index.lower()
    for idx, info in enumerate(sd.query_devices()):
        if info["max_input_channels"] > 0 and needle in info["name"].lower():
            return idx
    return None


def softmax(x):
    e = np.exp(x - np.max(x))
    return e / e.sum()


class AudioProcessorNode(Node):
    def __init__(self):
        super().__init__('audio_processor')

        # Hardware / capture -- must match dataset_collector's capture format
        # exactly (48kHz mono int16), since the trained model has never seen
        # audio in any other format.
        self.DEVICE_NAME = 'voicehat'
        self.SAMPLE_RATE = 48000
        self.CHANNELS = 1
        self.BLOCKSIZE = 512

        # Trigger + window shape -- must match Stage 1/2 exactly (pre_roll +
        # post_roll = window_length used when the training clips were cut),
        # or the live window fed to the model won't look like what it was
        # trained on.
        self.PRE_ROLL = 0.05
        self.POST_ROLL = 0.2
        self.TRIGGER_THRESHOLD = 0.15  # retune against real deployment noise floor, same as dataset_collector
        self.COOLDOWN = 0.05

        # Feature extraction -- must match preprocessing/make_features.py's
        # defaults exactly, since that's what produced the training data.
        self.N_MELS = 64
        self.N_FFT = 1024
        self.HOP_LENGTH = 480

        # Decision + output
        self.HIT_CONFIDENCE_THRESHOLD = 0.5
        self.TOPIC = '/audio/impact_alert'
        self.POLL_INTERVAL = 0.02  # seconds between queue-drain ticks

        self.declare_parameter('model_dir', '')
        self._load_model(self._resolve_model_dir())

        self.publisher_ = self.create_publisher(String, self.TOPIC, 10)

        self.audio_q = queue.Queue()
        self.pre_roll_frames = int(self.PRE_ROLL * self.SAMPLE_RATE)
        self.post_roll_frames = int(self.POST_ROLL * self.SAMPLE_RATE)
        self.cooldown_frames = int(self.COOLDOWN * self.SAMPLE_RATE)
        self.threshold_amp = self.TRIGGER_THRESHOLD * FULL_SCALE

        self.trigger = TriggerCapture(
            self.pre_roll_frames, self.post_roll_frames, self.cooldown_frames,
            self.threshold_amp, on_capture=self._on_event,
        )

        device = resolve_device(self.DEVICE_NAME)
        if device is None:
            self.get_logger().warn(f"No input device matching '{self.DEVICE_NAME}', falling back to system default")

        self.stream = sd.InputStream(
            samplerate=self.SAMPLE_RATE,
            channels=self.CHANNELS,
            dtype='int16',
            device=device,
            blocksize=self.BLOCKSIZE,
            callback=self._audio_callback,
        )
        self.stream.start()
        self.get_logger().info(f"Listening for impacts on '{self.TOPIC}' (device={self.DEVICE_NAME})")

        self.timer = self.create_timer(self.POLL_INTERVAL, self._process_queue)

    # ──────────────────────── model resolution ────────────────────
    def _resolve_model_dir(self):
        configured = self.get_parameter('model_dir').value
        if configured:
            return configured
        env = os.environ.get('AUDIO_MODEL_DIR')
        if env:
            return env
        return os.path.join(get_package_share_directory('ai_audio'), 'models')

    def _load_model(self, model_dir):
        onnx_path = os.path.join(model_dir, 'impact_cnn.onnx')
        if not os.path.isfile(onnx_path):
            raise RuntimeError(
                f"Impact model not found at '{onnx_path}'. Check the model_dir "
                "parameter/AUDIO_MODEL_DIR, or rebuild the package so the model "
                "is installed into share/ai_audio/models/."
            )
        with open(os.path.join(model_dir, 'labels.json')) as f:
            label_to_idx = json.load(f)
        with open(os.path.join(model_dir, 'feature_stats.json')) as f:
            self.stats = json.load(f)

        self.idx_to_label = {idx: label for label, idx in label_to_idx.items()}
        self.hit_idx = label_to_idx.get('hit')
        if self.hit_idx is None:
            raise RuntimeError(f"labels.json has no 'hit' entry: {label_to_idx}")

        self.session = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.expected_shape = tuple(self.stats['input_shape'][1:])  # (n_mels, n_frames), drops the batch dim
        self.get_logger().info(f"Loaded {onnx_path}, labels={label_to_idx}, expected feature shape={self.expected_shape}")

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f"stream status: {status}")
        self.audio_q.put(indata.copy())

    def _process_queue(self):
        while True:
            try:
                block = self.audio_q.get_nowait()
            except queue.Empty:
                return
            self.trigger.feed(block.reshape(-1, self.CHANNELS))

    def _on_event(self, frames):
        # Raw int16 -> float by a fixed divisor, same as make_features.py --
        # not a peak-normalization, so relative loudness (the actual signal
        # the model learned from) survives into the feature values.
        samples = frames[:, 0].astype(np.float32) / FULL_SCALE

        mel = librosa.feature.melspectrogram(
            y=samples, sr=self.SAMPLE_RATE, n_fft=self.N_FFT,
            hop_length=self.HOP_LENGTH, n_mels=self.N_MELS, power=2.0,
        )
        feat = librosa.power_to_db(mel, ref=1.0).astype(np.float32)

        if feat.shape != self.expected_shape:
            self.get_logger().warn(f"feature shape {feat.shape} != expected {self.expected_shape}, skipping")
            return

        norm = (feat - self.stats['mean']) / self.stats['std']
        x = norm[None, None, :, :].astype(np.float32)

        logits = self.session.run(None, {self.input_name: x})[0][0]
        probs = softmax(logits)
        hit_prob = float(probs[self.hit_idx])
        label = self.idx_to_label[int(np.argmax(probs))]

        if hit_prob >= self.HIT_CONFIDENCE_THRESHOLD:
            msg = String()
            msg.data = json.dumps({
                'event': 'chassis_impact',
                'confidence': round(hit_prob, 4),
                'timestamp': time.time(),
            })
            self.publisher_.publish(msg)
            self.get_logger().info(f"HIT detected, confidence={hit_prob:.3f}")
        else:
            self.get_logger().debug(f"trigger fired but classified '{label}' (hit_prob={hit_prob:.3f}), no alert")

    def cleanup(self):
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AudioProcessorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as e:
        node.get_logger().warn(f"RCLError during shutdown, ignoring: {e}")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
