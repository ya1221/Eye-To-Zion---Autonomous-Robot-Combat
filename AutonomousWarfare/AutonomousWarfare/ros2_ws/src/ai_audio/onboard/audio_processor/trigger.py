"""IDLE -> CAPTURING -> COOLDOWN impact-trigger state machine.

This file is intentionally duplicated at
../../pipeline/dataset_collector/trigger.py. onboard/ must build and run as
a fully self-contained directory independent of pipeline/ (and vice versa),
so there is no shared build context or cross-directory import between them
-- see docker-compose.yml. If you tune the trigger logic here (threshold
behavior, pre/post-roll handling, cooldown), mirror the change in the other
copy, or training-time capture and live-time capture will silently diverge.
"""

import collections

import numpy as np


class TriggerCapture:
    """Feed it consecutive audio blocks; it calls on_capture(frames) once
    per detected event with exactly pre_roll_frames + post_roll_frames of
    audio, then enforces cooldown_frames of silence before re-arming."""

    def __init__(self, pre_roll_frames, post_roll_frames, cooldown_frames, threshold_amp, on_capture):
        self.pre_roll_frames = pre_roll_frames
        self.post_roll_frames = post_roll_frames
        self.cooldown_frames = cooldown_frames
        self.threshold_amp = threshold_amp
        self.on_capture = on_capture

        self.ring = collections.deque(maxlen=pre_roll_frames)
        self.capture_buf = []
        self.frames_captured = 0
        self.frames_to_skip = 0
        self.state = "IDLE"  # IDLE -> CAPTURING -> COOLDOWN -> IDLE

    def feed(self, block):
        if self.state == "IDLE":
            peak = int(np.abs(block).max()) if block.size else 0
            if peak >= self.threshold_amp:
                self.capture_buf = list(self.ring) + [block]
                self.frames_captured = len(self.ring) + block.shape[0]
                self.state = "CAPTURING"
            else:
                for row in block:
                    self.ring.append(row.reshape(1, -1))

        elif self.state == "CAPTURING":
            self.capture_buf.append(block)
            self.frames_captured += block.shape[0]
            if self.frames_captured >= self.pre_roll_frames + self.post_roll_frames:
                frames = np.concatenate(self.capture_buf, axis=0)
                target = self.pre_roll_frames + self.post_roll_frames
                self.on_capture(frames[:target])
                self.capture_buf = []
                self.ring.clear()
                self.frames_to_skip = self.cooldown_frames
                self.state = "COOLDOWN"

        elif self.state == "COOLDOWN":
            self.frames_to_skip -= block.shape[0]
            if self.frames_to_skip <= 0:
                tail = block[self.frames_to_skip:] if self.frames_to_skip < 0 else block[block.shape[0]:]
                for row in tail[-self.pre_roll_frames:]:
                    self.ring.append(row.reshape(1, -1))
                self.state = "IDLE"
