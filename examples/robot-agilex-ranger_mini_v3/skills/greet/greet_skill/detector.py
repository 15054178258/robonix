# SPDX-License-Identifier: Apache-2.0
"""Cheap, local person detector — the first stage of the greeting pipeline.

Runs YOLO on every camera frame (fast, free, on the Jetson GPU). Only when it
sees one or more people does the skill escalate to the paid VLM for a
customised greeting, so the expensive call happens once per encounter rather
than on every frame.

COCO class 0 is `person`; we restrict inference to that class.
"""
from __future__ import annotations

import logging
import os

import numpy as np

log = logging.getLogger("greet_skill")


class PersonDetector:
    def __init__(self, weights: str | None = None, conf: float = 0.4,
                 device: str | None = None, min_box_frac: float = 0.0):
        from ultralytics import YOLO  # imported lazily so import errors surface at on_init

        # Default to the smallest model; the operator can point at a larger /
        # custom weight file via config. Ultralytics fetches the stock weight
        # on first use if it isn't on disk.
        self.weights = weights or os.environ.get("GREET_YOLO_WEIGHTS", "yolov8n.pt")
        self.conf = conf
        # device None lets ultralytics pick CUDA when available, else CPU.
        self.device = device
        # Ignore people whose bounding box is shorter than this fraction of the
        # frame height — i.e. too far away to be "in the robot's path". 0 = count
        # everyone.
        self.min_box_frac = min_box_frac
        self.model = YOLO(self.weights)
        log.info("[yolo] loaded %s (conf=%.2f device=%s min_box_frac=%.2f)",
                 self.weights, conf, device or "auto", min_box_frac)

    def count_people(self, img: np.ndarray) -> int:
        """Number of NEARBY `person` boxes in a HxWx3 RGB frame — boxes shorter
        than `min_box_frac` of the frame height (far-away people) are skipped.
        Never raises — a detector hiccup returns 0 so the loop just skips."""
        try:
            results = self.model.predict(
                img, conf=self.conf, classes=[0], device=self.device, verbose=False
            )
        except Exception as e:  # noqa: BLE001
            log.warning("[yolo] predict failed: %s", e)
            return 0
        if self.min_box_frac <= 0.0:
            return sum(len(r.boxes) for r in results)
        h = max(int(img.shape[0]), 1)
        cnt = 0
        for r in results:
            for box in r.boxes:
                xy = box.xyxy[0]
                box_h = float(xy[3] - xy[1]) / h
                if box_h >= self.min_box_frac:
                    cnt += 1
        return cnt
