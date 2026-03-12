import cv2
import numpy as np
from ultralytics import YOLO

# COCO 17-keypoint skeleton connections (0-indexed)
_SKELETON = [
    (0, 1), (0, 2),           # nose -> eyes
    (1, 3), (2, 4),           # eyes -> ears
    (5, 6),                   # shoulders
    (5, 7), (7, 9),           # left arm
    (6, 8), (8, 10),          # right arm
    (5, 11), (6, 12),         # torso sides
    (11, 12),                 # hips
    (11, 13), (13, 15),       # left leg
    (12, 14), (14, 16),       # right leg
]

_KPT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]

_KPT_COLOR  = (0, 255, 0)    # green keypoints
_LIMB_COLOR = (0, 165, 255)  # orange limbs
_BOX_COLOR  = (255, 0, 0)    # blue bounding box
_TEXT_COLOR = (255, 255, 255)
_CONF_THR   = 0.3


class SkeletonDetector:
    def __init__(self, model_path: str = "yolo26l-pose.pt"):
        self.model = YOLO(model_path)

    def detect(self, image: np.ndarray) -> list[dict]:
        """
        Run pose estimation on a BGR image.

        Returns a list of person dicts:
            [
                {
                    "box": {"x1": int, "y1": int, "x2": int, "y2": int, "conf": float},
                    "keypoints": {
                        "nose":           {"x": float, "y": float, "conf": float},
                        "left_eye":       {...},
                        ...
                    }
                },
                ...
            ]
        """
        results = self.model(image)
        persons = []

        for result in results:
            boxes = result.boxes
            kpts  = result.keypoints.data  # (N, 17, 3)

            for i in range(len(boxes)):
                x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
                conf = float(boxes.conf[i])

                kp_array = kpts[i].cpu().numpy()  # (17, 3)
                keypoints = {}
                for name, (x, y, c) in zip(_KPT_NAMES, kp_array):
                    keypoints[name] = {
                        "x":    round(float(x), 4),
                        "y":    round(float(y), 4),
                        "conf": round(float(c), 4),
                    }

                persons.append({
                    "box": {
                        "x1": x1, "y1": y1,
                        "x2": x2, "y2": y2,
                        "conf": round(conf, 4),
                    },
                    "keypoints": keypoints,
                })

        return persons

    def draw(self, image: np.ndarray, persons: list[dict]) -> np.ndarray:
        """
        Draw bounding boxes and skeletons from a previous `detect()` call.
        Returns an annotated copy of `image`.
        """
        frame = image.copy()
        for person in persons:
            b = person["box"]
            cv2.rectangle(frame, (b["x1"], b["y1"]), (b["x2"], b["y2"]),
                          _BOX_COLOR, 2, cv2.LINE_AA)
            cv2.putText(frame, f"person {b['conf']:.2f}",
                        (b["x1"], b["y1"] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, _TEXT_COLOR, 1, cv2.LINE_AA)
            self._draw_skeleton(frame, person["keypoints"])
        return frame

    def process(self, image: np.ndarray) -> np.ndarray:
        """Convenience: detect + draw in one call. Returns annotated copy."""
        return self.draw(image, self.detect(image))

    def _draw_skeleton(self, frame: np.ndarray, keypoints: dict) -> None:
        for a, b in _SKELETON:
            kpa = keypoints[_KPT_NAMES[a]]
            kpb = keypoints[_KPT_NAMES[b]]
            if kpa["conf"] < _CONF_THR or kpb["conf"] < _CONF_THR:
                continue
            cv2.line(frame,
                     (int(kpa["x"]), int(kpa["y"])),
                     (int(kpb["x"]), int(kpb["y"])),
                     _LIMB_COLOR, 2, cv2.LINE_AA)

        for kp in keypoints.values():
            if kp["conf"] < _CONF_THR:
                continue
            cv2.circle(frame, (int(kp["x"]), int(kp["y"])),
                       4, _KPT_COLOR, -1, cv2.LINE_AA)


if __name__ == "__main__":
    import os

    img_path = os.path.join(os.path.dirname(__file__), "bus.jpg")
    out_path = os.path.join(os.path.dirname(__file__), "bus_skeleton.jpg")

    detector = SkeletonDetector("yolo26l-pose.pt")
    frame = cv2.imread(img_path)
    annotated = detector.process(frame)
    cv2.imwrite(out_path, annotated)
    print(f"Saved → {out_path}")
