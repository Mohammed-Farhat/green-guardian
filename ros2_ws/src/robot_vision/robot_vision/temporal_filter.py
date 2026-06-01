"""
Green Guardian — Temporal Consistency Filter
=============================================
Eliminates YOLO false positives on background by requiring a detection
to appear in the same location across N consecutive frames before it is
accepted as a real object.

How to use in your yolo_node.py:
    from temporal_filter import TemporalConsistencyFilter

    # Create once, outside the callback
    self.filter = TemporalConsistencyFilter(
        required_hits=3,   # frames a detection must persist
        iou_threshold=0.4, # min overlap to count as "same object"
        max_misses=2,      # frames allowed to disappear before removing
    )

    # Inside your image callback, replace direct publishing with:
    confirmed = self.filter.update(raw_detections)
    for det in confirmed:
        # publish det as normal
"""

from dataclasses import dataclass, field
from typing import List, Optional


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Detection:
    """
    A single YOLO detection for one frame.
    Bounding box is in normalized coordinates: values between 0.0 and 1.0.
    This matches YOLO's native output format so no conversion is needed.

    Fields:
        class_id    → 0 for organic, 1 for non_organic
        class_name  → "organic" or "non_organic"
        confidence  → float between 0.0 and 1.0
        x_center    → normalized x center of bounding box
        y_center    → normalized y center of bounding box
        width       → normalized width of bounding box
        height      → normalized height of bounding box
    """
    class_id:   int
    class_name: str
    confidence: float
    x_center:   float
    y_center:   float
    width:      float
    height:     float


@dataclass
class TrackedObject:
    """
    Internal tracker state for one candidate detection.
    Maintained across frames by the filter.

    Fields:
        detection     → the most recent Detection for this object
        hits          → how many consecutive frames we've seen this object
        misses        → how many consecutive frames this object was absent
        confirmed     → True once hits >= required_hits (ready to publish)
    """
    detection: Detection
    hits:      int = 1
    misses:    int = 0
    confirmed: bool = False


# ─────────────────────────────────────────────────────────────────────────────
# IoU helper
# ─────────────────────────────────────────────────────────────────────────────

def compute_iou(a: Detection, b: Detection) -> float:
    """
    Compute Intersection over Union between two bounding boxes.

    Both detections use normalized [x_center, y_center, width, height].
    We convert to [x1, y1, x2, y2] (top-left and bottom-right corners)
    to compute the intersection rectangle.

    Returns a float between 0.0 (no overlap) and 1.0 (perfect overlap).
    A value above 0.4 means the two boxes are describing the same object.
    """
    # Convert center format → corner format
    a_x1 = a.x_center - a.width  / 2
    a_y1 = a.y_center - a.height / 2
    a_x2 = a.x_center + a.width  / 2
    a_y2 = a.y_center + a.height / 2

    b_x1 = b.x_center - b.width  / 2
    b_y1 = b.y_center - b.height / 2
    b_x2 = b.x_center + b.width  / 2
    b_y2 = b.y_center + b.height / 2

    # Intersection rectangle
    inter_x1 = max(a_x1, b_x1)
    inter_y1 = max(a_y1, b_y1)
    inter_x2 = min(a_x2, b_x2)
    inter_y2 = min(a_y2, b_y2)

    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    if inter_area == 0.0:
        return 0.0

    area_a = a.width * a.height
    area_b = b.width * b.height
    union_area = area_a + area_b - inter_area

    return inter_area / (union_area + 1e-6)


# ─────────────────────────────────────────────────────────────────────────────
# The Filter
# ─────────────────────────────────────────────────────────────────────────────

class TemporalConsistencyFilter:
    """
    Filters YOLO detections across consecutive video frames.

    A detection is only considered REAL (and published to ROS2) once it
    has appeared in the same location for `required_hits` frames in a row.

    Parameters
    ----------
    required_hits : int
        How many consecutive frames a detection must persist before it is
        confirmed and published. Default 3.
        → Lower (2): faster response, slightly more false positives
        → Higher (4-5): more reliable, but slower response

    iou_threshold : float
        Minimum IoU overlap for two detections (across frames) to be
        considered the same object. Default 0.4.
        → Lower (0.3): more lenient matching, handles fast-moving objects
        → Higher (0.5): stricter — only nearly identical boxes match

    max_misses : int
        How many consecutive frames a tracked object is allowed to be
        absent before it is dropped from the tracker entirely. Default 2.
        This prevents the tracker from dying if YOLO misses one frame.
    """

    def __init__(
        self,
        required_hits: int   = 3,
        iou_threshold: float = 0.4,
        max_misses:    int   = 2,
    ):
        self.required_hits = required_hits
        self.iou_threshold = iou_threshold
        self.max_misses    = max_misses

        # Internal list of all objects currently being tracked
        self._tracked: List[TrackedObject] = []

    # ── Public API ────────────────────────────────────────────────────────────

    def update(self, detections: List[Detection]) -> List[Detection]:
        """
        Feed one frame's worth of YOLO detections into the filter.

        Returns a list of CONFIRMED detections — only those that have
        appeared consistently for `required_hits` frames. This is what
        you publish to ROS2.

        Call this once per frame inside your ROS2 image callback.

        Parameters
        ----------
        detections : List[Detection]
            Raw detections from YOLO for the current frame.
            Can be an empty list if YOLO found nothing.

        Returns
        -------
        List[Detection]
            Confirmed, stable detections ready to publish.
            Empty list if nothing has been confirmed yet.
        """
        matched_tracker_indices = set()
        matched_detection_indices = set()

        # ── Step 1: Match incoming detections to existing tracked objects ──
        #
        # For each new detection, find the tracked object with the highest
        # IoU overlap. If the overlap is above iou_threshold, they are the
        # same object — increment its hit counter.
        #
        for d_idx, det in enumerate(detections):
            best_iou        = 0.0
            best_track_idx  = -1

            for t_idx, tracked in enumerate(self._tracked):
                if t_idx in matched_tracker_indices:
                    continue  # already claimed by another detection

                iou = compute_iou(det, tracked.detection)
                if iou > best_iou:
                    best_iou       = iou
                    best_track_idx = t_idx

            if best_iou >= self.iou_threshold and best_track_idx >= 0:
                # This detection matches an existing tracked object
                track = self._tracked[best_track_idx]
                track.detection = det          # update with latest bbox + conf
                track.hits     += 1
                track.misses    = 0

                if track.hits >= self.required_hits:
                    track.confirmed = True     # ready to publish

                matched_tracker_indices.add(best_track_idx)
                matched_detection_indices.add(d_idx)

        # ── Step 2: New detections that didn't match anything ──────────────
        #
        # These are brand new — start tracking them with hit count = 1.
        # They will NOT be published until they persist for required_hits frames.
        #
        for d_idx, det in enumerate(detections):
            if d_idx not in matched_detection_indices:
                self._tracked.append(TrackedObject(detection=det))

        # ── Step 3: Handle tracked objects not seen this frame ─────────────
        #
        # Increment their miss counter. If they've been absent for too long,
        # remove them from the tracker entirely.
        #
        for t_idx, tracked in enumerate(self._tracked):
            if t_idx not in matched_tracker_indices:
                tracked.misses += 1

        # Remove objects that have been absent too long
        self._tracked = [
            t for t in self._tracked
            if t.misses <= self.max_misses
        ]

        # ── Step 4: Return only confirmed detections ───────────────────────
        return [t.detection for t in self._tracked if t.confirmed]

    def reset(self):
        """
        Clear all tracked objects.
        Call this if the robot teleports or the scene changes suddenly.
        """
        self._tracked = []

    @property
    def tracked_count(self) -> int:
        """How many objects are currently being tracked (confirmed or not)."""
        return len(self._tracked)

    @property
    def confirmed_count(self) -> int:
        """How many objects are currently confirmed (being published)."""
        return sum(1 for t in self._tracked if t.confirmed)