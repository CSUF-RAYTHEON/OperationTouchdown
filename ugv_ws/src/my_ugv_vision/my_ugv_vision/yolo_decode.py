"""YOLOv2-style decoder for the Akida-converted UGV YOLO model.

Pure numpy, no ROS imports. The Akida ``akida_models`` YOLO head uses YOLOv2
decoding: per-anchor (tx, ty, tw, th, to) + class softmax. Anchors are in
grid-cell units (NOT pixels), so the bbox dimensions are
``anchors[k] * exp(t_wh) / grid_size * input_size``.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


def sigmoid(x: np.ndarray) -> np.ndarray:
    out = np.empty_like(x, dtype=np.float32)
    np.negative(x, out=out)
    np.exp(out, out=out)
    out += 1.0
    np.reciprocal(out, out=out)
    return out


def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x.astype(np.float32, copy=False)
    x = x - np.max(x, axis=axis, keepdims=True)
    np.exp(x, out=x)
    s = np.sum(x, axis=axis, keepdims=True)
    return x / np.maximum(s, 1e-9)


def decode_yolo_v2(
    raw: np.ndarray,
    anchors: np.ndarray,
    num_classes: int,
    input_size: Tuple[int, int] = (224, 224),
    grid: Tuple[int, int] = (7, 7),
    score_thresh: float = 0.3,
) -> List[Tuple[float, float, float, float, float, int]]:
    """Decode raw YOLOv2 head output to a list of (x1,y1,x2,y2,score,cls).

    Box coords are in pixels of the model input frame (default 224x224).
    """
    anchors = np.asarray(anchors, dtype=np.float32).reshape(-1, 2)
    num_anchors = anchors.shape[0]
    input_w, input_h = input_size
    gw, gh = grid

    if raw.ndim == 4:
        raw = raw[0]
    if raw.shape[-1] != num_anchors * (5 + num_classes):
        raw = raw.reshape(gh, gw, num_anchors, 5 + num_classes)
    else:
        raw = raw.reshape(gh, gw, num_anchors, 5 + num_classes)

    tx = raw[..., 0]
    ty = raw[..., 1]
    tw = raw[..., 2]
    th = raw[..., 3]
    to = raw[..., 4]
    tcls = raw[..., 5:]

    cell_j = np.arange(gw, dtype=np.float32).reshape(1, gw, 1)
    cell_i = np.arange(gh, dtype=np.float32).reshape(gh, 1, 1)

    bx_center = (sigmoid(tx) + cell_j) / float(gw) * float(input_w)
    by_center = (sigmoid(ty) + cell_i) / float(gh) * float(input_h)

    anchor_w = anchors[:, 0].reshape(1, 1, num_anchors)
    anchor_h = anchors[:, 1].reshape(1, 1, num_anchors)
    bw = anchor_w * np.exp(np.clip(tw, -10.0, 10.0)) / float(gw) * float(input_w)
    bh = anchor_h * np.exp(np.clip(th, -10.0, 10.0)) / float(gh) * float(input_h)

    obj = sigmoid(to)
    cls_probs = softmax(tcls, axis=-1)
    cls_id = np.argmax(cls_probs, axis=-1)
    cls_score = np.take_along_axis(cls_probs, cls_id[..., None], axis=-1).squeeze(-1)
    score = obj * cls_score

    keep = score >= score_thresh
    if not np.any(keep):
        return []

    bx = bx_center[keep]
    by = by_center[keep]
    bw_k = bw[keep]
    bh_k = bh[keep]
    sc = score[keep]
    cl = cls_id[keep]

    x1 = bx - bw_k * 0.5
    y1 = by - bh_k * 0.5
    x2 = bx + bw_k * 0.5
    y2 = by + bh_k * 0.5

    return [
        (float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
         float(sc[i]), int(cl[i]))
        for i in range(sc.shape[0])
    ]


def _iou(box: Tuple[float, float, float, float],
         others: np.ndarray) -> np.ndarray:
    x1 = np.maximum(box[0], others[:, 0])
    y1 = np.maximum(box[1], others[:, 1])
    x2 = np.minimum(box[2], others[:, 2])
    y2 = np.minimum(box[3], others[:, 3])
    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_box = max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])
    area_others = np.maximum(0.0, others[:, 2] - others[:, 0]) * \
        np.maximum(0.0, others[:, 3] - others[:, 1])
    union = area_box + area_others - inter
    return inter / np.maximum(union, 1e-9)


def nms(
    boxes: Sequence[Tuple[float, float, float, float, float, int]],
    iou_thresh: float = 0.45,
) -> List[Tuple[float, float, float, float, float, int]]:
    """Per-class greedy NMS."""
    if not boxes:
        return []
    arr = np.asarray(boxes, dtype=np.float32)
    classes = arr[:, 5].astype(np.int32)
    kept: List[Tuple[float, float, float, float, float, int]] = []
    for c in np.unique(classes):
        cls_mask = classes == c
        cls_boxes = arr[cls_mask]
        order = np.argsort(-cls_boxes[:, 4])
        cls_boxes = cls_boxes[order]
        while cls_boxes.shape[0] > 0:
            top = cls_boxes[0]
            kept.append((float(top[0]), float(top[1]), float(top[2]),
                         float(top[3]), float(top[4]), int(top[5])))
            if cls_boxes.shape[0] == 1:
                break
            rest = cls_boxes[1:]
            ious = _iou((top[0], top[1], top[2], top[3]), rest[:, :4])
            cls_boxes = rest[ious < iou_thresh]
    return kept


def unletterbox_boxes(
    boxes: Sequence[Tuple[float, float, float, float, float, int]],
    orig_w: int,
    orig_h: int,
    input_w: int = 224,
    input_h: int = 224,
) -> List[Tuple[float, float, float, float, float, int]]:
    """Invert letterbox padding to map boxes from model-input coords back to
    the original image coords."""
    if not boxes:
        return []
    scale = min(input_w / float(orig_w), input_h / float(orig_h))
    new_w = orig_w * scale
    new_h = orig_h * scale
    pad_x = (input_w - new_w) * 0.5
    pad_y = (input_h - new_h) * 0.5
    inv = 1.0 / scale
    out: List[Tuple[float, float, float, float, float, int]] = []
    for x1, y1, x2, y2, sc, cl in boxes:
        ox1 = max(0.0, (x1 - pad_x) * inv)
        oy1 = max(0.0, (y1 - pad_y) * inv)
        ox2 = min(float(orig_w - 1), (x2 - pad_x) * inv)
        oy2 = min(float(orig_h - 1), (y2 - pad_y) * inv)
        if ox2 <= ox1 or oy2 <= oy1:
            continue
        out.append((ox1, oy1, ox2, oy2, sc, cl))
    return out


def letterbox(
    image_rgb: np.ndarray,
    input_w: int = 224,
    input_h: int = 224,
    pad_value: int = 114,
) -> Tuple[np.ndarray, float, int, int]:
    """Resize image preserving aspect ratio and pad with ``pad_value``.

    Returns (padded_image_uint8, scale, pad_x, pad_y).
    """
    import cv2

    h, w = image_rgb.shape[:2]
    scale = min(input_w / float(w), input_h / float(h))
    new_w = int(round(w * scale))
    new_h = int(round(h * scale))
    resized = cv2.resize(image_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x = (input_w - new_w) // 2
    pad_y = (input_h - new_h) // 2
    canvas = np.full((input_h, input_w, 3), pad_value, dtype=np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return canvas, scale, pad_x, pad_y


if __name__ == '__main__':
    rng = np.random.default_rng(0)
    fake = rng.standard_normal((1, 7, 7, 5 * (5 + 3))).astype(np.float32)
    anc = np.array(
        [[0.37449, 0.44152], [0.88506, 0.93953], [1.87466, 1.40808],
         [2.14199, 2.77736], [5.05428, 5.53107]], dtype=np.float32)
    boxes = decode_yolo_v2(fake, anc, num_classes=3, score_thresh=0.0)
    print(f"decoded {len(boxes)} raw boxes (no threshold)")
    pruned = nms(boxes, 0.45)
    print(f"after NMS: {len(pruned)} boxes")
    remapped = unletterbox_boxes(pruned, 1280, 720)
    print(f"after unletterbox to 1280x720: {len(remapped)} boxes")
