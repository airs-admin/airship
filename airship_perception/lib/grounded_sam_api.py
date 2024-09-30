import logging
import time
import warnings
warnings.simplefilter("ignore")

import cv2
import numpy as np
import torch
import torchvision

from lib.segment_anything.segment_anything import SamPredictor, sam_model_registry

def grounded_sam(grounding_dino_model, sam_predictor, image, classes, box_threshold=0.3, text_threshold=0.25, nms_threshold=0.8, action=None):
    start_time = time.time()
    # detect objects
    detections = grounding_dino_model.predict_with_classes(
        image=image,
        classes=classes,
        box_threshold=box_threshold,
        text_threshold=text_threshold
    )

    # NMS post process
    nms_idx = torchvision.ops.nms(
        torch.from_numpy(detections.xyxy),
        torch.from_numpy(detections.confidence),
        nms_threshold
    ).numpy().tolist()

    detections.xyxy = detections.xyxy[nms_idx]
    detections.confidence = detections.confidence[nms_idx]
    detections.class_id = detections.class_id[nms_idx]

    # Prompting SAM with detected boxes
    def segment(sam_predictor: SamPredictor, image: np.ndarray, xyxy: np.ndarray) -> np.ndarray:
        sam_predictor.set_image(image)
        result_masks = []
        for box in xyxy:
            masks, scores, logits = sam_predictor.predict(
                box=box,
                multimask_output=True
            )
            index = np.argmax(scores)
            result_masks.append(masks[index])
        return np.array(result_masks)

    # convert detections to masks
    detections.mask = segment(
        sam_predictor=sam_predictor,
        image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
        xyxy=detections.xyxy
    )

    # mask
    masks = detections.mask
    mask_list = []
    labels = []

    if action == 'grasp':
        # confidence max
        conf = [
            confidence
            for _, _, confidence, _, _, _
            in detections]
        if conf:
            for mask in masks:
                mask_img = np.zeros(image.shape[:2], dtype=np.uint8)
                mask_img[mask > 0] = 255
                mask_list.append(mask_img)
            mask_list = [mask_list[conf.index(max(conf))]]
        else:
            mask_list = [np.zeros(image.shape[:2], dtype=np.uint8)]

    if action == 'map':
        # [mask1, mask2, ...]
        labels = [
            f"{classes[class_id]}"
            for _, _, _, class_id, _, _
            in detections]
        for mask in masks:
            mask_img = np.zeros(image.shape[:2], dtype=np.uint8)
            mask_img[mask > 0] = 255
            mask_list.append(mask_img)

    end_time = time.time()
    logging.info(f'inference time: %s {end_time - start_time}')

    return mask_list, labels

