# -*- coding: utf-8 -*-
"""
Created on Wed Mar 30 09:06:30 2022

@author: ctn
"""

import torch
import torch.backends.cudnn as cudnn
import numpy as np


from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                            increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import time_sync

class Network:
    def __init__(self):
        cudnn.benchmark = True
        self.weights = 'yolov5n.pt'
        self.device = torch.device('cuda:0')
        self.dnn = False
        self.data = 'coco128.yaml'
        self.half = False
        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.model.warmup(imgsz=(1, 3, 640,640))
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.classes=None
        self.agnostic_nms=False
       
        
        
    def run_inference(self, image):
        im = [image]
        img = np.stack(im, 0)
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img) 
        img = torch.from_numpy(img).to(self.device)
        img = im.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim
        start = time_sync()
        results = self.model(img, augment=False, visualize=False)
        results = non_max_suppression(results, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
        
        return results, (time_sync()-start)*1000
    
    def get_bottle(self, results):
        a = results[0].cpu().numpy()
        if (len(a)>0):
            for entry in a:
                if self.names[int(entry[5])] == 'bottle':
                    return (int(entry[0]),int(entry[1])), (int(entry[2]),int(entry[3]))
        return (-1,-1),(-1,-1)
        
    
    def get_middle(self, p1, p2):
        x = (p2[0] - p1[0])/2 + p1[0]
        y = (p2[1] - p1[1])/2 + p1[1]
        return x,y
