# -*- coding: utf-8 -*-
"""
Created on Wed Mar 30 09:06:30 2022

@author: ctn
"""

import torch

class Network:
    def __init__(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

    def run_inference(self, image):
        return self.model(image)
    
    def get_bottle(self, result):
        objs = result.pandas().xyxy[0]
        objs_name = objs.loc[objs['name'] == 'bottle']
        obj = objs_name.iloc[0]
        return (obj.xmin, obj.ymin), (obj.xmax, obj.ymax)
    
    def get_middle(self, p1, p2):
        x = (p2[0] - p1[0])/2 + p1[0]
        y = (p2[1] - p1[1])/2 + p1[1]
        return x,y