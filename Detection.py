# -*- coding: utf-8 -*-
"""
Created on Wed Mar 30 09:06:30 2022

@author: ctn
"""

import torch

class Network:
    def __init__(self):
        #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='last.pt', force_reload=True)

    def run_inference(self, image):
        return self.model(image)
    
    def get_bottle(self, result):
        objs = result.pandas().xyxy[0]
        objs_name = objs.loc[objs['name'] == 'Weed']
        if(len(objs_name) > 0):
            obj = objs_name.iloc[0]
            return (int(obj.xmin), int(obj.ymin)), (int(obj.xmax), int(obj.ymax)), obj.confidence
        else:
            return (-1,-1), (-1, -1), 0
        
    def get_middle(self, p1, p2):
        x = (p2[0] - p1[0])/2 + p1[0]
        y = (p2[1] - p1[1])/2 + p1[1]
        return int(x),int(y)