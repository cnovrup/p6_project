print("\033[H\033[J") 
import os
from PIL import Image
import torch
import numpy as np
import albumentations as A
import cv2

class BoundingBox: #Create class to easier contain bounding box data
    def __init__(self,Name, u, v, width, height ):
        self.u = u
        self.v = v
        self.height = height
        self.width = width
        self.name = Name

        
currentDirec = os. getcwd()
ImagePath = os.path.join(currentDirec, 'images')
BBPath = os.path.join(currentDirec, 'labels')

Oldn = 0
Images = os.listdir(ImagePath)
BBList = os.listdir(BBPath)
bboxes = []


for i in range(len(Images)): #len(Images) #Here is the amount of images started with

    for n in range(5): #This loop is the multiplied, meaning each number 'i' with be multiplied with 4 
        image = Image.open(os.path.join(ImagePath,str(Images[i]))) #Path of the image
        color_image = np.asanyarray(image) #Import the image
        file = open(os.path.join(BBPath,str(BBList[i])), 'r') #Open BB data file: #center_u center_v width height
        InputBB = file.readlines()
        file.close()
        Input = len(InputBB) #Check how many bounding boxes
        print(Input)
        
        bboxesInput = InputBB
        
        category_ids = []
        bboxes = []
        
        for j in range(Input):
            Read = bboxesInput[j]
            ReadSplit = Read.split() #Read the four YOLO coordinates of the bounding box 
            if ReadSplit[0] == '0':
                bb1 = 0 #Ene
            if ReadSplit[0] == '1':
                bb1 = 1 #Gyvel
            category_ids.append(bb1) #This list has to contain the classes of the bounding boxes for the image   
            bboxes.append([[ReadSplit[1],ReadSplit[2],ReadSplit[3],ReadSplit[4]]])
            
        category_id_to_name = {0: 'Ene', 1: 'Gyvel'} #Here we related the class number to a name 
        
        transform = A.Compose([ #This is where the transformations are declared 
            A.HorizontalFlip(p=0.5), #p = the propability if this augmentation happening
            A.RandomSizedBBoxSafeCrop(416, 416, p = 0.5), 
            A.Rotate(limit = [-50, 50],p = 0.5), 
            A.ColorJitter(brightness=(0.75, 1.65), contrast=(0.8, 1.2), saturation=(0.25, 2), hue=(-0.2, 0.2))
        ], bbox_params=A.BboxParams(format='yolo', label_fields=['category_ids']),)
        
        data3 = []
        for j in range(len(bboxes)):
            data1 = bboxes[j][0]
            data3.append([float(data1[0]),float(data1[1]),float(data1[2]),float(data1[3])])
        
        transformed = transform(image=color_image, bboxes=data3, category_ids=category_ids) #Here the transformation is applied

        transformed_image = transformed['image']
        transformed_bboxes = transformed['bboxes']
        
        BBout = []
        for j in range(len(transformed_bboxes)):
            BBout.append(BoundingBox(transformed['category_ids'][j],transformed_bboxes[j][0],transformed_bboxes[j][1],transformed_bboxes[j][2],transformed_bboxes[j][3]))
        
        BBTransformedPath = os.path.join(currentDirec,'ProcessedBB')
        file = open(os.path.join(BBTransformedPath, 'Data'+str(Oldn+n)+'.txt'),"w+") #Write to the file if it exist, or overwrite
        
        for j in range(len(BBout)):
            file.writelines(str(BBout[j].name)+' '+str(BBout[j].u)+' '+str(BBout[j].v)+' '+str(BBout[j].width)+' '+str(BBout[j].height))
            file.writelines('\n')
        file.close()
        
        u = []
        v = []
        height = []
        width = []
        for j in range(len(BBout)):
            u.append(BBout[j].u*416) #Note this number '416' is the amount if pixels, should be changed accordingly
            v.append(BBout[j].v*416)
            width.append(BBout[j].width*416-1)
            height.append(BBout[j].height*416-1)
            
        TransFormedImagePath = os.path.join(currentDirec, 'ProcessedImages') #The the image without bounding box
        TransformedImageName = os.path.join(TransFormedImagePath,"Data"+str(Oldn+n)+".jpg")
        cv2.imwrite(TransformedImageName,transformed_image)
        
        for j in range(len(BBout)):
            if BBout[j].name == 1:
                color = (0,255,0) #Green 1 #Gyvel
            if BBout[j].name == 0:
                color = (255,0,0) #Blue 0 #Ene
            cv2.rectangle(transformed_image, (int(u[j]-(width[j]/2)), int(v[j]-(height[j]/2))), (int(u[j]+(width[j]/2)), int(v[j]+(height[j]/2))),color  , 2) #This is were we draw the rectangle
        
        TransFormedImagePathBB = os.path.join(currentDirec, 'ProcessedImagesWBB') #Save the image with bounding box
        TransformedImageNameBB = os.path.join(TransFormedImagePathBB,"Data"+str(Oldn+n)+".jpg")
        cv2.imwrite(TransformedImageNameBB,transformed_image)
        
    Oldn  = Oldn +n
print('Done')
