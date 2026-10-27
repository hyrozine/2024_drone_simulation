#! /usr/bin/env python3

#rosimport
import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox

IMAGE_WIDTH=1241
IMAGE_HEIGHT=376

import sys
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')


import argparse
import os
import platform
import sys
from pathlib import Path
import time
import cv2
import torch
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative


from models.experimental import attempt_load
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

from matplotlib import pyplot as plt

ros_image=None
vid_path, vid_writer = [None] * 1, [None] * 1
save_dir = None  # increment run
seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
stride, names, pt =None,None,None
"""Params>>>"""
weights=ROOT / 'robocup_yolov5m.pt'   # model path or triton URL
source="/typhoon_h480_0/cgo3_camera/image_raw"   # ros topic
data=ROOT / 'data/robocup.yaml'   # dataset.yaml path
imgsz=(640, 640)   # inference size (height, width)
conf_thres=0.25   # confidence threshold
iou_thres=0.45   # NMS IOU threshold
max_det=1000   # maximum detections per image
device=''   # cuda device, i.e. 0 or 0,1,2,3 or cpu
view_img=False   # show results
save_txt=False   # save results to *.txt
save_conf=False   # save confidences in --save-txt labels
save_crop=False   # save cropped prediction boxes
nosave=False   # do not save images/videos
classes=None   # filter by class: --class 0, or --class 0 2 3
agnostic_nms=False   # class-agnostic NMS
augment=False   # augmented inference
visualize=False   # visualize features
update=False   # update all models
project=ROOT / 'runs/detect'   # save results to project/name
name='exp'   # save results to project/name
exist_ok=False   # existing project/name ok, do not increment
line_thickness=3   # bounding box thickness (pixels)
hide_labels=False   # hide labels
hide_conf=False   # hide confidences
half=False   # use FP16 half-precision inference
dnn=False   # use OpenCV DNN for ONNX inference
vid_stride=1   # video frame-rate stride
"""<<<Params"""
def ros_arg_phase():
    global weights,\
        source,\
        data,\
        imgsz,\
        conf_thres,\
        iou_thres,\
        max_det,\
        device,\
        view_img,\
        save_txt,\
        save_conf,\
        save_crop,\
        nosave,\
        classes,\
        agnostic_nms,\
        augment,\
        visualize,\
        update,\
        project,\
        name,\
        exist_ok,\
        line_thickness,\
        hide_labels,\
        hide_conf,\
        half,\
        dnn,\
        vid_stride
                    
    weights=rospy.get_param("weights",ROOT / 'robocup_yolov5m.pt')
    source=rospy.get_param("source","/typhoon_h480_0/cgo3_camera/image_raw")   # ros topic
    data=rospy.get_param("data",ROOT / 'data/robocup.yaml' )  # dataset.yaml path
    imgsz=rospy.get_param("imgsz",(640, 640)  ) # inference size (height, width)
    conf_thres=rospy.get_param("conf-thres",0.65)   # confidence threshold
    iou_thres = rospy.get_param("iou-thres", 0.2)   # NMS IOU threshold
    max_det = rospy.get_param("max-det", 20)   # maximum detections per image
    device=rospy.get_param("device",0)   # cuda device, i.e. 0 or 0,1,2,3 or cpu
    view_img = rospy.get_param("view-img", True)   # show results
    save_txt = rospy.get_param("save-txt", False)   # save results to *.txt
    save_conf=rospy.get_param("save-conf",False)   # save confidences in --save-txt labels
    save_crop=rospy.get_param("save-crop",False)   # save cropped prediction boxes
    nosave=rospy.get_param("nosave",False)   # do not save images/videos
    classes=rospy.get_param("classes",None)   # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=rospy.get_param("agnostic-nms",False)   # class-agnostic NMS
    augment=rospy.get_param("augment",False)   # augmented inference
    visualize=rospy.get_param("visualize",False)   # visualize features
    update=rospy.get_param("update",False)   # update all models
    project=rospy.get_param("project",ROOT / 'runs/detect')   # save results to project/name
    name=rospy.get_param("name",'exp')   # save results to project/name
    exist_ok=rospy.get_param("exist-ok",False)   # existing project/name ok, do not increment
    line_thickness=rospy.get_param("line-thickness",2)   # bounding box thickness (pixels)
    hide_labels=rospy.get_param("hide-labels",False)   # hide labels
    hide_conf=rospy.get_param("hide-conf", False)   # hide confidences
    half=rospy.get_param("half",False)   # use FP16 half-precision inference
    dnn=rospy.get_param("dnn",False)   # use OpenCV DNN for ONNX inference
    vid_stride=rospy.get_param("vid-stride",1)   # video frame-rate stride
  
def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)

def loadimg(img,topic:str):  # 接受opencv图片
    img_size=640
    cap=None
    source=[topic,]
    im0 = [img[..., ::-1],]
    
    global stride
    im = np.stack([letterbox(x, img_size, stride=stride, auto=True)[0] for x in im0])  # resize
    # print(im.shape)
    im = im[..., ::-1 ].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
    im = np.ascontiguousarray(im)  # contiguous
    s= f'from:{source} '
    return source, im, im0, cap , s
# src=cv2.imread('biye.jpg')
def detect(img,topic):

    global seen ,windows,dt,source
    
    cudnn.benchmark = True
    dataset = []
    dataset.append(loadimg(img,topic))
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    target_msg=BoundingBoxes()
    for path, im, im0s, vid_cap, s in dataset:
        with dt[0]:
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        # print(im.shape)
        with dt[1]:
            # visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            
            p, im0, frame = path[i], im0s[i].copy(), 0
            s += f'{i}: '
            # print(im)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results & pub target 
                
                
                for *xyxy, conf, cls in reversed(det):
                    #set bboxmsg
                    bbox_msg=BoundingBox()
                    # print(xyxy[0].item(),xyxy[1].item(),xyxy[2].item(),xyxy[3].item())
                    bbox_msg.id=0
                    bbox_msg.xmin=int(xyxy[0])
                    bbox_msg.xmax=int(xyxy[2])
                    bbox_msg.ymin=int(xyxy[3])
                    bbox_msg.ymax=int(xyxy[1])
                    bbox_msg.probability=float(conf)
                    bbox_msg.Class=names[int(cls)]
                    target_msg.bounding_boxes.append(bbox_msg)
                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                    

            # Stream results
            im0 = annotator.result()
            if view_img:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if vid_path[i] != save_path:  # new video
                    vid_path[i] = save_path
                    if isinstance(vid_writer[i], cv2.VideoWriter):
                        vid_writer[i].release()  # release previous video writer
                    if vid_cap:  # video
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    else:  # stream
                        fps, w, h = 30, im0.shape[1], im0.shape[0]
                    save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                    vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                vid_writer[i].write(im0)

        # Print time (inference-only)
        rospy.loginfo(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms,FPS:{1.0/dt[1].dt:.2f}")
    #### Create CompressedIamge ####
    publish_image(im0[...,::-1])
    return target_msg

def image_callback_1(image,args):
    
    ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    # cv2.imshow("test", ros_image)
    # cv2.waitKey(1)  # 1 millisecond
    topic=args[0]
    # print(topic)
    with torch.no_grad():
        target_msg=detect(ros_image,topic)
    target_msg.header=Header(stamp=rospy.Time.now())
    target_msg.image_header=image.header
    darknet_pub.publish(target_msg)

def publish_image(imgdata):
    image_temp=Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    
    image_temp.height=imgdata.shape[0]
    image_temp.width=imgdata.shape[1]
    image_temp.encoding='rgb8'
    image_temp.data=np.array(imgdata).tobytes()
    #print(imgdata)
    #image_temp.is_bigendian=True
    image_temp.header=header
    
    image_temp.step=image_temp.width*3
    image_pub.publish(image_temp)


if __name__ == '__main__':
    
    ros_arg_phase()
    '''
    yolo init
    '''
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)
    
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    
    bs = 1  # batch_size
    
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
    
    '''
    ros node init
    '''
    rospy.init_node('ros_yolo')
    image_topic_1 = "/typhoon_h480_0/cgo3_camera/image_raw"
    image_topic_2 = "/typhoon_h480_2/cgo3_camera/image_raw"
    rospy.Subscriber(image_topic_1, Image, image_callback_1,callback_args=("typhoon_h480_0",), queue_size=2, buff_size=52428800 )
    rospy.Subscriber(image_topic_2, Image, image_callback_1,callback_args=("typhoon_h480_2",), queue_size=2, buff_size=52428800 )
    
    image_pub = rospy.Publisher('/yolo_result_out', Image, queue_size=4)
    vehicle_id='0'
    darknet_pub = rospy.Publisher("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, queue_size=3)
    #rospy.init_node("yolo_result_out_node", anonymous=True)
    #   # allow window resize (Linux)
    # cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])

    rospy.spin()
