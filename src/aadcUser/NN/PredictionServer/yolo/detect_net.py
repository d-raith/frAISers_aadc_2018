import torch
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from yolo.darknet import Darknet
from yolo.util import *
from yolo.preprocess import prep_image
import random
import pickle as pkl


class DetectionNetwork(object):
    def __init__(self):
        self.confidence = 0.7
        self.nms_thesh = 0.4
        self.resolution = 640
        self.scales = "1,2,3"

        self.confidence = float(self.confidence)
        self.nms_thesh = float(self.nms_thesh)
        self.CUDA = torch.cuda.is_available()
        self.num_classes = 80
        self.classes = load_classes('yolo/data/coco.names')
        print("Loading network.....")
        self.model_detect = Darknet('cfg/yolov3.cfg')
        self.model_detect.load_weights('yolo/yolov3.weights')
        print("Network successfully loaded")
        self.model_detect.net_info["height"] = self.resolution
        self.inp_dim = int(self.model_detect.net_info["height"])
        assert self.inp_dim % 32 == 0
        assert self.inp_dim > 32

        if self.CUDA:
            self.model_detect.cuda()

        self.model_detect.eval()
        self.colors = pkl.load(open("yolo/pallete", "rb"))

    def write(self, x, org_img):
        c1 = tuple(x[1:3].int())
        c2 = tuple(x[3:5].int())
        img = org_img
        cls = int(x[-1])
        label = "{0}".format(self.classes[cls])
        color = random.choice(self.colors)
        cv2.rectangle(img, c1, c2, color, 1)
        t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
        c2 = c1[0] + t_size[0] + 3, c1[1] + t_size[1] + 4
        cv2.rectangle(img, c1, c2, color, -1)
        cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225, 255, 255], 1)
        return img

    def detect(self, image, im_dim_list):

        # Detection Inference ##########################################################################################
        prediction = self.model_detect(image, True)
        prediction = write_results(prediction, self.confidence, self.num_classes, nms=True, nms_conf=self.nms_thesh)
        output = prediction

        # Check if something was found....
        if type(prediction) == int:
            return None

        objs = [self.classes[int(x[-1])] for x in output]
        print("{0:20s} {1:s}".format("Objects Detected:", " ".join(objs)))
        print("----------------------------------------------------------")

        # Scaling, considering original input resolution ###############################################################
        im_dim_list = torch.index_select(im_dim_list, 0, output[:, 0].long())

        scaling_factor = torch.min(self.inp_dim / im_dim_list, 1)[0].view(-1, 1)

        output[:, [1, 3]] -= (self.inp_dim - scaling_factor * im_dim_list[:, 0].view(-1, 1)) / 2
        output[:, [2, 4]] -= (self.inp_dim - scaling_factor * im_dim_list[:, 1].view(-1, 1)) / 2

        output[:, 1:5] /= scaling_factor

        for i in range(output.shape[0]):
            output[i, [1, 3]] = torch.clamp(output[i, [1, 3]], 0.0, im_dim_list[i, 0])
            output[i, [2, 4]] = torch.clamp(output[i, [2, 4]], 0.0, im_dim_list[i, 1])

        return output

    def visualize_outputs(self, detect_output, draw_image):
        # Draw every bounding box iteratively
        for n_f in range(detect_output.size(0)):
            draw_image = self.write(detect_output[n_f, ...], draw_image)
        return draw_image