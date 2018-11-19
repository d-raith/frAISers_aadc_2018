from random import randint
import erfnet
import torch
import torch.backends.cudnn as cudnn
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
from yolo.darknet import Darknet
from yolo.util import *
from yolo.preprocess import prep_image
import random
import pickle as pkl
import yolo.detect_net as detect_net


def initModelRenamed(model, weights_path, to_rename, rename):
    saved_model = torch.load(weights_path, map_location=lambda storage, loc: storage)['state_dict']
    # print(saved_model.keys())
    model_dict = model.state_dict()
    # print(model_dict.keys())

    weights_changed = {}
    for k, v in saved_model.items():
        k = k.replace(to_rename, rename)
        weights_changed[k] = v

    weights_changed = {k: v for k, v in weights_changed.items() if k in model_dict}

    print("Loaded dict with %d entries..." % len(weights_changed))
    model_dict.update(weights_changed)
    model.load_state_dict(model_dict)


class ColorCode():
    def random_color_coding(self, max_label):
        coding = {}
        for i in range(max_label + 1):
            coding[i] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        return coding

    def color_code_labels(self, net_out, argmax=True):
        if argmax:
            labels, indices = net_out.max(1)
            labels_cv = indices.cpu().data.numpy().squeeze()
        else:
            labels_cv = net_out.cpu().data.numpy().squeeze()

        h = labels_cv.shape[0]
        w = labels_cv.shape[1]

        color_coded = np.zeros((h, w, 3), dtype=np.uint8)

        for x in range(w):
            for y in range(h):
                color_coded[y, x, :] = self.color_coding[labels_cv[y, x]]

        return color_coded

    def __init__(self, max_classes):
        super(ColorCode, self).__init__()
        self.color_coding = self.random_color_coding(max_classes)


def visJetColorCoding(name, img):
    img = img.detach().cpu().squeeze().numpy()
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img)
    cv2.imshow(name, color_img)


def visImage3Chan(data, name):
    cv = np.transpose(data.cpu().data.numpy().squeeze(), (1, 2, 0))
    cv = cv2.cvtColor(cv, cv2.COLOR_RGB2BGR)
    cv2.imshow(name, cv)

color_coder = ColorCode(11)

def main():
    global confidence, nms_thesh, num_classes, classes

    # Load configuration
    model = erfnet.Net(11)
    model = torch.nn.DataParallel(model.cuda(), [0])
    model.eval()

    detect_model = detect_net.DetectionNetwork()

    # Inference
    # Data loading
    matches = ['/home/vertensj/Documents/annotated_real_data/processed/images/track3_withCP/image_paths.txt']
    list_dataset_paths = []  # Stores pairs of consecutive image paths

    for f in matches:
        with open(f) as f:
            content = f.readlines()
        content = [x.strip() for x in content]

        for line in content:
            frames = line.split(" ")  # Current first, then previous
            list_dataset_paths.append(frames[0])  # Image, Seg, Dt

    for path in list_dataset_paths:

        # Data preparation #############################################################################################
        image_prepared, org_img, dim = prep_image(path, detect_model.inp_dim)
        image_prepared = image_prepared.cuda()
        im_dim_list = [dim]
        im_dim_list = torch.FloatTensor(im_dim_list).repeat(1, 2).cuda()

        # Detection Inference ##########################################################################################
        detect_output = detect_model.detect(image_prepared, im_dim_list)
        if detect_output is not None:
            detect_image = detect_model.visualize_outputs(detect_output, org_img)

            cv2.imshow('detect_image', detect_image)
            cv2.waitKey()

        # DONE #########################################################################################################

if __name__ == '__main__':
    main()
