
'''
*******************************************************************************
  Audi Autonomous Driving Cup 2018
  Team frAIsers
*******************************************************************************
'''


""" includes """
import sys
import glob
sys.path.append('../NNThrift/gen-py')
import datetime

# python includes
import numpy as np
import threading

import torch
import torch.backends.cudnn as cudnn
import torch.distributed as dist
import torch.nn as nn
import torch.nn.parallel
import torch.optim
import torch.utils.data
import torch.utils.data.distributed
import torch.nn.functional as F
import cv2
import erfnet
from yolo.preprocess import letterbox_image
import yolo.detect_net as detect_net
from random import randint

# map_thrift data structure include
from nn_thrift import NNComm as nn_thrift

# Thrift includes
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from thrift.server import TServer
import sys


""" variables """
# variables
car26_ip = '192.168.168.126'
car27_ip = '192.168.168.127'
car_locally = 'localhost'
localhost_ip = '127.0.0.1'

host = car27_ip
port = '9093'

DEBUG = False
VISUALIZE = False   # imshow on or of


def initModelRenamed(model, weights_path, to_rename, rename):
    saved_model = torch.load(weights_path, map_location=lambda storage, loc: storage)['state_dict']
    #print(saved_model.keys())
    model_dict = model.state_dict()
    #print(model_dict.keys())

    weights_changed = {}
    for k, v in saved_model.items():
        k = k.replace(to_rename, rename)
        weights_changed[k] = v

    weights_changed = {k: v for k, v in weights_changed.items() if k in model_dict}

    print("Loaded dict with %d entries..." % len(weights_changed))
    model_dict.update(weights_changed)
    model.load_state_dict(model_dict)

class NNCommHandler():
    def __init__(self):
        self.log = {}
        self.framecounter = 0
    
    def ping(self):
        print('ping()')
        return True

    def getNNPrediction(self, raw_image):
        """
        Parameters:
         - raw_image
        """
        self.framecounter += 1
        print('NNServer: receiving image', self.framecounter)
        segmented_image = handleThriftNNImage(raw_image)
        return segmented_image



""" functions """

net = 1
#detect_model = 1
device = 'cuda'

class ColorCode():

    def random_color_coding(self, max_label):
        coding = {}
        no_manual = 11
        coding[0] = [0, 0, 0]  # background
        coding[1] = [0, 0, 255]  # road
        coding[2] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        coding[3] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        coding[4] = [255, 0, 0]  # car
        coding[5] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        coding[6] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        coding[7] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        coding[8] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        coding[9] = [0, 255, 0]  # intersection
        coding[10] = [255, 255, 0]  # person
        for i in range(no_manual, max_label + 1 + no_manual):
            coding[i] = [randint(0, 255), randint(0, 255), randint(0, 255)]
        return coding

    def color_code_labels(self, labels_cv):
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


color_coder = ColorCode(19)


def visJetColorCoding(img):
    color_img = np.zeros(img.shape, dtype=img.dtype)
    cv2.normalize(img, color_img, 0, 255, cv2.NORM_MINMAX)
    color_img = color_img.astype(np.uint8)
    color_img = cv2.applyColorMap(color_img, cv2.COLORMAP_JET, color_img) # works with BGR? but ok for us
    return color_img


def prep_image(orig_im, inp_dim):
    """
    Prepare image for inputting to the neural network. 
    
    Returns a Variable 
    """
    dim = orig_im.shape[1], orig_im.shape[0]
    img = (letterbox_image(orig_im, (inp_dim, inp_dim)))
    img_ = img[:,:,::-1].transpose((2,0,1)).copy()
    img_ = torch.from_numpy(img_).float().div(255.0).unsqueeze(0)
    return img_, dim


def adtf_network_init():
    global net
    global device
    #global detect_model

    #detect_model = detect_net.DetectionNetwork()
    #net = ENet(num_classes=6)
    #net = FCN32s2()
    net = erfnet.Net(11)
    print("network loaded")
    net = net.to(device)
    net = torch.nn.DataParallel(net)
    initModelRenamed(net, '/home/aadc/AADC/src/aadcUser/NN/PredictionServer/trained_models/erfnet_best_seg_reg_2.pth.tar', 'module.features.', 'module.')

    net.eval()
    print("network ready")


def adtf_inference(image):
    global net
    global device
    global color_coder
    single_image_tensor = torch.from_numpy(np.moveaxis(image, 2, 0) / 255.0)
    single_image_tensor = single_image_tensor.to(torch.float32)
    single_image_tensor = single_image_tensor.unsqueeze(0)
    # NB: Asserts a batch size of 1 atm
    mean = [123.7164293/255, 124.40157681/255, 119.43845371/255]
    with torch.no_grad():
        single_image_tensor = single_image_tensor.to(device)
        single_image_tensor[:, 0, :, :] -= mean[0]
        single_image_tensor[:, 1, :, :] -= mean[1]
        single_image_tensor[:, 2, :, :] -= mean[2]
        seg, reg = net(single_image_tensor)
        labels, indices = seg.max(1)
        seg_cv = indices.cpu().data.numpy().squeeze()
        reg_cv = reg.detach().cpu().squeeze().numpy()
        # print("seg_cv.shape", labels_cv.shape)  # shape=(80, 600), values=[0, 10]
        # print("reg_cv.shape", reg_cv.shape)  # shape=(80, 600), values=[0, 1]

        if (VISUALIZE == True):
            # Visualization
            color_reg = visJetColorCoding(reg_cv)
            color_seg = color_coder.color_code_labels(seg_cv)
            # print("color_reg.shape", color_reg.shape)  # shape=(height, width, channels), values[0, 255]
            # print("color_seg.shape", color_seg.shape)  # shape=(height, width, channels), values[0, 255]
            # image_cv = np.transpose(single_image_tensor.cpu().detach().numpy().squeeze(), (1, 2, 0))
            # image_cv = cv2.cvtColor(image_cv, cv2.COLOR_RGB2BGR)
            cv2.imshow("image", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.imshow("dt", cv2.cvtColor(color_reg, cv2.COLOR_RGB2BGR))
            cv2.imshow("seg", cv2.cvtColor(color_seg, cv2.COLOR_RGB2BGR))
            cv2.waitKey(10)
        cv2.normalize(reg_cv, reg_cv, 0, 255, cv2.NORM_MINMAX)
        reg_cv = reg_cv.astype(np.uint8)
        seg_cv = seg_cv.astype(np.uint8)
        # result = np.stack((seg_cv, np.zeros(seg_cv.shape, dtype=np.uint8), reg_cv), axis=2)
        result = np.stack((seg_cv, reg_cv), axis=2)
        # result = image
        # result = cv2.addWeighted(image, 0.5, color_reg, 0.5, 0.0)
        no_channels = 2
        # result = cv2.line(result, (int(result.shape[1] / 2), 0), (int(result.shape[1] / 2), result.shape[0] - 1), (255, 0, 0))
        return no_channels, result  # print("result.shape", result.shape)  # shape=(height, width, channels), values=[0, 255]
def createNNServer(port):
    try:
        print(bcolors.OKBLUE + 'NNServer: trying to create NNServer on port {}'.format(port))
        handler = NNCommHandler()
        processor = nn_thrift.Processor(handler)
        transport = TSocket.TServerSocket(host=localhost_ip, port=port)
        tfactory = TTransport.TBufferedTransportFactory()
        pfactory = TBinaryProtocol.TBinaryProtocolFactory()

        server = TServer.TSimpleServer(processor, transport, tfactory, pfactory)
        print('NNServer: server created!' + bcolors.ENDC)
        return [server, handler]
    except:
        print('NNServer: could not create NNServer!' + bcolors.ENDC)

def startNNServer(server_handle):
    print(bcolors.OKBLUE + 'NNServer: starting to serve...' + bcolors.ENDC)
    try:
        print(bcolors.BOLD + '\nPress [STRG+C] to stop serving' + bcolors.ENDC)
        server_handle.serve()
    except KeyboardInterrupt:    
        print(bcolors.OKBLUE + '\nNNServer: stopped serving.' + bcolors.ENDC)


""" process incoming MapMessages """
def handleThriftNNImage(thrift_nnImage):
    nn_input_numpy = convertBinaryToImage(thrift_nnImage)

    no_channels, prediction = adtf_inference(nn_input_numpy)
    # prediction = nn_input_numpy

    # convert back to binary
    predicted_thrift_binary = nn_thrift.NNImage()
    predicted_thrift_binary.width = thrift_nnImage.width
    predicted_thrift_binary.height = thrift_nnImage.height
    predicted_thrift_binary.channels = no_channels
    predicted_thrift_binary.bytes = convertImageToBinary(prediction)
    return predicted_thrift_binary


""" convert binary data to images for the NN.
    Returned image: shape=(height, width, channels), values=float[0, 1]  """
def convertBinaryToImage(thrift_nnImage):
    height = thrift_nnImage.height
    width = thrift_nnImage.width
    channels = thrift_nnImage.channels
    bytelist = thrift_nnImage.bytes
    numpy_image = np.frombuffer(bytelist, dtype=np.uint8)
    numpy_image = np.reshape(numpy_image, (height, width, channels))

    # # to avoid read only access, do not use this line if not necessary
    # copiedImage = numpy_image.copy()
    # # draw something
    # for i in range(width):
    #     copiedImage[int(height / 2), i, 0] = 0
    #     copiedImage[int(height / 2), i, 1] = 0
    #     copiedImage[int(height / 2), i, 2] = 255

    # write to file
    # global mycounter
    # mycounter += 1
    # if(mycounter == 1):
    #     file = open("receivedImage.ppm", "w")
    #     file.write("P3 ")
    #     file.write(str(width))
    #     file.write(" ")
    #     file.write(str(height))
    #     file.write(" 255")
    #     for h in range(height):
    #         for w in range(width):
    #             for c in range(channels):
    #                 file.write(" ")
    #                 file.write(str(copiedImage[h, w, c]))

    return numpy_image


""" convert the predicted image (probably np.array) to binary """
def convertImageToBinary(image):
    return image.tobytes()  # numpy image to bytes



""" main """

def main():
    now = datetime.datetime.now()
    today = now.strftime("%Y-%m-%d_%H:%M")
    todaystring = str(today)
    print("Launched at", todaystring)

    server_handle = None
    
    host = None
    print('*** NNServer - PyTorch remote prediction ***')
    
    try:
        print('\nAutostart NNServer!\n')
        print(bcolors.OKGREEN + '> Press [Strg+C] to exit\n\n' + bcolors.ENDC)
        sys.stdout.flush()
        [server_handle, handler_handle] = createNNServer(port)
        
        if (server_handle != None):
            """ # create server in thread
            serverThread = threading.Thread(target=startNNServer, args=(server_handle,))
            serverThread.start()
            print('outside of the thread') """
            adtf_network_init()
            startNNServer(server_handle)
        else:
            print("Error while initializing.")
            sys.stdout.flush()

    except KeyboardInterrupt:
        print('exiting.')
        exit()


""" helper class for console coloring """
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


""" run main """
if __name__ == '__main__':
    main()
