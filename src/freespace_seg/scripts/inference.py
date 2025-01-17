import numpy as np
import cv2
import rospy
import time

import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.backend import set_session, set_floatx
from tensorflow.keras.layers import DepthwiseConv2D

from nn.vis import view_seg_map

from nn.config import Config
from nn.models.floornet import FloorNet

from tqdm import tqdm

millis_time = lambda: int(round(time.time() * 1000))

# Transfer weights from src to model while fixing dilated convolutions
def transfer_weights(src, model):
    for layer in tqdm(src.layers):
        if isinstance(layer, DepthwiseConv2D) and (layer.dilation_rate[0] > 1):
            weights = layer.get_weights()[0]

            x = 3 + (2 * (layer.dilation_rate[0] - 1))
            new_weights = np.zeros((x, x, weights.shape[2], 1), dtype=weights.dtype)
            new_weights[::(layer.dilation_rate[0]), ::(layer.dilation_rate[0])] = weights
            model.get_layer(layer.name).set_weights([new_weights])
        else:
            try:
                model.get_layer(layer.name).set_weights(layer.get_weights())
            except Exception as e:
                print(e, layer.name)


class Inference:
    """
    Class for running inference on a model with image input (RGB channel ordering) and segmentation softmax output.
    """
    def __init__(self, input_shape, model_path):
        rospy.loginfo("Loading model weights from %s", model_path)

        config = Config()
        config.init_weights = None
        config.image_size = input_shape
        config.input_shape = (config.image_size[0], config.image_size[1], 3)

        #set_floatx("float16")

        tf_config = tf.ConfigProto()
        tf_config.gpu_options.allow_growth = True
        session = tf.Session(config=tf_config)
        set_session(session)

        self.model = FloorNet(config)
        self.model.load_weights(model_path)


        rospy.loginfo("Model loaded where input_shape = {} and output_shape = {}".format(self.model.input_shape[1:], self.model.output_shape[1:]))


    def _preprocess(self, img):
        img2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        x = cv2.Sobel(img2, cv2.CV_32F, 1, 0, ksize=3)
        x = cv2.convertScaleAbs(x) / 255.0
        x = x.reshape((x.shape[0], x.shape[1], 1))
        y = cv2.Sobel(img2, cv2.CV_32F, 0, 1, ksize=3)
        y = cv2.convertScaleAbs(y) / 255.0
        y = y.reshape((y.shape[0], y.shape[1], 1))
        z = cv2.Laplacian(img2, cv2.CV_32F)
        z = cv2.convertScaleAbs(z) / 255.0
        z = z.reshape((z.shape[0], z.shape[1], 1))
        image2 = np.concatenate((x, y, z), axis=2)
        image2 = cv2.resize(image2, (112, 112))
        nnInput = np.array(img, dtype=np.float32) / 255.0

        nnInput = 2 * (nnInput - 0.5)
        image2 = 2 * (image2 - 0.5)

        return [nnInput[:, :, ::-1].reshape((1, 224, 224, 3)), image2.reshape((1, 112, 112, 3))]

    def predict(self, image, visualization=False):
        """
        Runs model on an OpenCV camera image and outputs a binary segmentation map and optionally a visualization image
        """
        image = cv2.resize(image[int(image.shape[0] * 0.375):], (224, 224))
        input = self._preprocess(image)

        start_time = millis_time()
        predictions = self.model.predict(input)[0]
        print("Prediction time: {}ms".format(millis_time() - start_time))

        predictions = predictions.reshape((image.shape[0], image.shape[1], 2))

        if visualization:
            vis = view_seg_map(image, predictions.argmax(axis=2), color=(0, 1, 0)) * 255

            return predictions, vis

        return predictions
