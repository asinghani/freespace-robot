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
from nn.models.deeplab_mobilenet import DeepLabV3_MobileNetV2

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

        self.model = DeepLabV3_MobileNetV2(config, tx2_gpu=True)
        self.model.load_weights(model_path)


        rospy.loginfo("Model loaded where input_shape = {} and output_shape = {}".format(self.model.input_shape[1:], self.model.output_shape[1:]))


    def _preprocess(self, image):
        """
        Preprocesses an OpenCV format image (0-255, BGR) into proper input format for neural network
        """

        # Scale from [0, 255] to [0, 1] and BGR to RGB 
        return (image / 255.0)[:, :, ::-1]

    def predict(self, image, visualization=False):
        """
        Runs model on an OpenCV camera image and outputs a binary segmentation map and optionally a visualization image
        """
        image = cv2.resize(self._preprocess(image), (224, 224))

        start_time = millis_time()
        predictions = self.model.predict(np.array([image]))[0]
        print("Prediction time: {}ms".format(millis_time() - start_time))

        predictions = predictions.reshape((image.shape[0], image.shape[1], 2))

        if visualization:
            vis = view_seg_map(image, predictions.argmax(axis=2), color=(0, 1, 0)) * 255

            return predictions, vis

        return predictions
