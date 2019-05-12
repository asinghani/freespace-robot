import numpy as np
import cv2
import tensorflow as tf

import tensorflow.keras.backend as K

class Inference:
    def __init__(self, model_path):
        with tf.gfile.FastGFile(model_path, "rb") as f:
            trt_graph = tf.GraphDef()
            trt_graph.ParseFromString(f.read())

        tf_config = tf.ConfigProto()
        tf_config.gpu_options.allow_growth = True
        self.sess = tf.Session(config=tf_config)
        tf.import_graph_def(trt_graph, name="")

        self.input_tensor_name = "input_1:0"
        self.aux_input_tensor_name = "input_2:0"

        self.output_tensor = self.sess.graph.get_tensor_by_name("decoder_softmax/truediv:0")

    def infer(self, image):
        img = cv2.resize(image, (224, 224))

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
        nnInput = nnInput[:, :, ::-1].reshape((1, 224, 224, 3))
        image2 = 2 * (image2 - 0.5)
        image2 = image2.reshape((1, 112, 112, 3))

        prediction = self.sess.run(self.output_tensor, {self.input_tensor_name: nnInput, self.aux_input_tensor_name: image2})

        seg = prediction[0].argmax(axis=2).astype(np.float32)
        seg = cv2.resize(seg, (image.shape[1], image.shape[0]))
        seg = np.around(seg)

        return seg
