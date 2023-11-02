#!/home/allan/catkin_ws/src/allan_husky/myvenv/bin/python3
import rospy
import tensorflow as tf
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2
from erfnet import ERFNet
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import time
from sensor_msgs.msg import Image

class SemanticSegmentation:
    def __init__(self):
        self.model = self.load_model()
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber('/image_publisher/image_raw', Image, self.image_callback)
        # Create publisher for mask
        self.mask_pub = rospy.Publisher('/image_raw_bin', Image, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        rospy.loginfo('Image received')
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            img = cv2.resize(img, (640, 480))
            cv2.imshow('original', img)
            cv2.waitKey(1)
        except Exception as e:
            print(e)
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        mask = self.predict(image)
        mask = np.array(mask, dtype=np.uint8)
        # convert to 0-255 range
        mask *= 255
        mask = cv2.resize(mask, (640, 360))
        mask_msg = Image()
        mask_msg.header.stamp = rospy.Time.now()
        mask_msg.height = mask.shape[0]
        mask_msg.width = mask.shape[1]
        mask_msg.encoding = 'mono8'
        mask_msg.step = mask.shape[1]
        mask_msg.data = mask.tobytes()
        self.mask_pub.publish(mask_msg)

    def load_model(self):
        erf = ERFNet(input_shape=(176, 320, 3))
        model = erf.build()
        dirname = os.path.dirname(__file__)
        model.load_weights(os.path.join(dirname, '20231018_after_ramp.h5'))
        full_model = tf.function(lambda x: model(x))
        full_model = full_model.get_concrete_function(tf.TensorSpec(model.inputs[0].shape, model.inputs[0].dtype))
        frozen_func = convert_variables_to_constants_v2(full_model)
        frozen_func.graph.as_graph_def()
        return frozen_func

    def predict(self, image):
        img = cv2.resize(image, (320, 176)) / 255.0
        img = np.array(img, dtype=np.float32)
        input_tensor = np.expand_dims(img, 0)
        input_tensor = tf.convert_to_tensor(input_tensor)
        return self.model(input_tensor)[0][0]

if __name__ == '__main__':
    rospy.init_node('semantic_segmentation_node')
    semantic_segmentation = SemanticSegmentation()
    rospy.loginfo('.:: Semantic Segmentation Node Started ::.')
    rospy.spin()


    