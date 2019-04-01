import time
import rospy
import rospkg

import os
import sys

import numpy as np
import tensorflow as tf
from styx_msgs.msg import TrafficLight
from io import StringIO


class TLClassifier(object):
    def __init__(self, simulator):
        # current_path = os.path.dirname(os.path.realpath(__file__))
        self.simulator_used = simulator

        # We support two different frozen graphes which are trained with
        # real car camera data and with data from the simulator. Depending
        # where the application is executed (car or simulator) different
        # models are loaded.
        if (self.simulator_used == 1):
            model_path = 'light_classification/classifiers/inference_graph_sim.pb'
        else:
            model_path = 'light_classification/classifiers/inference_graph_real.pb'

        rospy.logwarn('model path {0}'.format(model_path))

        detection_graph = self.load_graph(model_path)

        # The input placeholder for the image.
        # `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')

        # The classification of the object (integer id).
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

        self.sess = tf.Session(graph=detection_graph)


    def load_graph(self, graph_file):
        # Loads a frozen TF inference graph

        graph = tf.Graph()
        with graph.as_default():
        
            od_graph_def = tf.GraphDef()
        
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Load a sample image
        image_expanded = np.expand_dims(image, axis=0)
        result = TrafficLight.UNKNOWN
        
        # Perform detection
        (boxes, scores, classes) = self.sess.run([self.detection_boxes, self.detection_scores, 
                                            self.detection_classes], 
                                            feed_dict={self.image_tensor: image_expanded})
        # Remove unnecessary dimensions
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        # Debug classifications
        # rospy.logwarn('TF classes {0} and scores {1}'.format(classes, scores))

        # Find traffic light with highest confidence level
        conv_level = 0
        for i in range(boxes.shape[0]):
            if scores[i] > conv_level:
                conv_level = scores[i]
                if classes[i] == 2: #'Green':
                    result = TrafficLight.GREEN
                elif classes[i]  == 4: #'Red':
                    result = TrafficLight.RED
                elif classes[i]  == 3: #'Yellow':
                    result = TrafficLight.YELLOW
        
        # Debug traffic light output - Red: 0, 1: Yellow, 2: Green, 4: Unknown
        rospy.logwarn('Traffic light {0}'.format(result))

        return result