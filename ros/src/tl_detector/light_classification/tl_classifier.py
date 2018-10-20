from styx_msgs.msg import TrafficLight
import numpy as np
import os
import sys
import tensorflow as tf
from PIL import Image
import rospy

class TLClassifier(object):
    def __init__(self):
        #Loading the TF session variables and variables for Object detection
        dir_path = os.path.dirname(os.path.realpath(__file__))
        PATH_TO_CKPT = dir_path + '/model/frozen_inference_graph.pb'
        self.TrafficLight=TrafficLight.UNKNOWN
        self.category_index={1: {'id': 1, 'name': 'Green'},
                        2: {'id': 2, 'name': 'Red'},
                        3: {'id': 3, 'name': 'Yellow'}}
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
          self.sess=tf.Session(graph=self.detection_graph)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    #helper function to identify class name
    def class_name_func(self,image_np,boxes, classes,scores,category_index):
        max_boxes_to_draw=20
        min_score_thresh=0.5
        class_name='Green'
        if not max_boxes_to_draw:
             max_boxes_to_draw = boxes.shape[0]
        for i in range(min(max_boxes_to_draw, boxes.shape[0])):
              if scores[i] > min_score_thresh:
                box = tuple(boxes[i].tolist())
                if classes[i] in category_index.keys():
                    class_name = category_index[classes[i]]['name']
        return class_name

    # helper function to perform inference operations
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        class_name='UNKNOWN'
        with self.detection_graph.as_default():
            #currently the image is read by feeding the path of the image directory
            #image_np = load_image_into_numpy_array(image)
            image_np = np.asarray(image)
            image_np_expanded = np.expand_dims(image_np, axis=0)

            (boxes, scores, classes, num) = self.sess.run(
                                   [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                                   feed_dict={self.image_tensor: image_np_expanded})

            class_name=self.class_name_func(image_np,
                                np.squeeze(boxes),
                                np.squeeze(classes).astype(np.int32),
                                np.squeeze(scores),
                                self.category_index)

        if class_name == 'Green':
                  self.TrafficLight= TrafficLight.GREEN
        elif class_name == 'Red':
                  self.TrafficLight= TrafficLight.RED
                  print('Red Light detected!')
        elif class_name == 'Yellow':
                  self.TrafficLight= TrafficLight.YELLOW
        else:
                 self.TrafficLight=TrafficLight.UNKNOWN

        return self.TrafficLight

if __name__ == '__main__':
    try:
        TLClassifier()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
