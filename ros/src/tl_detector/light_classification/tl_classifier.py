# Standard imports
import os

# Local imports
from styx_msgs.msg import TrafficLight

# Dependecy imports
import numpy as np
import tensorflow as tf
import rospy

from keras.models import model_from_json
import keras.backend.tensorflow_backend as K

K_CONFIG = K.tf.ConfigProto()
K_CONFIG.allow_soft_placement = True
K_CONFIG.gpu_options.allow_growth = True
K.set_session(K.tf.Session(config=K_CONFIG))

class _TLClassifier(object):
    """By give image inpy detect and classify traffic lights.

    Possible states:
        TrafficLight.RED
        TrafficLight.YELLOW
        TrafficLight.GREEN
        TrafficLight.UNKNOWN

    """

    def __init__(self, sim=True):

        self.model_dir = '/'.join(os.path.abspath(__file__).split('/')[0:-5]) + '/'

        self.detection_graph = None # Loaded and then used for inference

        self.sim = sim

        # TF Tensors
        self.image_tensor = None
        self.detection_boxes = None
        self.detection_scores = None
        self.detection_classes = None

        # Load graph, session and tensors as class variables
        self._load_graph()

    def color_detector(self, image):
        """
        Detect color of traffic light based on pixel values.

        image: traffic light cropped from image
        """

        states = [TrafficLight.GREEN, TrafficLight.YELLOW, TrafficLight.RED]

        best_state = TrafficLight.UNKNOWN
        for state in states:

            _image = image.copy()

            if state == TrafficLight.RED:
                _image = _image[int(image.shape[0] * 0.2) - 3: int(image.shape[0] * 0.2) + 3,
                                int(image.shape[1] * 0.5) - 3: int(image.shape[1] * 0.5) + 3]
            if state == TrafficLight.YELLOW:
                _image = _image[int(image.shape[0] * 0.55) - 3: int(image.shape[0] * 0.55) + 3,
                                int(image.shape[1] * 0.5) - 3: int(image.shape[1] * 0.5) + 3]
            if state == TrafficLight.GREEN:
                _image = _image[int(image.shape[0] * 0.85) - 3: int(image.shape[0] * 0.85) + 3,
                                int(image.shape[1] * 0.5) - 3: int(image.shape[1] * 0.5) + 3]

            if _image[np.where(np.squeeze(_image) > 250)].shape[0] > 15:
                best_state = state

        return best_state

    def crop_bbox(self, img, bbox, extend_x=0, extend_y=0):
        """Crop ROI from bounding boxes."""
        im_shape = img.shape

        y_0, x_0, y_1, x_1 = bbox

        x_0 = int(img.shape[1] * x_0)
        x_1 = int(img.shape[1] * x_1)
        y_0 = int(img.shape[0] * y_0)
        y_1 = int(img.shape[0] * y_1)

        y_start = y_0 - extend_y
        y_end = y_1 + extend_y
        x_start = x_0 - extend_x
        x_end = x_1 + extend_x

        if y_start <= 0:
            y_start = 0
        if y_end >= im_shape[0]:
            y_end = im_shape[0]
        if x_start <= 0:
            x_start = 0
        if x_end >= im_shape[1]:
            x_end = im_shape[1]

        crop = img[y_start: y_end, x_start: x_end]

        return crop

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        bboxes = self._locate_traffic_lights(image)

        color_count = {}
        for bbox in bboxes:

            traffic_light_crop = self.crop_bbox(image, bbox)
            if traffic_light_crop.shape[0] > 3 * 7 and traffic_light_crop.shape[1] > 7:
                state = self.color_detector(traffic_light_crop)
            else:
                continue

            if state != TrafficLight.UNKNOWN:
                if state not in color_count:
                    color_count[state] = 1
                else:
                    color_count[state] += 1

        if color_count:
            return max(color_count, key=color_count.get)
        else:
            return TrafficLight.UNKNOWN

    def _load_graph(self):
        """Load protobuf Tensorflow graph with weights, session and tensors"""

        model_path = self.model_dir + 'object_detection/frozen_inference_graph.pb'

        rospy.loginfo("model_path %s", model_path)

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.log_device_placement = True

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default(): # pylint: disable=E1129

            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_path, 'rb') as fid:

                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=self.detection_graph, config=config)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')


    def _locate_traffic_lights(self, image):
        """Run traffic light TF detection.

        image: numpy array, uint8
        """

        image_np_expanded = np.expand_dims(image, axis=0)

        (boxes, scores, classes) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes],
            feed_dict={self.image_tensor: image_np_expanded}
        )

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        # Select only traffic lights from all possible classes and with confidence higher than 0.8
        boxes = boxes[(classes == 10) & (scores > 0.65)]

        return boxes


class TLClassifier(object):
    """By give image inpy detect and classify traffic lights.

    Possible states:
        TrafficLight.RED
        TrafficLight.YELLOW
        TrafficLight.GREEN
        TrafficLight.UNKNOWN

    """

    def __init__(self):

        self.model_dir = '/'.join(os.path.abspath(__file__).split('/')[0:-5]) + '/'

        self.graph = None

        # Load graph, session and tensors as class variables
        self._load_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            state = round(self.model.predict(np.expand_dims(image, 0))[0, 0])

        rospy.loginfo('STATE: %s', state)
        if state == 0:
            return TrafficLight.RED
        else:
            return TrafficLight.GREEN

    def _load_graph(self):
        """Load Keras model."""

        if self.sim:
            model_path = self.model_dir + 'object_detection/kerasmodel'
        else:
            model_path = self.model_dir + 'object_detection/keras_udacitymodel'

        self.model = model_from_json(open(model_path + '.json', 'r').read())
        self.model.load_weights(model_path + '.h5')

        self.graph = tf.get_default_graph()
