import numpy as np
from PIL import Image, ImageDraw
from scipy.integrate import quad
from scipy.optimize import curve_fit
from skimage.morphology import medial_axis

class Curve:

    def __init__(self, direction: str = None, curve_x: np.array = None, curve_y: np.array = None, params: np.array = None):
        self._parabola_direction = direction
        self._curve_x = curve_x
        self._curve_y = curve_y
        self._params = params

    @property
    def parabola_direction(self):
        return self._parabola_direction

    @parabola_direction.setter
    def parabola_direction(self, direction):
        self._parabola_direction = direction

    @property
    def curve_x(self):
        return self._curve_x

    @curve_x.setter
    def curve_x(self, value):
        self._curve_x = value

    @property
    def curve_y(self):
        return self._curve_y

    @curve_y.setter
    def curve_y(self, value):
        self._curve_y = value

    @property
    def params(self):
        return self._params

    @params.setter
    def params(self, params):
        self._params = params


    def parabola(self, t, a, b, c):
        return a * t ** 2 + b * t + c
    
    
    def curve_length_derivative(self, t):
        return (1 + (2*self._params[0]*t + self._params[1])**2)**0.5
    

    def curve_length(self, idx):
        if self._parabola_direction == 'vertical':
            return abs(quad(self.curve_length_derivative, self._curve_y[0], self._curve_y[idx])[0])
        else:
            return abs(quad(self.curve_length_derivative, self._curve_x[0], self._curve_x[idx])[0])
        

    def full_curve_length(self):
        if self._parabola_direction == 'vertical':
            return abs(quad(self.curve_length_derivative, self._curve_y[0], self._curve_y[-1])[0])
        else:
            return abs(quad(self.curve_length_derivative, self._curve_x[0], self._curve_x[-1])[0])

    
    def fit_curve_to_mask(self, mask):

        medial_img, _ = medial_axis(mask, return_distance=True)

        x, y = np.where(medial_img == 1)
        
        params1, _ = curve_fit(self.parabola, y, x)
        a, b, c = params1
        fit_curve_x = self.parabola(y, a, b, c)
        
        params2, _ = curve_fit(self.parabola, x, y)
        a, b, c = params2
        fit_curve_y = self.parabola(x, a, b, c)

        if np.linalg.norm(x - fit_curve_x) < np.linalg.norm(y - fit_curve_y):
            self._parabola_direction = 'vertical'
            self._params = params1

            # Sorted assuming that the pepper to the left of the peduncle
            # Sorting with respect to y in ascending order
            sorted_x = np.array([x for _, x in sorted(zip(y, x))])
            sorted_y = np.array([y for y, _ in sorted(zip(y, x))])
            
            self._curve_y = sorted_y
            self._curve_x = self.parabola(sorted_y, self._params[0], self._params[1], self._params[2])
        else:
            self._parabola_direction = 'horizontal'
            self._params = params2
        
            # Sorted assuming that the pepper is above the peduncle
            # Sorted with respect to x in ascending order
            sorted_x = np.array([x for x, _ in sorted(zip(x, y))])
            sorted_y = np.array([y for _, y in sorted(zip(x, y))])
            
            self._curve_x = sorted_x
            self._curve_y = self.parabola(sorted_x, self._params[0], self._params[1], self._params[2])


class PepperPeduncle:

    def __init__(self, number: int, mask=None, conf=None, percentage=0.5):
        self._number: int = number
        self._mask = mask
        self._conf: float = conf
        self._percentage = percentage
        self._xywh = None
        self._curve = Curve()
        self._poi_px = None

    @property
    def number(self):
        return self._number

    @property
    def mask(self):
        return self._mask

    @mask.setter
    def mask(self, mask):
        self._mask = mask

    @property
    def conf(self):
        if self._conf is None:
            return 0
        return self._conf

    @conf.setter
    def conf(self, conf):
        self._conf = conf

    @property
    def xywh(self):
        return self._xywh

    @xywh.setter
    def xywh(self, value):
        self._xywh = value

    @property
    def curve(self):
        return self._curve

    @curve.setter
    def curve(self, curve):
        self._curve = curve

    @property
    def poi_px(self):
        return self._poi_px

    @poi_px.setter
    def poi_px(self, poi_px):
        self._poi_px = poi_px


    def determine_poi(self, total_curve_length):
        for idx in range(len(self._curve.curve_y)):
            curve_length = self._curve.curve_length(idx)
            if abs(curve_length - self._percentage * total_curve_length) < 2:
                return self._curve.curve_x[idx], self._curve.curve_y[idx]
    
        return self._curve.curve_x[len(self._curve.curve_y) // 2], self._curve.curve_y[len(self._curve.curve_y) // 2]


    def set_point_of_interaction(self):
        self._curve.fit_curve_to_mask(self._mask)
        total_curve_length = self._curve.full_curve_length()

        poi_x_px, poi_y_px = self.determine_poi(total_curve_length)
        self._poi_px = (poi_x_px, poi_y_px)

        return self._poi_px

'''
Speed: 1.0ms preprocess, 33.6ms inference, 3.0ms postprocess per image at shape (1, 3, 640, 640)
[ERROR] [1695763655.042531]: bad callback: <bound method PerceptionNode.image_callback of <__main__.PerceptionNode object at 0x7f622e574f10>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/sridevi/Documents/iowa_ws/src/ISU_Demo_Perception/scripts/perception.py", line 105, in image_callback
    _ = self.run_yolo(cv_image)
  File "/home/sridevi/Documents/iowa_ws/src/ISU_Demo_Perception/scripts/perception.py", line 181, in run_yolo
    poi_x, poi_y = peduncle.set_point_of_interaction()
  File "/home/sridevi/Documents/iowa_ws/src/ISU_Demo_Perception/scripts/get_poi.py", line 178, in set_point_of_interaction
    self._curve.fit_curve_to_mask(self._mask)
  File "/home/sridevi/Documents/iowa_ws/src/ISU_Demo_Perception/scripts/get_poi.py", line 73, in fit_curve_to_mask
    medial_img, _ = medial_axis(mask, return_distance=True)
  File "/home/sridevi/.local/lib/python3.8/site-packages/skimage/morphology/_skeletonize.py", line 473, in medial_axis
    corner_score = _table_lookup(masked_image, cornerness_table)
  File "/home/sridevi/.local/lib/python3.8/site-packages/skimage/morphology/_skeletonize.py", line 551, in _table_lookup
    if image.shape[0] < 3 or image.shape[1] < 3:
IndexError: tuple index out of range

'''