import os
import cv2
from module.WebControl.base_camera import BaseCamera
from module import shared, config as conf


class Camera(BaseCamera):
    video_source = 0

    def __init__(self):
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(Camera, self).__init__()

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source

    @staticmethod
    def frames():
        #camera = cv2.VideoCapture(Camera.video_source)
        #camera.set(3, 320)
        #camera.set(4, 240)
        #camera.set(5, 20)
        #if not camera.isOpened():
        #    raise RuntimeError('Could not start camera.')

        while True:
            # read current frame
            #_, img = camera.read()

            img = shared.rawarray_to_nparray(shared.raw_image, conf.IMG_SIZE)

            # encode as a jpeg image and return it
            yield cv2.imencode('.jpg', img)[1].tobytes()
