# run this program on each RPi to send a labelled image stream
import socket
import time
from imutils.video import VideoStream
import imagezmq


try:
    sender = imagezmq.ImageSender(connect_to='tcp://192.168.1.32:5555')

    rpi_name = socket.gethostname()  # send RPi hostname with each image
    picam = VideoStream(usePiCamera=True).start()
    time.sleep(2.0)  # allow camera sensor to warm up
    while True:  # send images as stream until Ctrl-C
        image = picam.read()
        sender.send_image(rpi_name, image)
except KeyboardInterrupt:
    pass
