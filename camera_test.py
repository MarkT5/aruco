import cv2
import time, os
import cv2
import time, os
import numpy as np
import v4l2capture
import select

prev_time = 0


def cur_milli_time():
    return round(time.time() * 1000)


video = v4l2capture.Video_device("/dev/video0")

size_x, size_y = video.set_format(4208, 3120, fourcc='MJPG')
# size_x, size_y = video.set_format(1280, 720, fourcc='UYVY' )
print("device chose {0}x{1} res".format(size_x, size_y))

video.set_exposure_auto(1)
video.set_exposure_absolute(40)
video.set_auto_white_balance(0)
video.set_white_balance_temperature(4200)
video.create_buffers(10)
video.queue_all_buffers()
video.start()