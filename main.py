import time
import pygame as pg
from Robot import Robot

pg.init()
screen = pg.display.set_mode([500, 500])
pressed_keys = []
move_speed_val = 80
last_checked_pressed_keys = []
grab = False
up = False

robot = Robot("COM5")


def update_keys():
    global last_checked_pressed_keys
    global grab, up
    for event in pg.event.get():
        # Did the user hit a key?
        if event.type == pg.KEYDOWN:
            key = event.key
            if key not in pressed_keys:
                pressed_keys.append(key)
            if event.key == pg.K_ESCAPE:
                running = False
        elif event.type == pg.KEYUP:
            key = event.key
            if key in pressed_keys:
                pressed_keys.pop(pressed_keys.index(key))
    move_speed = [0, 0]
    fov = 0
    if pg.K_w in pressed_keys:
        fov += 1
    if pg.K_s in pressed_keys:
        fov -= 1
    move_speed[0] = fov * move_speed_val

    rot = 0
    if pg.K_a in pressed_keys:
        rot += 1
    if pg.K_d in pressed_keys:
        rot -= 1
    move_speed[1] = rot * move_speed_val
    if last_checked_pressed_keys != pressed_keys:
        last_checked_pressed_keys = pressed_keys[:]
        robot.move(*move_speed)
        if pg.K_u in pressed_keys:
            robot.lift(90 + 120 * up)
            up = not up
        if pg.K_g in pressed_keys:
            robot.grab(110 + 130 * grab)
            grab = not grab


def main_thr():
    while True:
        update_keys()
        pg.display.flip()


while 1:
    time.sleep(0.03)
    main_thr()
    # print(datafromUser[0].encode("ascii")+int(datafromUser[1]).to_bytes(2, 'big', signed=True))
