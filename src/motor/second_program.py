import time
import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1])
# dxl_io.set_moving_speed({2: 0})  # Degrees / s


def goto(x, y, theta):
    # Wheel diameter : 52mm
    # Wheel to wheel : 152mm
    # On one turn : 2 PI 52 / 360 mm/°
    dxl_io.set_moving_speed({1: 0})  # Degrees / s
    dxl_io.set_moving_speed({2: 0})  # Degrees / s

    time.sleep(1)


if __name__ == '__main__':
    while True:
        distance = input("Distance à faire: (mm)")
        if distance > 0:
            goto(distance)
        else:
            break
