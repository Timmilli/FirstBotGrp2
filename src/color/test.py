import odom, control
import random


NB_OF_TESTS = 100
MAX_SPEED = 100.
EPSILON = 0.0000001
speeds = [(random.random() * MAX_SPEED, random.random() * MAX_SPEED) for _ in range(NB_OF_TESTS)]

for i in range(NB_OF_TESTS):
    (v_droit, v_gauche) = speeds[i]
    (linear_speed, angular_speed) = odom.direct_kinematics(v_droit, v_gauche)
    (calculated_v_droit, calculated_v_gauche) = control.inverse_kinematics(linear_speed, angular_speed)
    if (abs(v_droit - calculated_v_droit) > EPSILON or abs(v_gauche - calculated_v_gauche) > EPSILON):
        print(f"Error for values {v_droit}, {v_gauche}.")
        print(f"\tGot {calculated_v_droit}, {calculated_v_gauche}")

control.go_to_xya(10, 20, 4)
