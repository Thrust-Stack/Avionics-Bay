import time

accel_data = []
start_time = time.time()

while True:
    elapsed_time = time.time() - start_time

    ax = get_accel_x()
    ay = get_accel_y()
    az = get_accel_z()

    accel_data.append([elapsed_time, ax, ay, az])