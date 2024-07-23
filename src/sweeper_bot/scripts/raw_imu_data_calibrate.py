import time 
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu9250 import MPU9250

import numpy as np 

def calibrate_gyro(mpu, num_samples=1000):
    print("calibrating gyroscope: ")
    print("Please keep the sensor stationary during calibration.")


    gyro_data=[]
    for _ in range(num_samples):
        gyro_data.append(mpu.readGyroscopeMaster())
        time.sleep(0.01)

    gyro_data = np.array(gyro_data)
    gx_offset, gy_offset, gz_offset = np.mean(gyro_data, axis=0)

    print("Calibration Complete")
    print("Gyroscope offsets: gx_offset={}, gy_offset={}, gz_offset={}".format(gx_offset, gy_offset, gz_offset))

    return gx_offset, gy_offset, gz_offset

def calibrate_accel(mpu, num_samples=1000):
    print("Calibrating accelerometer: ")
    print("Please follow the instructions for each axis below: ")

    accel_offsets = [0, 0, 0]
    axis_labels = ['x', 'y', 'z']

    for axis in range(3):
        print("Orient the sensor so that {} axis is pointed against gravity.".format(axis_labels[axis]))
        input("Press Enter when ready")

        accel_data=[]
        for _ in range(num_samples):
            accel_data.append(mpu.readAccelerometerMaster()[axis])
            time.sleep(0.01)

        accel_offset = np.mean(accel_data)
        accel_offsets[axis] = accel_offset - 1

        print("{} axis offset: {}".format(axis_labels[axis], accel_offsets[axis]))

    print("Calibration complete.")
    print("Accelerometer offsets: ax_offset={}, ay_offset={}, az_offset={}".format(*accel_offsets))

    return accel_offsets

mpu = MPU9250(
    address_ak = AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_1000,
    afs=AFS_8G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C100HZ
)

# Configure the MPU9250
mpu.configure()

gx_offset, gy_offset, gz_offset = calibrate_gyro(mpu)

ax_offset, ay_offset, az_offset = calibrate_accel(mpu)

while True:
    accel_data = mpu.readAccelerometerMaster()
    gyro_data = mpu.readGyroscopeMaster()
    mag_data = mpu.readMagnetometerMaster()

    print("Gyroscope (Uncalibrated):", gyro_data)
    print("Accelerometer (Uncalibrated):", accel_data)

    gyro_data[0] -= gx_offset
    gyro_data[1] -= gy_offset
    gyro_data[2] -= gz_offset

    accel_data[0] -= ax_offset
    accel_data[0] -= ay_offset
    accel_data[0] -= az_offset

    # Print the sensor values
    print("Accelerometer (calibrated):", accel_data)
    print("Gyroscope (calibrated):", gyro_data)
    print("Magnetometer:", mag_data)

    time.sleep(1)


    

