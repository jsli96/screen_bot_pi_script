from gpiozero import *
import pigpio
import time


# -------------------Here below GPIO are using general gpio library-------
MOTOR_A_PWM = 'GPIO12'       # PWM input for extension motor
MOTOR_A_PHASE = 'GPIO5'      # Phase input for extension motor
MOTOR_B_PWM = 'GPIO13'       # PWM input for rotation motor
MOTOR_B_PHASE = 'GPIO6'      # Phase input for rotation motor
# ROTATION_C1 = 'GPIO21'     # Motor encoder C1
# ROTATION_C2 = 'GPIO20'     # Motor encoder C2
ROTATION_VCC = 'GPIO16'      # Encoder power line
IR_1 = 'GPIO23'              # IR Sensor 1
IR_2 = 'GPIO24'              # IR Sensor 2
IR_VCC = 'GPIO26'            # IR Sensor Power line

# -------------------Here below GPIO are using pigpio-------------------
ROTATION_C1 = 21             # Motor encoder C1
ROTATION_C2 = 20             # Motor encoder C2
pi = pigpio.pi()             # Initial pigpio
pi.set_mode(ROTATION_C1, pigpio.INPUT)
pi.set_mode(ROTATION_C2, pigpio.INPUT)

# --------------------Here below initial gpio function-------------------
E_MOTOR = PhaseEnableMotor(MOTOR_A_PHASE, MOTOR_A_PWM, pwm=True)  # Set up extension motor
R_MOTOR = PhaseEnableMotor(MOTOR_B_PHASE, MOTOR_B_PWM, pwm=True)  # Set up rotation motor
# ENCODER_C1 = DigitalInputDevice(ROTATION_C1)                    # Set up encoder C1
# ENCODER_C2 = DigitalInputDevice(ROTATION_C2)                    # Set up encoder C2
IR_SENSOR_1 = DigitalInputDevice(IR_1)                            # Set up IR sensor 1
IR_SENSOR_2 = DigitalInputDevice(IR_2)                            # Set up IR sensor 2
ENCODER_VCC = DigitalOutputDevice(ROTATION_VCC, initial_value=True)
IR_LED_VCC = DigitalOutputDevice(IR_VCC, initial_value=True)
POSITION = 0


# def read_encoder(gpio, level, tick):
#     global POSITION
#     if pi.read(ROTATION_C2) == 0:
#         print("low")
#     else:
#         print("high")


def pos_plus(gpio, level, tick):
    global POSITION
    POSITION = POSITION + 1
    print(POSITION)


def pos_minus(gpio, level, tick):
    global POSITION
    POSITION = POSITION - 1
    print(POSITION)


def motor_pid(input_target):
    global POSITION
    kp = 1
    kd = 0.025
    ki = 0.01
    p_time = time.time()
    p_error = 0.00
    i_error = 0.00
    callback_plus_status = False
    callback_minus_status = False
    while True:
        c_time = time.time()
        delta_t = (c_time - p_time)  # convert time from seconds to micro-seconds
        p_time = c_time
        error = POSITION - input_target
        de_dt = (p_error - error) / delta_t  # de/dt
        i_error = i_error + error * delta_t
        u = kp * error + kd * de_dt + ki * i_error
        # print('current time: ', c_time)
        print('delta_t: ', delta_t)
        print("error: ", error)
        print("de/dt: ", de_dt)
        print("i_error: ", i_error)
        print('u: ', u)
        if u > 0:
            if callback_plus_status:
                callback_plus.cancel()
                callback_plus_status = False
                print("callback minus cancelled")
            if not callback_minus_status:
                callback_minus = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, pos_minus)
                callback_minus_status = True
                print("callback plus started")
            if u > 50:
                u = 50.00
            power = u / 50
            # print('forward')
            R_MOTOR.forward(speed=power)
        else:
            if callback_minus_status:
                callback_minus.cancel()
                callback_minus_status = True
                print("callback plus cancelled")
            if not callback_plus_status:
                callback_plus = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, pos_plus)
                callback_plus_status = True
                print("callback minus started")
            if u < -50:
                u = -50
            power = u / 50 + 1
            # print('backward')
            R_MOTOR.backward(speed=power)
        # else:
        #     if callback_plus_status:
        #         callback_plus.cancel()
        #         callback_plus_status = False
        #         # print("callback plus cancelled")
        #     if callback_minus_status:
        #         callback_minus.cancel()
        #         callback_minus_status = False
        #         # print("callback minus cancelled")
        #     R_MOTOR.stop()

        if abs(error) < 10:
            R_MOTOR.forward(speed=0)
            if callback_plus_status:
                callback_plus.cancel()
                callback_plus_status = False
                # print("callback plus cancelled")
            if callback_minus_status:
                callback_minus.cancel()
                callback_minus_status = False
                # print("callback minus cancelled")
            print("Motor at position: ", POSITION)
            break

        p_error = error
        # print('Position: ', POSITION)


# def read_encoder_minus(gpio, level, tick):
#     global POSITION
#     POSITION = POSITION - 1
#
#
# def read_encoder_plus(gpio, level, tick):
#     global POSITION
#     POSITION = POSITION + 1
#
#
#
#
# while True:
#     callback_m = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, read_encoder_minus)
#     R_MOTOR.forward(speed=0.3)
#     print('Position: ', POSITION)
#     time.sleep(0.5)
#     print('Position: ', POSITION)
#     time.sleep(0.5)
#     print('Position: ', POSITION)
#     time.sleep(0.5)
#     callback_m.cancel()
#     callback_p = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, read_encoder_plus)
#     R_MOTOR.backward(speed=0.7)
#     print('Position: ', POSITION)
#     time.sleep(0.5)
#     print('Position: ', POSITION)
#     time.sleep(0.5)
#     print('Position: ', POSITION)
#     time.sleep(0.5)
#     callback_p.cancel()





def test_in_function_callback():
    callback_plus = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, pos_plus)


# callback = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, read_encoder)
# test_in_function_callback()

while True:
    # callback_plus = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, pos_plus)
    # R_MOTOR.forward(speed=0.5)
    # time.sleep(2)
    # callback_plus.cancel()
    # callback_minus = pi.callback(ROTATION_C1, pigpio.RISING_EDGE, pos_minus)
    # R_MOTOR.backward(speed=0.5)
    # time.sleep(2)
    # callback_minus.cancel()
    motor_pid(-300)
    time.sleep(2)
    # None

