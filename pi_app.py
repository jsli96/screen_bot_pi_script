import time
import socketio
import cv2 as cv
import base64
from gpiozero import *
# from picamera import Picamera
MOTOR_A_PWM = 'GPIO12'     # PWM input for extension motor
MOTOR_A_PHASE = 'GPIO5'    # Phase input for extension motor
MOTOR_B_PWM = 'GPIO13'     # PWM input for rotation motor
MOTOR_B_PHASE = 'GPIO6'    # Phase input for rotation motor
ROTATION_C1 = 'GPIO21'     # Motor encoder C1
ROTATION_C2 = 'GPIO20'     # Motor encoder C2
ROTATION_VCC = 'GPIO16'    # Encoder power line
IR_1 = 'GPIO23'            # IR Sensor 1
IR_2 = 'GPIO24'            # IR Sensor 2
IR_VCC = 'GPIO26'          # IR Sensor Power line

# camera = Picamera()
URL_LOCAL = 'http://127.0.0.1:5000/'
URL_CLOUD = 'https://screen-bot-proj.herokuapp.com/'
E_MOTOR = PhaseEnableMotor(MOTOR_A_PHASE, MOTOR_A_PWM, pwm=True)    # Set up extension motor
R_MOTOR = PhaseEnableMotor(MOTOR_B_PHASE, MOTOR_B_PWM, pwm=True)    # Set up rotation motor
ENCODER_C1 = DigitalInputDevice(ROTATION_C1)    # Set up encoder C1
ENCODER_C2 = DigitalInputDevice(ROTATION_C2)    # Set up encoder C2
IR_SENSOR_1 = DigitalInputDevice(IR_1)                 # Set up IR sensor 1
IR_SENSOR_2 = DigitalInputDevice(IR_2)                 # Set up IR sensor 2
ENCODER_VCC = DigitalOutputDevice(ROTATION_VCC, initial_value=True)
IR_LED_VCC = DigitalOutputDevice(IR_VCC, initial_value=True)
sio = socketio.Client()
POSITION = 0


def send_img():
    img = cv.imread("photo/test.png", cv.IMREAD_GRAYSCALE)  # Read first image
    img = cv.resize(img, (0, 0), fx=0.5, fy=0.5)
    size, data = cv.imencode('.png', img)
    data = base64.b64encode(data)
    sio.emit('img_data', data)


def motor_pid(input_target):
    ENCODER_C1.when_activated = read_encoder
    global POSITION
    kp = 1
    kd = 0.025
    ki = 0.01
    p_time = 0.00
    p_error = 0.00
    i_error = 0.00

    while True:
        c_time = time.time()
        # print('current time: ', c_time)
        delta_t = (c_time - p_time)
        p_time = c_time
        error = input_target - POSITION
        # print("error: ", error)
        de_dt = (error - p_error) / delta_t     # de/dt
        i_error = i_error + error * de_dt
        u = kp * error + kd * de_dt + ki * i_error
        # print('u: ', u)
        if u > 0:
            if u > 255:
                u = 255.00
            power = u / 255
            R_MOTOR.forward(speed=power)
        else:
            if u < -255:
                u = -255
            power = u / 255 + 1
            R_MOTOR.backward(speed=power)
        if abs(error) < 10:
            break
        p_error = error


def read_encoder():
    global POSITION
    b = ENCODER_C2.value
    print('b: ', b)
    if b > 0:
        POSITION = POSITION + 1
    else:
        POSITION = POSITION - 1
    print('Position: ', POSITION)


@sio.event
def connect():
    print('my sid is: ' + sio.sid)
    print('connection established')


@sio.event
def connect_error(error):
    print(error)


@sio.event
def disconnect():
    print("disconnected")


@sio.on('receive finished')
def disconnect():
    sio.disconnect()


@sio.on('request img')
def start_send_img(data):
    print(data)
    send_img()
    return 'OK'


# sio.connect('http://127.0.0.1:5000/')
# sio.emit('This is test in main function', "It\'s me")
# sio.wait()
# ENCODER_C1.when_activated = read_encoder
# motor_pid(300)

pos = 0
print("System ready!\n")

while True:
    # if IR_SENSOR_1.value == 0:
    #     pos = pos + 1
    #     print(pos)
    print("IR1: ", IR_SENSOR_1.value)
    # print("IR2: ", IR_SENSOR_2.value)
    time.sleep(1)





