import sys
import socket
import threading
import RPi.GPIO as GPIO
import spidev
import time
import conf_parser

TEST_MODE = False

BUF_SIZE = 1024

MAX_VALUE = 1023
SCALE_JOY_VALUE = 200
CAL_COUNT = 100
MIN_MOTOR_V = 100
SCALE_MOTOR_VALUE = 150

swt_gpio = [12, 5]
vrx_channel = [3, 1]
vry_channel = [2, 0]

spi = None
condition = None

ledon = 0

## Calibration Data #######
center_x = [MAX_VALUE/2, MAX_VALUE/2]
center_y = [MAX_VALUE/2, MAX_VALUE/2]
###########################

def recv_data(sock):
    while True:
        try:
            data = sock.recv(BUF_SIZE)

            if not data:
                print("Disconnected by no data")
                break

            str = data.decode()

            with condition:
                condition.notify_all()

        except socket.error:
            print("Thread exit by socket error")
            break

        except ConnectionResetError:
            print("Disconnected by exception")
            break

    with condition:
        condition.notify_all()

def SpiRead(channel):
    global spi

    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def Calibration(channel):
    i = 0
    sum_value = 0
    while i < CAL_COUNT:
        sum_value += SpiRead(channel)
        i += 1

    return int(sum_value/CAL_COUNT)


def Reverse(value):
    return MAX_VALUE - value

def Adjust(x, y, center_x, center_y):
    if x < center_x:
        new_x = GetCalValue(x, center_x, True)
    else:
        new_x = GetCalValue(x, center_x, False)

    if y < center_y:
        new_y = GetCalValue(y, center_y, True)
    else:
        new_y = GetCalValue(y, center_y, False)

    return new_x, new_y

def GetCalValue(value, center, lower):
    if lower:
        new_value = (value/center)*(MAX_VALUE/2)
    else:
        new_value = ((value-center)/(MAX_VALUE-center))*(MAX_VALUE/2)+(MAX_VALUE/2)

    return int(new_value)

def Scale(x, y):
    new_x = int((x/MAX_VALUE)*SCALE_JOY_VALUE)
    new_y = int((y/MAX_VALUE)*SCALE_JOY_VALUE)

    if new_x%10 < 5:
        new_x = new_x - (new_x%10)
    else:
        new_x = new_x + (10-(new_x%10))

    if new_y%10 < 5:
        new_y = new_y - (new_y%10)
    else:
        new_y = new_y + (10-(new_y%10))

    return new_x, new_y

def GpioIsrHandler(channel):
    print("Key({0}) Pressed".format(channel))
    global ledon
    ledon = (ledon+1)%2

def InitSocket():
    try:
        print("Trying to connect to server")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((DEFAULT_HOST, DEFAULT_PORT))

        client = threading.Thread(target=recv_data, args=(sock,))
        client.start()
        print("Connect to server")

        # Wait for "connected" msg
        with condition:
            condition.wait()

        return sock, client

    except socket.error as e:
        print("Exit by socket error : host {0}, port {1}".format(DEFAULT_HOST, DEFAULT_PORT))
        sys.exit(1)

def InitSpi():
    global spi

    spi = spidev.SpiDev()
    spi.open(1,0)
    spi.max_speed_hz=4000000

def InitGpio(joynum):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(swt_gpio[joynum], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(swt_gpio[joynum], GPIO.FALLING, callback=GpioIsrHandler, bouncetime=200)

def InitJoy(joynum):
    global center_ch0_x, center_ch0_y, center_ch1_x, center_ch1_y

    center_x[joynum] = Calibration(vrx_channel[joynum])
    center_y[joynum] = Calibration(vry_channel[joynum])

    print("New center after calibration :")
    print("[JOY{}] ({}, {})".format(joynum, center_x[joynum], center_y[joynum]))

def ReadJoy(joynum):
    vrx_pos = SpiRead(vrx_channel[joynum])
    vry_pos = SpiRead(vry_channel[joynum])
    vrx_pos_cal, vry_pos_cal = Adjust(vrx_pos, vry_pos, center_x[joynum], center_y[joynum])
    vrx_pos_scale, vry_pos_scale = Scale(Reverse(vrx_pos_cal), Reverse(vry_pos_cal))

    return vrx_pos_scale, vry_pos_scale

def GetJoyValue():
    JOY_CENTER = SCALE_JOY_VALUE/2
    SCALE_RATIO = SCALE_MOTOR_VALUE/JOY_CENTER

    vrx_pos, vry_pos = ReadJoy(0)

    if vrx_pos < JOY_CENTER:
        direction = "l"
        speed = int((JOY_CENTER - vrx_pos) * SCALE_RATIO) + MIN_MOTOR_V
    elif vrx_pos > JOY_CENTER:
        direction = "r"
        speed = int((vrx_pos - JOY_CENTER) * SCALE_RATIO) + MIN_MOTOR_V
    else:
        vrx_pos, vry_pos = ReadJoy(1)
        if vry_pos < JOY_CENTER:
            direction = "f"
            speed = int((JOY_CENTER - vry_pos) * SCALE_RATIO) + MIN_MOTOR_V
        elif vry_pos > JOY_CENTER:
            direction = "b"
            speed = int((vry_pos - JOY_CENTER) * SCALE_RATIO) + MIN_MOTOR_V
        else:
            direction = "f"
            speed = 0

    return speed, direction

def main(argc, argv):
    global TEST_MODE, condition, ledon
    if argc >= 2:
        if argv[1] == "test":
            TEST_MODE = True

    if not TEST_MODE:
        condition=threading.Condition()
        client_socket, client_thread = InitSocket()

    InitSpi()
    InitGpio(0)
    InitGpio(1)
    InitJoy(0)
    InitJoy(1)

    lastmsg = ""
    last_ledon = ledon

    while True:
        if not TEST_MODE and not client_thread.is_alive():
            break

        try:
            speed, direction = GetJoyValue()

            msg = "{0}{1}".format(speed, direction)
            if msg != lastmsg:
                print(msg)
                if not TEST_MODE:
                    client_socket.send(msg.encode())

                    #with condition:
                    #    condition.wait()

                lastmsg = msg

            if ledon != last_ledon:
                ledmsg = "{0}z".format(ledon)
                print(ledmsg)
                if not TEST_MODE:
                    client_socket.send(ledmsg.encode())
                    last_ledon = ledon

                    with condition:
                        condition.wait()

        except KeyboardInterrupt:
            print("Thread exit by KeyboardInterrupt")
            break

        except socket.error:
            print("Thread exit by socket error : ")
            break

    GPIO.cleanup()
    if not TEST_MODE:
        print("socket closed")
        client_socket.close()

def get_config():
    config = conf_parser.get_config("JOYSERVER")

    global DEFAULT_HOST, DEFAULT_PORT
    DEFAULT_HOST = config["host_addr"]
    DEFAULT_PORT = int(config["host_port_num"])

if __name__ == '__main__':
    get_config()

    main(len(sys.argv), sys.argv)
