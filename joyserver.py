import sys
import threading
import socket
import RPi.GPIO as GPIO
import spidev
import time
import asyncio
from bleak import BleakScanner, BleakClient

## BLE Settings #######
BLE_DEV_ADDRESS = "3C:8A:1F:C1:DC:D2"
RX_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
TX_CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#######################

## WIFI Settings ######
DEFAULT_HOST = "192.168.219.113"
DEFAULT_PORT = 9000
BUF_SIZE = 1024
#######################

## JOY Settings #######
MAX_VALUE = 1023
SCALE_JOY_VALUE = 200
CAL_COUNT = 100
MIN_MOTOR_V = 100
SCALE_MOTOR_VALUE = 150

swt_gpio = [12, 5]
vrx_channel = [0, 2]
vry_channel = [1, 3]
#######################

## Calibration Data #######
center_x = [MAX_VALUE/2, MAX_VALUE/2]
center_y = [MAX_VALUE/2, MAX_VALUE/2]
###########################

spi = None
condition = threading.Condition()
ledon = 0

async def scan():
    print("Scanning...")
    devices = await BleakScanner.discover()
    for device in devices:
        print(device)

async def get_services(address):
    async with BleakClient(address) as client:
        services = client.services
        for service in services:
            print(f"Service: {service.uuid}")
            for characteristic in service.characteristics:
                print(f"  Characteristic: {characteristic.uuid}")

async def send_data(client, data):
    byte_data = data.encode("utf-8")
    await client.write_gatt_char(RX_CHARACTERISTIC_UUID, byte_data)
    print(f"[BT] Sent: {data}")

async def receive_data(client):
    def notification_handler(sender, data):
        print(f"Received: {data}")

    client.start_notify(TX_CHARACTERISTIC_UUID, notification_handler)
    time.sleep(1)
    client.stop_notify(TX_CHARACTERISTIC_UUID)

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

    print("New center after calibration for JOY{} : ({}, {})".format(joynum, center_x[joynum], center_y[joynum]))

def ReadJoy(joynum):
    vrx_pos = SpiRead(vrx_channel[joynum])
    vry_pos = SpiRead(vry_channel[joynum])
    vrx_pos_cal, vry_pos_cal = Adjust(vrx_pos, vry_pos, center_x[joynum], center_y[joynum])
    vrx_pos_scale, vry_pos_scale = Scale(Reverse(vrx_pos_cal), vry_pos_cal)

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

def main_test():
    InitSpi()
    InitGpio(0)
    InitGpio(1)
    InitJoy(0)
    InitJoy(1)

    lastmsg = ""
    while True:
        try:
            speed, direction = GetJoyValue()

            msg = "{0}{1}".format(speed, direction)
            if msg != lastmsg:
                print(msg)

                lastmsg = msg

        except KeyboardInterrupt:
            print("Thread exit by KeyboardInterrupt")
            break

    GPIO.cleanup()

async def main():
    global ledon

    client_socket, client_thread = InitSocket()
    InitSpi()
    InitGpio(0)
    InitGpio(1)
    InitJoy(0)
    InitJoy(1)

    lastmsg = ""
    last_ledon = ledon

    address = BLE_DEV_ADDRESS
    async with BleakClient(address) as client:
        try:
            if not client.is_connected:
                print(f"Failed to connect to {address}")

            print(f"connected: {client.is_connected}")

            while client.is_connected:
                speed, direction = GetJoyValue()

                msg = "{0}{1}".format(speed, direction)
                if msg != lastmsg:
                    await send_data(client, msg)
                    lastmsg = msg

                if ledon != last_ledon:
                    ledmsg = "{0}z".format(ledon)
                    print(f"[WIFI] Sent: {ledmsg}")
                    client_socket.send(ledmsg.encode())
                    last_ledon = ledon

                    with condition:
                        condition.wait()

        except KeyboardInterrupt:
            print("Thread exit by KeyboardInterrupt")

        except socket.error:
            print("Thread exit by socket error")

        except Exception as e:
            print(f"Exception : {e}")

    print('disconnect')

    GPIO.cleanup()

if __name__ == '__main__':
    argc = len(sys.argv)
    argv = sys.argv
    if argc >= 2:
        if argv[1] == "test":
            main_test()
        elif argv[1] == "scan":
            asyncio.run(scan())
        else:
            print("not valid command : {}".format(argv[1]))
    else:
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            print("KeyboardInterrupt")

        except Exception as e:
            print(f"Exception : {e}")





