import sys
import threading
import socket
import RPi.GPIO as GPIO
import spidev
import time
import asyncio
from bleak import BleakScanner, BleakClient
import conf_parser

## JOY Settings #######
MAX_VALUE = 1023
SCALE_JOY_VALUE = 200
CAL_COUNT = 100
MIN_MOTOR_V = 100
SCALE_MOTOR_VALUE = 150

swt_gpio = [12, 5]
vrx_channel = [3, 1]
vry_channel = [2, 0]
#######################

## Calibration Data #######
center_x = [MAX_VALUE/2, MAX_VALUE/2]
center_y = [MAX_VALUE/2, MAX_VALUE/2]
###########################

spi = None
ledon = 0

async def scan():
    print("Scanning...")
    devices = await BleakScanner.discover(timeout=5.0)
    for device in devices:
        if device.name == BLE_DEVICE_NAME:
            found = await get_services(device.address)
            if found:
                print(f"device found")
                print(f" - address {device.address}")
                print(f" - name    {device.name}")
                return device.address

    return ""

async def get_services(address):
    try:
        async with BleakClient(address) as client:
            services = client.services
            for service in services:
                #print(f"Service: {service.uuid}")
                for characteristic in service.characteristics:
                    #print(f"  Characteristic: {characteristic.uuid}")
                    if characteristic.uuid == RX_CHARACTERISTIC_UUID:
                        return True
    except:
        return False

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
    #print("Key({0}) Pressed".format(channel))
    global ledon
    ledon = (ledon+1)%2

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

def main_test():
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

async def main_ble():
    global ledon
    lastmsg = ""
    last_ledon = ledon

    address = await scan()
    if address == "":
        print("No matching BLE device found")
        return

    retry = True
    while retry:
        async with BleakClient(address) as client:
            retry = False
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
                        await send_data(client, ledmsg)
                        last_ledon = ledon

            except KeyboardInterrupt:
                print("Thread exit by KeyboardInterrupt")
                break

            except socket.error:
                print("Thread exit by socket error")
                break

            except Exception as e:
                retry = True
                print(f"BT Exception : {e}")

    print('disconnect')

def main(argc, argv):
    InitSpi()
    InitGpio(0)
    InitGpio(1)
    InitJoy(0)
    InitJoy(1)

    if argc >= 2:
        if argv[1] == "test":
            main_test()
        elif argv[1] == "scan":
            if argc >= 3:
                asyncio.run(get_services(argv[2]))
            else:
                asyncio.run(scan())
        else:
            print("not valid command : {}".format(argv[1]))
    else:
        try:
            asyncio.run(main_ble())
        except KeyboardInterrupt:
            print("KeyboardInterrupt")

        except Exception as e:
            print(f"Exception : {e}")

    GPIO.cleanup()
    print("Exit")

def get_config():
    config = conf_parser.get_config("JOYSERVER")

    global RX_CHARACTERISTIC_UUID, TX_CHARACTERISTIC_UUID, BLE_DEVICE_NAME
    RX_CHARACTERISTIC_UUID = config["ble_rx_char_uuid"]
    TX_CHARACTERISTIC_UUID = config["ble_tx_char_uuid"]
    BLE_DEVICE_NAME = config["ble_device_name"]

if __name__ == '__main__':
    get_config()

    main(len(sys.argv), sys.argv)

