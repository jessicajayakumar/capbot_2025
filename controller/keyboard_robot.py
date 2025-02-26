from typing import Tuple, Dict
from rcs import *
import rcs

import json
import paho.mqtt.client as mqtt
import asyncio

from bas import *
import asyncio
from threading import Event

from bleak import BleakClient
from bleak.backends.device import BLEDevice
from pynput import keyboard

'''
    MQTT message data format

    {
        "id": int,
        "speed": {
            'front_left': int,
            'front_right': int,
            'back_left': int,
            'back_right': int,
            },
        "angle": {
            'front_left': int,
            'front_right': int,
            'back_left': int,
            'back_right': int,
            },
        "voltage": double
    }

'''

class MQTT_connection:

    def __init__(self, id: int, client: BleakClient):
        self.id = id

        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_message = self.on_message

        self.client = client
        self.connection = True

    def on_message(self, message):
        print("Received message: ", str(message.payload.decode("utf-8")))

    async def start(self):
        self.mqttc.connect('192.168.1.2', 1883)
        self.mqttc.loop_start()

        while self.connection:

            speed = await read_speed(self.client)
            angle = await read_angle(self.client)
            voltage = await read_voltage(self.client)
            battery = await read_battery_level(self.client)
            
            status = {'speed': speed, 'angle': angle, 'voltage': voltage}
            battery_level = {'percent': battery}
            
            print('publishing...', status, battery_level)
            
            self.mqttc.publish('eums-2025/%d/status'%self.id, json.dumps(status))
            self.mqttc.publish('eums-2025/%d/battery'%self.id, json.dumps(battery_level))

            await asyncio.sleep(1)

    def stop(self):
        self.connection = False
        self.mqttc.disconnect()
        self.mqttc.loop_stop()



# Define motor parameters
async def set_robot_speed(client: BleakClient, speeds: dict):
    """Set motor speeds with a default duration."""
    await rcs.set_motors(client, speeds, 250)


# Movement commands
MOVEMENTS = {
    'w': {'front_left': 80, 'front_right': 80, 'back_left': 80, 'back_right': 80},      # forward
    'x': {'front_left': -80, 'front_right': -80, 'back_left': -80, 'back_right': -80},  # backward
    'a': {'front_left': 80, 'front_right': -80, 'back_left': -80, 'back_right': 80},    # left
    'd': {'front_left': -80, 'front_right': 80, 'back_left': 80, 'back_right': -80},    # right
    'q': {'front_left': 80, 'front_right': 0, 'back_left': 0, 'back_right': 80},        # forward_left
    'e': {'front_left': 0, 'front_right': 80, 'back_left': 80, 'back_right': 0},        # forward_right
    'z': {'front_left': 0, 'front_right': -80, 'back_left': -80, 'back_right': 0},      # backward_left
    'c': {'front_left': -80, 'front_right': 0, 'back_left': 0, 'back_right': -80},      # backward_right
    'n': {'front_left': -80, 'front_right': 80, 'back_left': -80, 'back_right': 80},    # counter_clockwise
    'm': {'front_left': 80, 'front_right': -80, 'back_left': 80, 'back_right': -80},    # clockwise
}

key_state = {key: False for key in MOVEMENTS.keys()}
stop_event = Event()


def on_press(key):
    try:
        if key.char in key_state:
            key_state[key.char] = True
            print(f'Pressed {key.char}')
    except AttributeError:
        pass


def on_release(key):
    try:
        if key.char in key_state:
            key_state[key.char] = False
            print(f'Released {key.char}')
    except AttributeError:
        if key == keyboard.Key.esc:
            stop_event.set()
            return False


async def move_robot(client: BleakClient):
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while not stop_event.is_set():
            for key, state in key_state.items():
                if state:
                    await set_robot_speed(client, MOVEMENTS[key])
                    break
            await asyncio.sleep(0.2)
    finally:
        listener.stop()

async def get_id(dev: BLEDevice) -> Tuple[int, BLEDevice] | None:
    """Connect to a robot device and read its ID."""
    client = await rcs.connect(dev)
    if client is None:
        print(f'[WARN] Could not connect to device {dev.address}')
        return None
    id = await rcs.read_id(client)
    await client.disconnect()
    return (id, dev)

# Main BLE control
async def main() -> None:
    # Scan for available robots
    print('[INFO] Scanning for robots...')
    available_robots = await rcs.scan()
    if not available_robots:
        print('[ERROR] No robots found')
        return

    # Read the ID of each available robot (concurrently)
    print('[INFO] Reading robot IDs ...')
    tasks = []
    async with asyncio.TaskGroup() as tg:
        tasks = [tg.create_task(get_id(bot)) for bot in available_robots]
    candidate_robots: Dict[int, BLEDevice] = {}
    for t in tasks:
        res = t.result()
        if res is not None:
            candidate_robots[res[0]] = res[1]
    if len(candidate_robots) == 0:
        print('[ERROR] Failed to read any IDs')
        return


    # Choose a robot in case there are multiple
    (id, device) = list(candidate_robots.items())[0]
    if len(candidate_robots) > 1:
        print('Found', len(candidate_robots), 'robot(s)')
        print('identifier | address')
        for (id,dev) in candidate_robots.items():
            print(f'{id:10d} | {dev.address}')
        id = int(input('Please select an id: '))
        device = candidate_robots[id]

    # (Re)connect to chosen robot
    print(f'[INFO] Connecting to CapBot {id} ...')
    client = await rcs.connect(device)
    if client is None:
        print(f'[ERROR] Failed to connect to chosen robot ({device.address})')
        return

    # Connect to MQTT broker
    mqtt_connection = MQTT_connection(id=id, client=client)

    # Create two tasks: one for keyboard based control, one for pushing telemetry to MQTT
    move_task = asyncio.create_task(move_robot(client))
    mqtt_task = asyncio.create_task(mqtt_connection.start())

    # Wait for keyboard control task to finish
    await move_task

    # Cleanup on exit
    mqtt_connection.stop()
    await mqtt_task
    await client.disconnect()

if __name__ == '__main__':
    asyncio.run(main())
