import math
import asyncio
from dataclasses import dataclass
from typing import Dict, Optional
from bleak import BleakClient
import rcs
import bas
from paho.mqtt import client as mqtt
import json


class DeviceNotFound(Exception):
    pass


class FailedToConnect(Exception):
    pass


@dataclass
class Location:
    x: float
    y: float
    angle: float

    def __sub__(self, other):
        return Location(self.x - other.x, self.y - other.y, self.angle - other.angle)

    def __str__(self):
        return str(self.__dict__)


locations = {"charging-station-1": Location(0, 0, 0), "charging-station-2": Location(0, 0, 0)}


async def get_and_send_status(robot: BleakClient, mqtt_client: mqtt.Client) -> None:
    id = await rcs.read_id(robot)
    status = {
        "speed": await rcs.read_speed(robot), 
        "angle": await rcs.read_angle(robot), 
        "voltage": await rcs.read_voltage(robot)
        }

    print(f"[TRACE] status = {status}")
    mqtt_client.publish(f"eums-2025/{id}/status", json.dumps(status))


async def get_and_send_battery(robot: BleakClient, mqtt_client: mqtt.Client) -> None:
    id = await rcs.read_id(robot)
    battery = {"percent": await bas.read_battery_level(robot)}

    print(f"[TRACE] battery = {battery}")
    mqtt_client.publish(f"eums-2025/{id}/battery", json.dumps(battery))


async def connect(mac_addres: str) -> BleakClient:
    device: Optional[rcs.BLEDevice] = await rcs.find(mac_addres)
    if device is None:
        raise DeviceNotFound

    robot: Optional[rcs.BleakClient] = await rcs.connect(device)
    if robot is None:
        raise FailedToConnect

    return robot


def on_location_update(client, userdata, msg) -> None:
    """Update global locations dictionary"""
    global locations

    id = msg.topic.split('/')[1]
    loc = json.loads(msg.payload)
    
    print(f"[TRACE] location {id} = {loc}")
    locations[id] = Location(loc["x"], loc["y"], loc["orientation"])


async def subscribe_location(id: int):
    print("mqtt")
    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqtt_client.on_message = on_location_update

    mqtt_client.connect("192.168.1.2")
    mqtt_client.subscribe(f"eums-2025/10/location") # robot-10 is the target robot
    mqtt_client.subscribe(f"eums-2025/{id}/location")

    mqtt_client.loop_start()

    return mqtt_client


turn_right = {"front_left": 40, "front_right": -40, "back_left": 40, "back_right": -40}
turn_left = {"front_left": -40, "front_right": 40, "back_left": -40, "back_right": 40}
forward = {"front_left": 40, "front_right": 40, "back_left": 40, "back_right": 40}


async def scale_speed(speed: Dict[str, int], scale):
    """Scale speed according to factor, clamping it to a maximal value"""
    MAX = 40
    MIN = 20
    for (k, v) in speed.items():
        dir = 1 if v > 0 else -1
        speed[k] = dir * max(min(int(v * scale), MAX), MIN)
    return speed


async def move(robot: BleakClient, target: Location):
    id = str(await rcs.read_id(robot))
    dist_threshold = 0.2 # Distance threshold in m
    angle_threshold = 20 # Angle threshold in degrees

    # Get location difference
    curr: Location = locations[id]
    delta: Location = curr - target
    print(f"[TRACE] current = {curr}")
    print(f"[TRACE] target  = {target}")

    # Calculate smallest angle to move
    angle = math.atan2(delta.y, delta.x) / math.pi * 180
    delta.angle = (angle - curr.angle + 360) % 360 - 180
    print(f"[TRACE] location delta => {delta}")

    if abs(delta.angle) < angle_threshold:
        print(f"[DEBUG] Ignoring turn, angle < {angle_threshold}m")
    else:
        speeds = (await scale_speed(turn_left, delta.angle / -180)) if delta.angle < 0 else (await scale_speed(turn_right, delta.angle / 180))
        print(f"[TRACE] drive: {speeds}")
        await rcs.set_motors(robot, speeds, 500)
        await asyncio.sleep(0.500)
        return
    
    # Move distance if needed   
    distance = math.sqrt(math.pow(delta.x, 2) + math.pow(delta.y, 2))
    if distance < dist_threshold:
        print(f"[DEBUG] Ignoring move, distance < {dist_threshold}m")
    else:
        speeds = await scale_speed(forward, distance / 0.2)
        print(f"[TRACE] drive: {speeds}")
        await rcs.set_motors(robot, speeds, 500)
        await asyncio.sleep(0.500)
        return


async def run(robot: BleakClient, mqtt_client: mqtt.Client):
    while True:
        await get_and_send_status(robot, mqtt_client)
        await get_and_send_battery(robot, mqtt_client)

        try:
            target = locations["10"]
            await move(robot, target)
        except KeyError:
            print(f"[ERROR] no data available for location")
            await asyncio.sleep(1)
        

if __name__ == "__main__":

    async def main():
        print("[INFO] Connecting...")
        robot = await connect("F0:74:F7:D8:7B:03") # replace the address with your robot address
        id = await rcs.read_id(robot)
        try:
            mqtt_client = await subscribe_location(id)
            print(f"[INFO] Connected to robot {id}")
            await run(robot, mqtt_client)
        except Exception as e:
            print(f"[ERROR] {e}")
            await robot.disconnect()

    asyncio.run(main())
