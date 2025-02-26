"""
CapBot control over BLE.

This is a BLE central library for the "Robot Control Service" that is used for
communicating with CapBots.
"""

import asyncio
from enum import StrEnum
from typing import Dict, List, Optional

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.exc import BleakDeviceNotFoundError
from bleak.uuids import normalize_uuid_32


class CapBotUuid(StrEnum):
    """Enum with different UUIDs for our "Robot Control Service"."""

    SERVICE = normalize_uuid_32(0x00000030)  # Robot Control Service
    DRIVE = normalize_uuid_32(0x00000031)  # Motor drive characteristic
    SPEED = normalize_uuid_32(0x00000032)  # Motor speed characteristic
    ANGLE = normalize_uuid_32(0x00000033)  # Motor angle characteristic
    VOLTAGE = normalize_uuid_32(0x00000034)  # Voltage measurement characteristic
    ID = normalize_uuid_32(0x00000035)  # Identifier characteristic


async def scan() -> List[BLEDevice]:
    """Scan for BLE devices and filter out robots."""
    return await BleakScanner.discover(
        service_uuids=[CapBotUuid.SERVICE],
        timeout=5,
    )


async def find(addr: str) -> Optional[BLEDevice]:
    """Search for a robot with given address."""
    return await BleakScanner.find_device_by_address(
        addr,
        service_uuids=[CapBotUuid.SERVICE],
        timeout=5,
    )


async def connect(device: BLEDevice) -> Optional[BleakClient]:
    """Connect to the given device."""
    client = BleakClient(device, timeout=5)
    try:
        await client.connect()
        return client
    except (BleakDeviceNotFoundError, TimeoutError):
        return None


async def read_id(client: BleakClient) -> int:
    """Read the robot's capacitor voltage."""
    assert client.is_connected
    raw = await client.read_gatt_char(CapBotUuid.ID)
    assert len(raw) == 4
    return int.from_bytes(raw, 'little', signed=False)


async def read_voltage(client: BleakClient) -> float:
    """Read the robot's capacitor voltage."""
    assert client.is_connected
    raw = await client.read_gatt_char(CapBotUuid.VOLTAGE)
    assert len(raw) == 2
    return int.from_bytes(raw, 'little', signed=False) / 1000.0


async def read_angle(client: BleakClient) -> Dict[str, int]:
    """Get the angle of the robot's wheels."""
    assert client.is_connected
    raw = await client.read_gatt_char(CapBotUuid.ANGLE)
    assert len(raw) == 16
    return {
        'front_left': int.from_bytes(raw[0:4], 'little', signed=True),
        'front_right': int.from_bytes(raw[4:8], 'little', signed=True),
        'back_left': int.from_bytes(raw[8:12], 'little', signed=True),
        'back_right': int.from_bytes(raw[12:16], 'little', signed=True),
    }


async def read_speed(client: BleakClient) -> Dict[str, int]:
    """Get the speed of the robot's wheels."""
    assert client.is_connected
    raw = await client.read_gatt_char(CapBotUuid.SPEED)
    assert len(raw) == 4
    return {
        'front_left': int.from_bytes(raw[0:1], 'little', signed=True),
        'front_right': int.from_bytes(raw[1:2], 'little', signed=True),
        'back_left': int.from_bytes(raw[2:3], 'little', signed=True),
        'back_right': int.from_bytes(raw[3:4], 'little', signed=True),
    }


async def set_motors(client: BleakClient, speeds: Dict[str, int], duration: int) -> None:
    """Set the target speed of the robot's wheels."""
    assert client.is_connected
    raw = (
        speeds['front_left'].to_bytes(1, 'little', signed=True)
        + speeds['front_right'].to_bytes(1, 'little', signed=True)
        + speeds['back_left'].to_bytes(1, 'little', signed=True)
        + speeds['back_right'].to_bytes(1, 'little', signed=True)
        + duration.to_bytes(4, 'little', signed=False)
    )
    assert len(raw) == 8
    await client.write_gatt_char(CapBotUuid.DRIVE, raw, True)


# FOR DEMONSTRATION PURPOSES ONLY
if __name__ == '__main__':
    from sys import argv

    async def main() -> None:
        """CLI in order to demonstrate API usage."""
        if len(argv) == 1:
            print('Scanning for robots...')
            robots = await scan()
            for bot in robots:
                client = await connect(bot)
                if client is None:
                    print(f'[WARN] Failed to connect with {bot.address}')
                    continue
                id = await read_id(client)
                vcap = await read_voltage(client)
                print(f'\tFound {client.address} with ID: {id} (Vcap = {vcap}V)')
                await client.disconnect()
        else:
            addr = argv[1]
            print(f'Getting telemetry from robot {addr} ...')
            bot = await find(addr)
            if bot is None:
                print('[ERROR] Robot not found')
                return
            client = await connect(bot)
            if client is None:
                print('[ERROR] Failed to connect')
                return
            speeds = {'front_left': 80, 'front_right': -80, 'back_left': 80, 'back_right': -80}
            await set_motors(client, speeds, 1000)
            id = await read_id(client)
            vcap = await read_voltage(client)
            angles = await read_angle(client)
            speeds = await read_speed(client)
            print(
                f'\tID: {id}\n' f'\tVcap: {vcap}V\n' f'\tAngles: {angles}\n' f'\tSpeeds: {speeds}\n'
            )
            await client.disconnect()

    asyncio.run(main())
