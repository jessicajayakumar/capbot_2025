"""Read a BLE device's battery level over BAS."""

from bleak import BleakClient
from bleak.exc import BleakCharacteristicNotFoundError
from bleak.uuids import normalize_uuid_16


async def read_battery_level(client: BleakClient) -> int:
    """Read the battery level characteristic."""
    assert client.is_connected
    raw = await client.read_gatt_char(normalize_uuid_16(0x2A19))
    return int.from_bytes(raw, 'little', signed=False)


# FOR DEMONSTRATION PURPOSES ONLY
if __name__ == '__main__':
    import asyncio

    from rcs import connect, scan

    async def main() -> None:
        """Demonstrate API usage."""
        for bot in await scan():
            client = await connect(bot)
            if client is not None:
                try:
                    battery = await read_battery_level(client)
                    print(f'Robot {client.address} has {battery}% "battery" left')
                except BleakCharacteristicNotFoundError:
                    print(f'Robot {client.address} does not implement BAS')

    asyncio.run(main())
