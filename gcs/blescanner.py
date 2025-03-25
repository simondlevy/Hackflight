#!/usr/bin/python3

import asyncio
from bleak import BleakScanner
from time import sleep

async def main():
    stop_event = asyncio.Event()

    # TODO: add something that calls stop_event.set()

    def callback(device, advertising_data):

        try:
            print(advertising_data)
            sleep(0.5)

        except KeyboardInterrupt:
            exit(0)

    async with BleakScanner(callback) as scanner:

        # Important! Wait for an event to trigger stop, otherwise scanner
        # will stop immediately.
        await stop_event.wait()

    # scanner stops when block exits
    ...

asyncio.run(main())
