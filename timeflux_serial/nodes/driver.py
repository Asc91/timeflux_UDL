import serial
import time
import numpy as np
import pandas as pd
from timeflux.core.exceptions import WorkerInterrupt
from timeflux.helpers import clock
from timeflux.core.node import Node
from threading import Thread, Lock
from datetime import timezone
import datetime



class SerialDevice(Node):

    """A generic serial device driver.

    Attributes:
        o (Port): Stream from the serial device, provides DataFrame.

    Args:
        port (string): The serial port.
            e.g. ``COM3`` on Windows;  ``/dev/tty.tty.SLAB_USBtoUART`` on MacOS;
            ``/dev/ttyUSB0`` on GNU/Linux.
        channels(int): No. of Channels. Default 1, Arduino-> A0, ESP32-> GPIO 32
        names (list of string): The names of channels. Ex: ['C1', 'C2' ..]
            Default: Channels are numbered from 1. 
        rate (int): The device rate in Hz.
            Max: 1600Hz, Default: 250Hz

    Example:
        .. literalinclude:: /../test/graphs/test.yaml
           :language: yaml

    Notes:

    .. attention::

        Make sure to set your graph rate to an high-enough value, otherwise the device
        internal buffer may saturate, and data may be lost. A 30Hz graph rate is
        recommended for a 1000Hz device rate.

    """
    
    def __init__(self, port, channels = 1,names = None, rate=250):
        
        # Check port
        if not port.startswith('/dev/') and not port.startswith('COM'):
            raise ValueError(f'Invalid serial port: {port}')

        # Check rate
        if not isinstance(rate, int) or rate > 1600:
            raise ValueError(f'Invalid rate: {rate}')
        self._rate = rate

        # Connect to device
        global serialdevice
        serialdevice = self._connect(port)
       
        
        self._channels = channels
        self._send(f'channels, {channels}')

        # Set channel names
        if isinstance(names, list) and len(names) == channels:
            self.names = names
        else:
            index = list(range(1, channels + 1))
            name = []
            for i in index:
                name.append(i)
            self.names = name


        # Set rate
        self._send(f'rate,{rate}')
        # Set meta
        self.meta = {"rate": rate}

        # Start Acquisition
        self._send('start')

        # Initialize counters for timestamp indices and continuity checks
        self._sample_counter = 0
        
        
        # Launch background thread
        self._reset()
        self._lock = Lock()
        self._running = True
        self._thread = Thread(target=self._loop).start()
    
    def _connect(self, port):
        device = serial.Serial(port, 115200)
        msg = device.readline(); # Wait for 'ready\n'
        try:
            msg.decode()
            print(f"ready")
        except UnicodeDecodeError:
            self.logger.error('Unstable state. Please re-plug the device and start again.')
            # https://stackoverflow.com/questions/21073086/wait-on-arduino-auto-reset-using-pyserial
            # https://forum.arduino.cc/index.php?topic=38981.0
            raise WorkerInterrupt()
        return device

    def _reset(self):
        """Empty cache."""
        self._rows = []
        self._timestamps = []


    def _loop(self):
        """Acquire and cache data."""
        while self._running:
            try:
                row = self._read()
                if len(row):
                    self._check(row[0])
                    timestamp = np.datetime64(int(time() * 1e6), "us")

                    self._lock.acquire()  # `with self.lock:` is about twice as slow
                    self._timestamps.append(timestamp)
                    self._rows.append(row[2])
                    self._lock.release()
                elif len(row) == 0:
                    print(f"error")
            except:
                pass

    def _send(self, cmd):
        cmd = (cmd + '\n').encode()
        global serialdevice
        serialdevice.write(cmd)

    def _read(self):
        """Read a line of data from the device."""
        row = []
        global serialdevice
        buffersize = serialdevice.in_waiting
        
        serialData = serialdevice.read_until()
        print(f"buffer: {buffersize} data: {serialData}")
        data = np.full((self._channels + 2), np.nan, np.uint16)
        for i in range(0, (self._channels + 2)):
            j = i*2
            data[i] = int.from_bytes(serialData[j:j+2], byteorder='little', signed=False)
          

        print(f"data: {data}")

        if len(data) == self._channels + 2:
            packet = data[0]
            timestamp = data[1]
            lsldata = data[2:]
            if lsldata:
                row = [ packet, timestamp, lsldata]
        return row
    
    def _check(self, count):
        """Report dropped samples.

        We don't even bother about a possible overflow of the sample counter as it
        would take about 3 days at 16K SPS before reaching the maximum value.
        """
        missed = (count - self._count) - 1
        self._count = count
        if missed:
            self._missed += missed
        else:
            if self._missed:
                self.logger.warn(f"Missed {self._missed} samples")
                self._missed = 0

    def update(self):
        """Update the node output."""
        with self._lock:
            if self._rows:          
                self.o.set(self._rows, self._timestamps, self.names, self.meta)
                self._reset()
            

    def terminate(self):
        """Cleanup."""
        self._running = False
        self._send('stop')
        global serialdevice
        serialdevice.close()
        while self._thread and self._thread.is_alive():
            sleep(0.001)
