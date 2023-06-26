import serial
import time
import json
from json import JSONDecodeError
from timeflux.core.node import Node
from timeflux.helpers.clock import now
from threading import Thread, Lock
from time import sleep, time
from pylsl import StreamInfo, StreamOutlet, local_clock
import numpy as np
from datetime import timezone
import datetime

maxFrequency = 2000


class UDL(Node):
    """UDL driver.

    Attributes:
        o (Port): Default output, provides DataFrame.

    Args:
        port (string): The serial port.
            e.g. ``COM3`` on Windows;  ``/dev/cu.usbmodem14601`` on MacOS;
            ``/dev/ttyUSB0`` on GNU/Linux.
        
        rate (int): The device rate in Hz.Default: 250. Max limit: 2000 Hz

        channels (int): The number of channels to enable. Default:1.
        
        names (list of string): The names of channels. Ex: ['C1', 'C2' ..]
            Default: Channels are numbered from 1. 
        
        debug (bool): If ``True``, print debug information. Default: ``False``.

    Example:
        .. literalinclude:: /../examples/simple.yaml
           :language: yaml

    """
    def __init__(self, port, rate=250, channels=1, names=None, debug=False):

        # Validate input
        if rate > maxFrequency:
            raise ValueError(
                f"'{rate}' exceeded max limit"
            )
        
         # Set channel names
        if isinstance(names, list) and len(names) == channels:
            self.names = names
        else:
            index = list(range(1, channels + 1))
            name = []
            for i in index:
                name.append(i)
            self.names = name


        global serialdevice
        serialdevice = serial.Serial(port=port, baudrate='115200')

        # Compute time offset
        row = self._read()

        # Remember sample count
        self._count = 0
        self._missed = 0

        # Set meta
        self.meta = {"rate": rate}

        # Launch background thread
        self._reset()
        self._lock = Lock()
        self._running = True
        self._thread = Thread(target=self._loop).start()
    
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

            except:
                pass
    
    def _read(self):
        """Read a line of data from the device."""
        row = []
        global serialdevice
        serialData = serialdevice.readline().rstrip()

        try:
            response_obj = json.loads(serialData)
        except JSONDecodeError:
            response_obj = {}
            print(f"json decode error: {serialData}")
            serialdevice.flush()
    
        if response_obj:
            data = list(response_obj.values())
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
        while self._thread and self._thread.is_alive():
            sleep(0.001)
        