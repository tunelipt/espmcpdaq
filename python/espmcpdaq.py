# -*- coding: utf-8 -*-

import serial
import time
import numpy as np
import argparse
import threading


from xmlrpc.server import SimpleXMLRPCServer

class ESPDaqThread(threading.Thread):
    def __init__(self, dev):
        threading.Thread.__init__(self)
        self.dev = dev
        return
    def run(self):
        self.dev.scan_raw()
        
        
class ESPDaq(object):

    def __init__(self, dev='/dev/ttyUSB0', speed=115200):

        self.s = serial.Serial(dev, speed, timeout=1)
        self.dev = dev
        self.speed = speed
        self._avg = 100
        self._period = 100
        self._fps = 1
        self.acquiring = False
        self.nsamples = 0
        self.stopaq = False
        self.frames = []
        self.thrd = None
        
    def close(self):
        if self.acquiring:
            raise RuntimeError("Acquiring data. Stopit first!")
        self.s.close()
        return None
    def open(self):
        if self.s.is_open:
            self.close()
        self.s.open()
        return None
    def avg(self, val=None):
        if self.acquiring:
            raise RuntimeError("Can't do this while acquiring data!")
        if val is None:
            cmd = '?A\n'.encode('ascii')
            self.s.write(cmd)
            b = self.s.readline()
            self._avg = int(b)
            return self._avg
        else:
            if val < 1 or val > 1000:
                val = 1
            val = int(val)
            self._avg = val
            cmd = '.A{}\n'.format(val).encode('ascii')
            self.s.write(cmd)
            return self.s.readline().decode('ascii')
    def period(self, val=None):
        if self.acquiring:
            raise RuntimeError("Can't do this while acquiring data!")
        if val is None:
            cmd = '?P\n'.encode('ascii')
            self.s.write(cmd)
            b = self.s.readline()
            self._period = int(b)
            return self._period
        else:
            if val < 10 or val > 1000:
                val = 100
            val = int(val)
            self._period = val
            cmd = '.P{}\n'.format(val).encode('ascii')
            self.s.write(cmd)
            return self.s.readline().decode('ascii')
    def fps(self, val=None):
        if self.acquiring:
            raise RuntimeError("Can't do this while acquiring data!")
        if val is None:
            cmd = '?F\n'.encode('ascii')
            self.s.write(cmd)
            b = self.s.readline()
            self._fps = int(b)
            return self._fps
        else:
            if val < 1 or val > 30000:
                val = 1
            val = int(val)
            self._fps = val
            cmd = '.F{}\n'.format(val).encode('ascii')
            self.s.write(cmd)
            return self.s.readline().decode('ascii')
    
    def scan_raw(self):
        if self.acquiring:
            raise RuntimeError("Already acquiring data!")
        nframes = self.fps()

        self.frames = []

        self.s.write(b'*\n')
        time.sleep(0.05)
        self.stopaq = False
        self.acquiring = True
        self.nsamples = 0
        fdt = self._period/1000
        self.s.timeout = max(2*fdt, 1)
        time.sleep(fdt)
        foundtrm = False
        for i in range(nframes):
            # Check if we should stop acquiring data:
            if self.stopaq:
                
                time.sleep(2.5*fdt) # Wait a little
                foundtrm = False
                tmpfr = self.s.read(80)
                if len(tmpfr) == 80:
                    xtmp = self.decode_frame(tmpfr)
                    if xtmp[4] == 0xffffffff: # Found the termination frame
                        foundtrm = True
                        break
                    else:
                        self.frames.append(xtmp) # Appears to be valid frame
                for iattempt in range(3):
                    time.sleep(2*fdt)
                    tmpfr = self.s.read(80)
                    
                    if len(tmpfr) == 80:
                        xtmp = self.decode_frame(tmpfr)
                        if xtmp[4] == 0xffffffff:
                            foundtrm = True
                            break
                if not foundtrm:
                    time.sleep(fdt)
                    self.s.read(80)
                    # I don't know what is going on. IU give up
                    raise RuntimeError("DAQ stopped but unable to find termination frame!")
                else:
                    foundtrm = True
                    break
            
            else: # Normal operation
                self.nsamples = i+1
                tmpfr = self.s.read(80)
                xtmp = self.decode_frame(tmpfr)
                if xtmp[4] != 0xffffffff:
                    self.frames.append(tmpfr)
                else:
                    break
        
        self.stopaq = False
        self.acquiring = False
        self.s.timeout = 1
        return self.frames
    
    def decode_frame(self, frame):
        h = np.frombuffer(frame, np.uint32, 1, 0)[0]
        t = np.frombuffer(frame, np.uint32, 1, 4)[0]
        num = np.frombuffer(frame, np.uint32, 1, 8)[0]
        E = np.frombuffer(frame, np.uint16, 32, 12)
        f = np.frombuffer(frame, np.uint32, 1, 76)[0]
        return E, t, num, h, f

    def parse_daqdata(self, frames):
        dados = [self.decode_frame(y) for y in frames]
        nfr = len(frames)
        # Allocate memory
        E = np.zeros((nfr, 32), np.uint16)

        # Initial daq time in ms (from ESP32)
        t0 = dados[0][1]
        if nfr == 1:
            freq = 1000/self._period
        else:
            dt = dados[nfr-1][1] - t0
            dtm  = dt / (nfr-1)
            freq = 1000.0/dtm
            
        for i in range(nfr):    
            E[i,:] = dados[i][0]
        return E, freq
        
        
    def scan(self):
        x = self.scan_raw()
        return self.parse_daqdata(x)

    def scanbin(self):
        E, f = self.scan()
        
        return E.tobytes(), int(E.shape[0]), int(E.shape[1]), float(f)
    
    def start(self):
        if self.acquiring:
            raise RuntimeError("Illegal operation: System is already acquiring!")
        self.thrd = ESPDaqThread(self)
        self.thrd.start()
        self.acquiring = True
    def read(self):
        if self.thrd is not None:
            self.thrd.join()
            
        if self.nsamples > 0:
            return self.parse_daqdata(self.frames)
        else:
            return 0, -1.0
    def readbin(self):
        E,f = self.read()
        if self.nsamples > 0:
            return E.tobytes(), int(E.shape[0]), int(E.shape[1]), float(f)
        else:
            return b'', 0, 0, -1.0
        
                
    def stop(self):
        if self.acquiring:
            self.stopaq = True
            self.s.write(b"!\n")
        return
    def isacquiring(self):
        return self.acquiring
    def samplesread(self):
        return self.nsamples
    
        
        
        
def start_server(ip='localhost', port=9541, comport='/dev/ttyUSB0', baud=115200):
    dev = ESPDaq(comport, baud)
    print("Starting XML-RPC server")
    print("IP: {}, port: {}".format(ip, port))
    server = SimpleXMLRPCServer((ip, port), allow_none=True)
    server.register_instance(dev)
    server.serve_forever()


if __name__ == "__main__":
    print("Creating interface ...")
    parser = argparse.ArgumentParser(description="ESPDaq server")
    parser.add_argument("-i", "--ip", help="IP address of the XML-RPC server", default="localhost")
    parser.add_argument("-p", "--port", help="XML-RPC server port", default=9541, type=int)
    parser.add_argument("-s", "--comport", help="Serial port to be used", default="/dev/ttyUSB0")

    args = parser.parse_args()
    start_server(args.ip, args.port, args.comport)
    
