#!/usr/bin/env python3

from dataclasses import dataclass, field
from contextlib import contextmanager

import serial
import pickle
from serial.tools import list_ports
from plot_floats import plot_floats
import time
import numpy as np
import struct
import sys
import platform
import math
import keyboard
import argparse
from PPP_stuffing import *
from abh_api_core import *

import pinocchio as pin
from pathlib import Path

import os


@contextmanager
def with_dir(d):
    d0 = os.getcwd()
    try:
        os.chdir(d)
        yield
    finally:
        os.chdir(d0)


class Ser(serial.Serial):
    def read(self, *args, **kwds):
        # print('<read>', args, kwds)
        out = super().read(*args, **kwds)
        # print('</read>', out)
        return out

    def write(self, *args, **kwds):
        # print('<write>', args, kwds)
        out = super().write(*args, **kwds)
        # print('</write>', out)
        return out


## Send Miscellanous Command to Ability Hand


def create_misc_msg(cmd):
    barr = []
    barr.append((struct.pack('<B', 0x50))[0])  # device ID
    barr.append((struct.pack('<B', cmd))[0])  # command!
    sum = 0
    for b in barr:
        sum = sum + b
    chksum = (-sum) & 0xFF
    barr.append(chksum)
    return barr


def index_map(k_to, k_from):
    """Returns an index mapping from k_from to k_to.

    Given k_to=a, k_from=b,
    returns an index map "a_from_b" such that
    array_a[a_from_b] = array_b

    Missing values are set to -1.
    """
    index_dict = {k: i for i, k in enumerate(k_to)}  # O(len(k_from))
    return [index_dict.get(k, -1) for k in k_from]  # O(len(k_to))


@dataclass
class HandData:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(
            6, dtype=np.float32))
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(
            6, dtype=np.float32))
    touch: np.ndarray = field(
        default_factory=lambda: np.zeros(
            30, dtype=np.float32))

    position_target: np.ndarray = field(
        default_factory=lambda: np.zeros(
            6, dtype=np.float32))
    torque_target: np.ndarray = field(
        default_factory=lambda: np.zeros(
            6, dtype=np.float32))


class Psyonic:

    @dataclass
    class Config:
        baud: int = 460800
        addr: int = 0x50

        # reply_mode: str = 0x10  # = Variant 1
        reply_mode: str = 0x11  # = Variant 2, positional ctrl
        # reply_mode: str = 0x13 # Variant 3
        # reply_mode: str = 0x31  # = Variant 2, torque ctrl

        # in torque mode, 0x30 = Variant 1
        # in torque mode, 0x31 = Variant 2
        # in torque mode, 0x32 = Variant 3

        port: str = ''
        # mode: str = 'position' / 'touch'
        stuff: bool = False
        rs485: bool = False

        init_pos: np.ndarray = field(default_factory=lambda: np.asarray([
            15, 15, 15, 15, 15, -15], dtype=np.float32))

        C: float = 620.606079

        # Nm/A; motor torque constant X gear ratio
        k_t: np.ndarray = field(default_factory=lambda: np.asarray(
            # FIXME(ycho): bit of duplication with gear_ratio below
            [1.49e-3 * 649] * 5 + [1.49e-3 * 162.45]
        ))

        # Gear ratios
        gear_ratio: np.ndarray = field(default_factory=lambda: np.asarray(
            [649] * 5 + [162.45]
        ))

        # kp: float = 24.0
        # kp: float = 4.0
        kp: np.ndarray = field(default_factory=lambda: np.asarray(
            [4.0] * 5 + [0.7]
            # [0.0] * 5 + [0.7]
            # [0.0] * 5 + [2.0]
            # [0.0] * 4 + [4.0, 0.0]
            # [0.0] * 6
        ))
        kd: np.ndarray = field(default_factory=lambda: np.asarray(
            # [0.0] * 5 + [0.001]  # <- backemf
            [0.01] * 5 + [0.01]  # <- backemf
            # [0.01] * 5 + [0.001]
        ))
        # 1A per motor (ad-hoc)
        max_current: float = 1.0
        max_torque: np.ndarray = field(default_factory=lambda: np.asarray(
            [6.0, 6.0, 6.0, 6.0, 6.0, 1.2]
        ))

        urdf_path: str = './urdf/ability_coacd.urdf'

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.port = cfg.port
        self.ser = None
        self.data = HandData()

        self.data.position[:] = np.deg2rad(cfg.init_pos)
        self.data.position_target[:] = np.deg2rad(cfg.init_pos)

        path = Path(cfg.urdf_path)
        with with_dir(path.parent):
            self.robot = pin.RobotWrapper.BuildFromURDF(filename=path.name,
                                                        package_dirs=["."],
                                                        root_joint=None)
            mot_joint = [
                'left_index_q1',
                'left_middle_q1',
                'left_ring_q1',
                'left_pinky_q1',
                'left_thumb_q2',  # "thumb flexor"
                'left_thumb_q1'  # "thumb rotator"
            ]
            pin_joint = list(self.robot.model.names[1:])
            self.pin_from_mot = index_map(pin_joint,
                                          mot_joint)

    def to_pin(self, q: np.ndarray):
        s = 1.05851325
        out = [
            q[0], s * q[0],
            q[1], s * q[1],
            q[2], s * q[2],
            q[3], s * q[3],
            # NOTE(ycho): flipped order
            q[5], q[4]
        ]
        return np.asarray(out)

    @contextmanager
    def open(self):
        try:
            suc = self.setup()
            if not suc:
                print('Setup failed')
                raise ValueError('stop')
            yield self
        finally:
            if self.ser is not None:
                # print('==close==')
                self.ser.close()
                # raise ValueError('stopped')
            self.ser = None

    def search_port(self):
        com_ports_list = list(list_ports.comports())
        port = ""
        for p in com_ports_list:
            if not p:
                continue

            if platform.system() == 'Linux' or platform.system() == 'Darwin':
                # if "USB" in p[0]:
                port = p[0]  # ?
                print("Found:", port)
                break
            elif platform.system() == 'Windows':
                if "COM" in p[0]:
                    port = p[0]  # ?
                    print("Found:", port)
                    break
        return port

    def write_pos(self, positions: np.ndarray):
        cfg = self.cfg
        self.data.position_target[:] = positions
        # self.data.position_target[..., 5] *= -1
        # positions[..., 5] *= -1

        if cfg.reply_mode in [0x30, 0x31, 0x32]:
            torque = self.get_torque(positions)
            self.data.torque_target[:] = torque
            msg = self.generate_tx_as_torque(torque)
        else:
            positions = np.rad2deg(positions)
            msg = self.generate_tx(positions)

        self.ser.write(msg)

    def read_pos(self) -> HandData:
        cfg = self.cfg
        ser = self.ser

        data = ser.read(1)

        # FIXME(ycho): deal with all these ad-hoc rad-deg
        # conversions.
        posRead = np.rad2deg(self.data.position.copy())
        velRead = self.data.velocity.copy()
        touchRead = self.data.touch.copy()

        needReset = True
        if len(data) == 1:
            replyFormat = data[0]
            ## Reply variant 3 length is
            if (replyFormat & 0xF) == 2:
                replyLen = 38
            else:
                replyLen = 71
            ##read the rest of the data
            data = ser.read(replyLen)
            needReset = False
            if len(data) == replyLen:
                ## Verify Checksum
                sum = replyFormat
                for byte in data:
                    sum = (sum + byte) % 256

                if sum != 0:
                    print('reset', sum)
                    needReset = True
                else:
                    # print('read')
                    ## Extract Position Data
                    ## Position Data is included in all formats in same way
                    ## So we can safely do this no matter the format
                    for i in range(0, 6):
                        rawData = struct.unpack(
                            '<h', data[i * 4:2 + (i * 4)])[0]
                        posRead[i] = rawData * 150 / 32767

                        # Bad data, reset serial device - probably
                        # framing error
                        if posRead[i] > 150:
                            print('baddata', sum)
                            needReset = True

                    # Extract rotor velocity data
                    # ONLY available in variant 2,3
                    # currently only supports variant 2
                    if cfg.reply_mode in [0x11, 0x31]:
                        # Apply index offset of 2 first
                        vel_data = data[2:]
                        # Afterward, parese the data
                        for i in range(0, 6):
                            rawData = struct.unpack(
                                '<h', vel_data[i * 4:2 + (i * 4)])[0]
                            velRead[i] = rawData * 0.25 / cfg.gear_ratio[i]

                    ## Extract Touch Data if Available
                    if replyLen == 71:
                        ## Extract Data two at a time
                        for i in range(0, 15):
                            dualData = data[(
                                i * 3) + 24:((i + 1) * 3) + 24]
                            data1 = struct.unpack('<H', dualData[0:2])[
                                0] & 0x0FFF
                            data2 = (
                                struct.unpack('<H', dualData[1: 3])
                                [0] & 0xFFF0) >> 4
                            touchRead[i * 2] = int(data1)
                            touchRead[(i * 2) + 1] = int(data2)
                            if data1 > 4096 or data2 > 4096:
                                print('baddata2', sum)
                                needReset = True
            else:
                needReset = True

        if needReset:
            ser.reset_input_buffer()

        self.data.position[:] = np.deg2rad(posRead)
        self.data.velocity[:] = velRead
        self.data.touch[:] = touchRead

        return self.data

    def generate_tx(self, positions: np.ndarray):
        cfg = self.cfg

        txBuf = []
        ## Address in byte 0
        txBuf.append((struct.pack('<B', cfg.addr))[0])
        ## Format Header in byte 1
        txBuf.append((struct.pack('<B', cfg.reply_mode))[0])

        # Position data for all 6 fingers, scaled to fixed point representation
        for i in range(0, 6):
            posFixed = int(positions[i] * 32767 / 150)
            txBuf.append((struct.pack('<B', (posFixed & 0xFF)))[0])
            txBuf.append((struct.pack('<B', (posFixed >> 8) & 0xFF))[0])

        ## calculate checksum
        cksum = 0
        for b in txBuf:
            cksum = cksum + b
        cksum = (-cksum) & 0xFF
        txBuf.append((struct.pack('<B', cksum))[0])

        return txBuf

    def get_torque(self, positions: np.ndarray):
        cfg = self.cfg
        p = (positions - self.data.position)
        d = self.data.velocity

        kp = np.copy(cfg.kp)
        # kp[5] = (8.0 if p[5] < 0 else 2.0)
        # print(kp[5])
        torque = kp * p - cfg.kd * d

        # Gravity compensation...ish
        if False:
            h = pin.nonLinearEffects(
                self.robot.model,
                self.robot.data,
                self.to_pin(self.data.position),
                self.to_pin(self.data.velocity),
            )
            # print(h[8])
            torque[5] += h[8]
        return torque

    def generate_tx_as_torque(self, torque: np.ndarray):
        cfg = self.cfg
        torque = np.clip(torque,
                         -cfg.max_torque,
                         +cfg.max_torque)
        current = torque / cfg.k_t
        current = np.clip(current,
                          -cfg.max_current,
                          +cfg.max_current)
        # current[5] = -current[5]
        # current[5] = np.sign(current[5]) * np.maximum(
        #     np.abs(current[5]),
        #     0.04)

        txBuf = []
        ## Address in byte 0
        txBuf.append((struct.pack('<B', cfg.addr))[0])
        ## Format Header in byte 1
        txBuf.append((struct.pack('<B', cfg.reply_mode))[0])

        # _Current_ data for all 6 fingers, scaled to fixed point
        # representation
        for i in range(0, 6):
            # I_digital = I_amps * C
            posFixed = int(current[i] * cfg.C)
            txBuf.append((struct.pack('<B', (posFixed & 0xFF)))[0])
            txBuf.append((struct.pack('<B', (posFixed >> 8) & 0xFF))[0])

        ## calculate checksum
        cksum = 0
        for b in txBuf:
            cksum = cksum + b
        cksum = (-cksum) & 0xFF
        txBuf.append((struct.pack('<B', cksum))[0])

        return txBuf

    def setup(self):
        cfg = self.cfg

        if not self.port:
            self.port = self.search_port()

        if not self.port:
            return False

        try:
            print(F"Connecting... {self.port}")
            # self.ser = serial.Serial(
            self.ser = Ser(
                self.port,
                cfg.baud,
                timeout=0
            )
            print("Connected!", type(self.ser))
            # " Upsample the thumb rotator "
            # whatever that means...
            print('create')
            msg = create_misc_msg(0xC2)
            self.ser.write(msg)
            self.ser.reset_input_buffer()
        except Exception as e:
            print(e)
            print("Failed to Connect!")
            return False

        return True


def main():
    action = np.zeros(6)
    log = {
        'stamp': [],
        'pos': [],
        'vel': [],
        'tau': [],
        'pos_target': []
    }

    try:
        with Psyonic(Psyonic.Config()).open() as hand:
            # for _ in range(4):
            target_hz = 256
            last_step = time.time()
            while True:
                for i in range(6):
                    ft = time.time() * 3.0 + i
                    action[i] = (0.5 * np.sin(ft) + 0.5) * 45 + 15
                    # action[i] = np.where((action[i] > 45), 60, 15)
                    # action[i] = np.where((action[i] > 37.5), 60, 15)
                    # action[
                # print(action[5])
                action[5] = -action[5]
                # action[:] = 15
                hand.write_pos(np.deg2rad(action))
                # hand.write_pos(hand.data.position)
                read = hand.read_pos()
                # print(read.position, read.velocity)
                # print('read', read)
                log['stamp'].append(time.time())
                log['pos'].append(np.copy(read.position))
                log['tau'].append(np.copy(read.torque_target))
                log['vel'].append(np.copy(read.velocity))
                log['pos_target'].append(np.copy(read.position_target))
                # time.sleep(1.0 / 256)
                time.sleep(1.0 / 300.0)
    finally:
        with open('/tmp/psyonic_log.pkl', 'wb') as fp:
            pickle.dump(log, fp)


if __name__ == '__main__':
    main()
