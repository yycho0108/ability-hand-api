import numpy as np
import time
import threading
from math import pi, sin

from ah_wrapper import AHSerialClient
# from ah_plotting.plots import RealTimePlotMotors

import viser
from viser.extras import ViserUrdf
import viser.transforms as vtf

RUNNING = True


def hand_wave_thread(hand_client):
    pos = [30, 30, 30, 30, 30, -30]
    while RUNNING:
        current_time = time.time()
        for i in range(0, len(pos)):
            ft = current_time * 3 + i * (2 * pi) / 12
            pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
        pos[5] = -pos[5]
        hand_client.set_position(positions=pos, reply_mode=2)
        # hand_client.send_command()
        time.sleep(1 / hand_client.rate_hz)


def main():
    VISER = False
    # client = AHSerialClient(write_thread=False)
    client = AHSerialClient(
        port='/dev/ttyUSB0',
        reply_mode=2,
        read_timeout=0,
        write_timeout=0.02, # at least 50Hz
        rate_hz=200, # no need to be too fast
        # NOTE(ycho): not too sure about these options
        # rs_485: bool = False,
        # read_thread: bool = True,
        # write_thread: bool = True,
        # auto_start_threads: bool = True,
        write_thread=False
    )
    
    # write_thread = threading.Thread(target=hand_wave_thread, args=(client,))
    # write_thread.start()

    # plotter = RealTimePlotMotors(client.hand)
    # try:
    #     plotter.start()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    global RUNNING
    try:
        from pathlib import Path
        if VISER:
            srv = viser.ViserServer()
            urdf = Path( '/home/imsquared/sim2real-robot-arm/alps/src/alps/simulation/assets/urdf/alps_short/robot.urdf' )
            viser_urdf = ViserUrdf(srv, urdf_or_path=urdf, root_node_name='/robot')

            joint_names = viser_urdf.get_actuated_joint_names()
            q_urdf = np.zeros(len(joint_names))
            # [index, middle, ring, pinky, thumb flexor, thumb rotator]
            finger_joint_names = [
                'right_index_q1',
                'right_middle_q1',
                'right_ring_q1',
                'right_pinky_q1',
                'right_thumb_q2',
                'right_thumb_q1', # <- this is the only backdrivable joint
            ]
            mimic_joint_names = [
                'right_index_q2',
                'right_middle_q2',
                'right_ring_q2',
                'right_pinky_q2']
            indices = [joint_names.index(k) for k in [*finger_joint_names, *mimic_joint_names]]
            indices = np.asarray(indices)
            print(F'Got indices = {indices}')

            R1 = vtf.SO3.from_x_radians(-np.pi/2) 
            R2 = vtf.SO3.from_z_radians(np.pi/2)
            srv.scene.add_frame(
                '/robot/link7/link8/link9/link10/link11/link12/hand_base',
                # wxyz=(1.0, 0.0, 0.0, 0.0),
                wxyz=(R1@R2).wxyz,
                position=(0,0,0),
            )

            urdf2 = Path('../../URDF/ability_hand_right_large.urdf')
            viser_urdf2 = ViserUrdf(srv, urdf_or_path=urdf2, root_node_name='/robot/link7/link8/link9/link10/link11/link12/hand_base')
            joint_names = viser_urdf2.get_actuated_joint_names()
            q_urdf2 = np.zeros(len(joint_names))
            finger_joint_names = [
                'index_q1',
                'middle_q1',
                'ring_q1',
                'pinky_q1',
                'thumb_q2',
                'thumb_q1', # <- this is the only backdrivable joint
            ]
            print(joint_names, finger_joint_names)
            indices2 = [joint_names.index(k) for k in finger_joint_names]
            indices2 = np.asarray(indices2)
            print(F'Got indices2 = {indices2}')



        while True:
            if True:
                pos = [30, 30, 30, 30, 30, -30]
                current_time = time.time()
                for i in range(0, len(pos)):
                    ft = current_time * 3 + i * (2 * pi) / 12
                    pos[i] = (0.5 * sin(ft) + 0.5) * 45 + 15
                pos[5] = -pos[5]
                client.set_position(positions=pos, reply_mode=2)
                client.send_command()
                time.sleep(1 / client.rate_hz)

            q_hand = client.hand.get_position() # -> 6 numbers (deg)
            print('q_hand', q_hand)

            if VISER:
                q_hand_rad = np.deg2rad(q_hand)
                q_mimic_rad = np.multiply(1.05, q_hand_rad[:4])
                q_hand_rad_m = np.concatenate([q_hand_rad, q_mimic_rad], axis=-1)
                q_urdf[indices] = q_hand_rad_m # NOTE(ycho): maybe a sign-flip is needed here?
                q_urdf2[indices2] = q_hand_rad
                viser_urdf.update_cfg( q_urdf )
                viser_urdf2.update_cfg( q_urdf2)

            # time.sleep(0.01) # 100Hz
            time.sleep(0.002)

    finally:
        RUNNING = False
        client.close()


if __name__ == "__main__":
    main()
