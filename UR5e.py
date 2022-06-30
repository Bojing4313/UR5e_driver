import numpy as np
import math as m
import socket
import time
import struct
import sys
import util
import rtde
import notify as nf

DEFAULT_TIMEOUT = 5.0

SIM_HOST = "192.168.159.128"
ROBOT_HOST = "192.168.31.234"

class URController:
    """Class for the Universal Robot allowing for communication and control of the
    Universal Robot with all of its features available.

    Attributes
    ----------
    address : string
        The IP address associated to the Universal Robot
    socket : socket object
        Socket connecting to physical Universal Robot.

    """

    def __init__(self, address):
        self.HOST = address
        self.PORT = 30003

    def get_current_tcp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(DEFAULT_TIMEOUT)
        s.connect((self.HOST, self.PORT))
        data = s.recv(1108)
        position = struct.unpack('!6d', data[444:492])
        s.close()
        return np.asarray(position)

    def get_pose(self):
        con = rtde.RTDE(self.HOST, 30004)
        con.connect()
        output_names = ['actual_TCP_pose']
        output_types = ['VECTOR6D']
        con.send_output_setup(output_names, output_types, frequency=125)
        con.send_start()
        state = con.receive(True)
        pose = np.asarray(struct.unpack('!6d', state))
        return pose

    def get_joints(self):
        con = rtde.RTDE(self.HOST, 30004)
        con.connect()
        output_names = ['actual_q']
        output_types = ['VECTOR6D']
        con.send_output_setup(output_names, output_types, frequency=125)
        con.send_start()
        state = con.receive(True)
        joints = np.asarray(struct.unpack('!6d', state))
        return np.degrees(joints)

    def movel(self, x, y, z, rx, ry, rz, tool_acc = 1.2, tool_vel= 0.05):
        """
        Move TCP linearly in reference to the WRF
        # tool_acc Safe: 0.5
        # tool_vel Safe: 0.2
        """
        target_tcp = np.array([x,y,z,rx,ry,rz])
        
        # Build command
        tcp_command = f"movel(p[{x},{y},{z},{rx},{ry},{rz}],a={tool_acc},v={tool_vel},t=0,r=0)\n"

        # Open socket send command
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(DEFAULT_TIMEOUT)
        s.connect((self.HOST, self.PORT))
        s.send(str.encode(tcp_command))
        s.close()

        # Make sure destination is reached, so the next command could be sent
        # Set a certeria to decide if a movement is finished
        tolerance = [0.001, 0.001, 0.001, 0.05, 0.05, 0.05]
        current_pos = self.get_pose()
        for i in range(15):
            if all(abs(current_pos-target_tcp) < tolerance):
                break
            elif i == 14:

                # if Server has no responding
                nf.log_print("Transimission Error, manual operation needed !")
                proceed = input("\nTo countinue motion, Please switch to Remote Mode on Teach Pendant and confirm [Y]: ")

                # Send the command again
                if proceed in ['Y', 'y']:
                    nf.log_print("UR motion resumed by user !")
                    self.movel(x,y,z,rx,ry,rz)
                else:
                    self.go_home()
                    nf.log_print("Motion interupted by user !")
                    sys.exit()
            else:
                current_pos = self.get_pose()
                nf.dynam_print("UR in motion")
        return 'End of Block.'

    def movej(self, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, joints:bool, tool_acc = 1.2, tool_vel= 0.5):
        # Drive the UR's 6 joints into specific angles (Arguments are in ° if joints is True)
        # If target joint is equal to or larger than 180°, implementation will be divided into two phases
        # tool_acc acceleration has a safe value of 0.5 m/s^2
        # tool_vel tool velocity has a safe value of 0.2 m/s

        if joints:
            target_j = np.array([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])

            # Convert degree into radians
            # Build tcp_command for joints
            tcp_command = f"movej([{m.radians(target_j[0])},{m.radians(target_j[1])},{m.radians(target_j[2])},{m.radians(target_j[3])},{m.radians(target_j[4])},{m.radians(target_j[5])}],a={tool_acc},v={tool_vel},t=0,r=0)\n"

            # Criteria to judge joints movement finishing
            tolerance = np.array([2.86, 2.86, 2.86, 2.86, 2.86, 2.86])

            # Open socket and send the command
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(DEFAULT_TIMEOUT)
            s.connect((self.HOST, self.PORT))
            s.send(str.encode(tcp_command))  # 利用字符串的encode方法编码成bytes，默认为utf-8类型
            s.close()

            # Make sure destination is reached, so the next command could be sent
            current_j = self.get_joints()
            for i in range(15):
                if all(abs(current_j - target_j) < tolerance):
                    break
                elif i == 14:

                    # if Server has no responding
                    nf.log_print("Transimission Error, manual operation needed !")
                    proceed = input("\nTo countinue motion, Please switch to Remote Mode on Teach Pendant and confirm [Y]: ")

                    # Send the command again
                    if proceed in ['Y', 'y']:
                        nf.log_print("UR motion resumed by user !")
                        self.movej(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, joints=True)
                    else:
                        self.go_home()
                        nf.log_print("Motion interupted by user !")
                        sys.exit()
                else:
                    current_j = self.get_joints()
                    nf.dynam_print("UR in motion")
            return 'End of Block.'

        if not joints:
            # Input is in pose forms
            target_tcp = np.array([theta_1,theta_2,theta_3,theta_4,theta_5,theta_6])

            # Build tcp_command for pose
            tcp_command = f"movej(p[{target_tcp[0]},{target_tcp[1]},{target_tcp[2]},{target_tcp[3]},{target_tcp[4]},{target_tcp[5]}],a={tool_acc},v={tool_vel},t=0,r=0)\n"

            # Criteria to judge pose movement finishing
            tolerance = np.array([0.001, 0.001, 0.001, 0.05, 0.05, 0.05])

            # Open socket and send the command
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(DEFAULT_TIMEOUT)
            s.connect((self.HOST, self.PORT))
            s.send(str.encode(tcp_command))  # 利用字符串的encode方法编码成bytes，默认为utf-8类型
            s.close()

            # Make sure destination is reached, so the next command could be sent
            current_pos = self.get_pose()
            for i in range(15):
                if all(abs(current_pos-target_tcp) < tolerance):
                    break
                elif i == 14:

                    # if Server has no responding
                    nf.log_print("Transimission Error, manual operation needed !")
                    proceed = input("\nTo countinue motion, Please switch to Remote Mode on Teach Pendant and confirm [Y]: ")

                    # Send the command again
                    if proceed in ['Y', 'y']:
                        nf.log_print("UR motion resumed by user !")
                        self.movej(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, joints=False)
                    else:
                        self.go_home()
                        nf.log_print("Motion interupted by user !")
                        sys.exit()
                else:
                    current_pos = self.get_pose()
                    nf.dynam_print("UR in motion")
            return 'End of Block.'

    def reference_move(self, delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz):
        increase_pose = np.array([delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz])
        act_pose = self.get_pose()
        target_pose = np.add(act_pose, increase_pose)

        self.movel(*target_pose)
        
    def increase_move(self, delta_x, delta_y, delta_z, delta_theta):
        tcp = self.get_pose()
        rpy = util.rv2rpy(tcp[3], tcp[4], tcp[5])
        rpy[2] = rpy[2] + delta_theta
        target_rv = util.rpy2rv(rpy)
        target_tcp = np.asarray([tcp[0] + delta_x, tcp[1] + delta_y, tcp[2] + delta_z,
                                target_rv[0], target_rv[1], target_rv[2]])
        self.movel(*target_tcp)


    def get_digital_output(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(DEFAULT_TIMEOUT)
        s.connect((self.HOST, self.PORT))
        data = s.recv(1108)
        tool = struct.unpack('!d', data[1044:1052])[0]
        s.close()
        return tool


    def operate_gripper(self, grip:bool):
        # Grip/Release the gripper
        # True: Grip, False: Release
        tcp_command = f"set_tool_digital_out(0, {grip})\n"
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(DEFAULT_TIMEOUT)
        s.connect((self.HOST, self.PORT))
        s.send(str.encode(tcp_command))  # 利用字符串的encode方法编码成bytes，默认为utf-8类型
        s.close()

    def check_grasp(self):
        con = rtde.RTDE(self.HOST, 30004)
        con.connect()
        output_names = ['tool_analog_input0']
        output_types = ['DOUBLE']
        con.send_output_setup(output_names, output_types, frequency=125)
        con.send_start()
        state = con.receive(True)
        voltage = struct.unpack('!d', state)
        return voltage

    def move_down(self, low):
        tcp = self.get_pose()
        tcp[2] = low
        self.movel(*tcp)


    def move_up(self, high):
        tcp = self.get_pose()
        tcp[2] = high
        self.movel(*tcp)


    def grasp(self, low, high):
        self.operate_gripper(grip=False)
        self.move_down(low)
        time.sleep(0.5)
        self.operate_gripper(grip=True)
        time.sleep(1)
        self.move_up(high)

    def drop(self, low, high):
        self.move_down(low)
        time.sleep(0.5)
        self.operate_gripper(grip=False)
        time.sleep(1)
        self.move_up(high)

    def go_home(self, target_home=np.array([None,None,None,None,None,None])):
        act_pos = self.get_pose()

        # Check if the TCP has potential collision
        time.sleep(0.01)
        if act_pos[2] <= 0.04:
            self.increase_move(0,0,0.04,0)
            time.sleep(0.2)
        if target_home.all() == None:
            act_joints = self.get_joints()
            if -90 < act_joints[0] and act_joints[0] < 90:
                target_home = np.array([0,-90,-90,-90,90,0])
            elif -270 < act_joints[0] and act_joints[0] < -90:
                target_home = np.array([-180,-90,-90,-90,90,0])
        self.movej(*target_home, joints=True, tool_vel=0.7)
        
        return 'Homed.'

if __name__ == '__main__':
    ur = URController(SIM_HOST)
    ur.go_home()
