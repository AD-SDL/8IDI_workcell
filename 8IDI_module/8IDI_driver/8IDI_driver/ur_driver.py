#!/usr/bin/env python3
from multiprocessing.connection import wait
from copy import deepcopy
import threading
import socket 

from multiprocessing.connection import wait
from time import sleep
from copy import deepcopy

from ur_dashboard import UR_DASHBOARD
import robotiq_gripper as robotiq_gripper
from urx import Robot, RobotException
import epics

class URRobot(UR_DASHBOARD):
    

    def __init__(self, IP:str = "146.137.240.38", PORT: int = 29999):
        
        super().__init__(IP=IP, PORT=PORT)

        # ur5 SETUP:
        self.ur5 = None
        self.gripper = None
        self.pipette = None
        self.tool_changer = None
        self.camera = None
    
        self.connect_ur()
        self.connect_gripper()
        self.connect_tool_changer()
        self.connect_pipette()
        self.connect_camera()

        self.pipette_drop_tip_value = -8
        self.pipette_aspirate_value = 2.0
        self.pipette_dispense_value = -2.0
        self.droplet_value = 0.3

        self.gripper_close = 130 # 0-255 (255 is closed)
        self.griper_open = 0
        self.gripper_speed = 150 # 0-255
        self.gripper_force = 0 # 0-255

        print('Opening gripper...')
        self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)

        self.acceleration = 2
        self.velocity = 2
        self.robot_current_joint_angles = None

        self.get_movement_state()

        self.module_entry = [-0.1828145484680406, 0.1501917529215074, 0.4157045667286946, -0.014753354925067616, -3.133785224432585, -0.01020982277167234]
        self.module_entry_joint = [-1.3963525930987757, -2.1945158443846644, 2.1684568564044397, -1.5495260164937754, -1.5337546507464808, 3.2634336948394775]
        self.home = [-0.13358071546889347, -0.009673715752021885, 0.5890782758304143, -0.014566051910791617, -3.133734935087693, -0.010359747956377084]
        self.home_joint = [-1.355567757283346, -2.5413090191283167, 1.8447726408587855, -0.891581193809845, -1.5595606009112757, 3.3403327465057373]
        self.plate_exchange_1_above = [-0.18284724105645211, 0.7914820291585895, 0.41175512257988434, -0.014545475433050672, -3.1337759450718, -0.010278634391729295]
        self.plate_exchange_1 = [-0.1828537989205587, 0.7914917511283945, 0.390542100409092, -0.014571172649734884, -3.133719848650817, -0.010138239501312422]


        # TODO: Move these locations outside of the driver
        self.home = [0.07965465094358973, 0.1331286487752626, 0.46622559375534944, 0.8893786745124569, -1.4184312527016423, -1.5680218376629318]
        self.home_J = [2.017202138900757, -1.137721137409546, -0.9426093101501465, -2.6425615749754847, -4.693090263997213, -3.8424256483661097]
        self.pipette_loc = [-0.30710397664568057, 0.2223363316295067, 0.25346649921490616, 0.9780579194931717, -1.3456500374612195, -1.5122814896417478]
        self.pipette_loc_J = [2.8711442947387695, -1.8251310787596644, -1.5156354904174805, -1.3721376222423096, -4.720762554799215, -3.0886977354632776]
        self.pipette_approach = [-0.30710468347240427, 0.22234393663902577, 0.2793166289891617, 0.977973715529265, -1.3455795650125528, -1.512392593568845]
        self.pipette_approach_J = [2.8711442947387695, -1.8102451763548792, -1.412275791168213, -1.4902585309794922, -4.720990244542257, -3.088721577321188]
        self.pipette_above = [-0.3075719688934094, 0.2227307810713913, 0.40515454739546075, 0.9815940238325527, -1.3416684284127856, -1.504904936327573]
        self.pipette_above_J = [2.869802236557007, -1.9749981365599574, -0.5613865852355957, -2.1772977314391078, -4.720307175313131, -3.0981438795672815]
        self.tip1_loc = [0.049076405377552826, 0.35130426249264163, 0.063, 0.9759108742683295, -1.3350220046082053, -1.5092226077826993]
        self.tip1_approach = [0.049076095756735466, 0.3513032390285145, 0.083, 0.9758916159413838, -1.3350252553821587, -1.5092057412143818]
        self.tip1_above = [0.04908221782054774, 0.3513003341332178, 0.138, 0.9758574103691817, -1.3350463108315163, -1.5091909291569083]
        self.tip2_loc = [0.04909177440821851, 0.3411316353820866, 0.0628, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        self.tip2_approach = [0.04909177440821851, 0.3411316353820866, 0.083, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        self.tip2_above = [0.04909177440821851, 0.3411316353820866, 0.138, 0.977119433532159, -1.3337829736507698, -1.5108373189678133]
        self.sample1 = [0.15220619381604186, 0.21043816573205595, 0.09618091909170277, 1.444826407763332, -0.2548060433102738, -0.31289353129621067]
        self.sample1_above = [0.15220723461648447, 0.2104311001071656, 0.14402782259610025, 1.4448359749910735, -0.2548206714588542, -0.31295915781137074]
        self.sample2_above = [0.15279755520703087, 0.18939793717407497, 0.14402267332894347, 1.444821393022025, -0.25485812796155616, -0.3128929822914916]
        self.sample2 = [0.15186061464767017, 0.18822197623964088, 0.09490910394912143, 1.4440966224799245, -0.255613147568461, -0.3122426586441542]
        self.empty_tube = [0.15203368788019977, 0.16531582069324421, 0.12185568609417977, 1.4402850302548993, -0.2846256403901101, -0.3468228184833902]
        self.empty_tube_above = [0.15203001904780783, 0.16531236663764431, 0.14222620538915642, 1.4402337440190125, -0.2846450307479814, -0.346876615018759]
        self.well1 = [0.12772478460859046, 0.21370236710062357, 0.08390608100945282, 1.4380130231592743, -0.2414629895555231, -0.2954608172533908]
        self.well1_above = [0.12773445855037924, 0.21371308008717516, 0.1271232135439438, 1.4380596200664426, -0.24151536289689018, -0.2954919320386042]
        self.trash_bin_above = [0.187412530306272, 0.2868009561100828, 0.12712991727750073, 1.438076830279249, -0.2414934112798892, -0.2954944172453427]
        self.trash_bin = [0.1874179391982658, 0.2867862635600429, 0.013156853887081085, 1.438022625162957, -0.24148065729851562, -0.2954808450568972]
        
        # Establishing a connection with the camera using EPICS library.
        self.cam_acquire =  epics.PV("8idiARV1:cam1:Acquire")
        self.cam_image = epics.PV("8idiARV1:Pva1:Image")
        self.cam_capture =  epics.PV("8idiARV1:Pva1:Capture")

    def connect_ur(self):
        """
        Description: Create conenction to the UR robot
        """

        for i in range(10):
            try:
                self.ur5 = Robot(self.IP)
                sleep(2)

            except socket.error:
                print("Trying robot connection ...")
            else:
                print('Successful ur5 connection')
                break

    def connect_gripper(self):
        """
        Connect to the gripper
        """
        try:
            # GRIPPER SETUP:
            self.gripper = robotiq_gripper.RobotiqGripper()
            print('Connecting to gripper...')
            self.gripper.connect(self.IP, 63352)

        except Exception as err:
            print("Gripper error: ", err)

        else:
            if self.gripper.is_active():
                print('Gripper already active')
            else:
                print('Activating gripper...')
                self.gripper.activate()

    def connect_tool_changer(self):
        """
        Connect tool changer
        """

        try:
            # Establishing a connection with the tool changer using EPICS library.
            self.tool_changer = epics.PV("8idSMC100PIP:LJT7:1:DO0")

        except Exception as err:
            print("Tool changer error: ", err)

        else:
            print("Tool changer is connected.")

    def connect_pipette(self):
        """
        Connect pipette
        """

        try:
            # Establishing a connection with the pipette using EPICS library.
            self.pipette = epics.PV("8idQZpip:m1.VAL")

        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected.")
    def connect_camera(self):
        """
        Connect camera
        """

        try:
            # Establishing a connection with the pipette using EPICS library.
            self.pipette = epics.PV("8idQZpip:m1.VAL")

        except Exception as err:
            print("Pipette error: ", err)

        else:
            print("Pipette is connected.")

    def disconnect_ur(self):
        """
        Description: Disconnects the socket connection with the UR robot
        """
        self.ur5.close()
        print("Robot connection is closed.")

    def get_joint_angles(self):
        
        return self.ur5.getj()
    
    def get_cartesian_coordinates(self):
        
        return self.ur5.getl()
    
    def get_movement_state(self):
        current_location = self.get_joint_angles()
        current_location = [ '%.2f' % value for value in current_location] #rounding to 3 digits
        # print(current_location)
        if self.robot_current_joint_angles == current_location:
            movement_state = "READY"
        else:
            movement_state = "BUSY"

        self.robot_current_joint_angles = current_location

        return movement_state

    def pick(self, pick_goal):

        '''Pick up from first goal position'''

        above_goal = deepcopy(pick_goal)
        above_goal[2] += 0.05

        print('Moving to home position')
        # self.ur5.movel(self.home, self.acceleration, self.velocity)
        self.ur5.movej(self.home_joint, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur5.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur5.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to above goal position')
        self.ur5.movel(above_goal, self.acceleration, self.velocity)

        print('Moving to goal position')
        self.ur5.movel(pick_goal, self.acceleration, self.velocity)

        print('Closing gripper')
        self.gripper.move_and_wait_for_pos(self.gripper_close, self.gripper_speed, self.gripper_force)

        print('Moving back to above goal position')
        self.ur5.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur5.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur5.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to home position')
        # self.ur5.movel(self.home, self.acceleration, self.velocity)
        self.ur5.movej(self.home_joint, self.acceleration, self.velocity)


    def place(self, place_goal):

        '''Place down at second goal position'''

        above_goal = deepcopy(place_goal)
        above_goal[2] += 0.05

        print('Moving to home position')
        # self.ur5.movel(self.home, self.acceleration, self.velocity)
        self.ur5.movej(self.home_joint, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur5.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur5.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to above goal position')
        self.ur5.movel(above_goal, self.acceleration, self.velocity)

        print('Moving to goal position')
        self.ur5.movel(place_goal, self.acceleration, self.velocity)

        print('Opennig gripper')
        self.gripper.move_and_wait_for_pos(self.griper_open, self.gripper_speed, self.gripper_force)

        print('Moving back to above goal position')
        self.ur5.movel(above_goal, self.acceleration, self.velocity)

        print("Moving to the module entry location")
        # self.ur5.movel(self.module_entry, self.acceleration, self.velocity)
        self.ur5.movej(self.module_entry_joint, self.acceleration, self.velocity)

        print('Moving to home position')
        # self.ur5.movel(self.home, self.acceleration, self.velocity)
        self.ur5.movej(self.home_joint, self.acceleration, self.velocity)

        
    def transfer(self, pos1, pos2):
        ''''''
        self.ur5.set_tcp((0, 0, 0, 0, 0, 0))
        # robot.ur5.set_payload(2, (0, 0, 0.1))

        self.pick(pos1)
        self.place(pos2)
        print('Finished transfer')

if __name__ == "__main__":

    pos1= [-0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    pos2= [0.22575, -0.65792, 0.39271, 2.216, 2.196, -0.043]
    
    robot = URRobot()
    # robot.transfer(robot.plate_exchange_1,robot.plate_exchange_1)
    for i in range(1000):
        print(robot.get_movement_state())
        robot.get_overall_robot_status()
        sleep(0.5)

    robot.disconnect_ur()
