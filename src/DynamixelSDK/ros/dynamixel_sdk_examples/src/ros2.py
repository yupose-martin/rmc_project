import rclpy
import rclpy.qos
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
import numpy as np
import time
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
import math
import array
import tf2_ros
import tf2_py as tf2
from tf2_ros.transform_listener import TransformListener
from pan_tilt_msgs.msg import PanJointState,PanTiltCmdDeg
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations

#0.005 0.01
class ArmController(Node):
    def __init__(self):
        super().__init__("ArmController")
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.aruco_sub = self.create_subscription(PoseArray,"/aruco_poses",self.ar_cb, 10)
        self.odom_sub = self.create_subscription(Odometry,"/odom",self.odom_callback, rclpy.qos.qos_profile_system_default)
        self.pub_timer = self.create_timer(0.5, self.timers_cb)
        self.pantil_deg_cmd = PanTiltCmdDeg()
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg,"/pan_tilt_cmd_deg",10)
        self.cmd_twist = Twist()
        self.twist_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()

        self.lastOdom = None
        self.currentOdom = None
        
        self.cnt = 0
        self.thred = 0.05
        self.waist_thred = 0.02
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')
        self.ar_pos = np.array([0.0, 0.0, 0.0])
        self.ar_quat = np.array([0.0, 0.0, 0.0, 0.0])
        self.rotation_matrix = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]])
        self.homogeneous_matrix = None
        self.transform_matrix = np.array([[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]])
        self.goal_matrix = None
        self.guess_list = None
        self.res = 0.1

        self.moving = False

        self.machine_state = "INIT"

        self.nav_state = "START"

        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure*(self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))
        self.tf_buffer=tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.27
        self.initial_pose.pose.position.y = -0.38
        self.initial_pose.pose.orientation.z = -0.707
        self.initial_pose.pose.orientation.w = 0.707
        self.navigator.setInitialPose(self.initial_pose)
        
        self.navigator.lifecycleStartup()

        
        self.global_costmap = None
        self.local_costmap = None
        self.path = None
        self.currentOdom = None
        self.lastOdom = None
        self.startyaw = None

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 0.32
        self.goal_pose.pose.position.y = -3.0
        self.goal_pose.pose.orientation.z = -0.707
        self.goal_pose.pose.orientation.w = 0.707

        self.current_theta = - math.pi / 2

        self.navigator.waitUntilNav2Active()
        self.navigator.changeMap('/home/tony/ros2_ws/files/map.yaml')
        self.navigator.clearAllCostmaps()
        self.global_costmap = self.navigator.getGlobalCostmap()
        self.local_costmap = self.navigator.getLocalCostmap()
        self.path = self.navigator.getPath(self.initial_pose, self.goal_pose)

        pass


    def odom_callback(self,msg):
        self.currentOdom = msg
        # print("odom get")



    def cam2arm(self):
        '''
        need to write
        :return:  transform matrix between camera and arm
        '''
        print("calc!!!!!!!!!!!!!!!!!!!")
        fix_x = 0.01
        fix_y = 0.005 #原本是0
        fix_z = 0.02
        if not self.transform_matrix[3, 3] == 0 and not self.homogeneous_matrix[3, 3] == 0:
            self.goal_matrix = np.dot(self.transform_matrix, self.homogeneous_matrix)
            self.goal_matrix[0, 3] = self.goal_matrix[0, 3] + fix_x
            self.goal_matrix[1, 3] = self.goal_matrix[1, 3] + fix_y
            self.goal_matrix[2, 3] = self.goal_matrix[2, 3] + fix_z
            angle = -math.atan2(self.goal_matrix[1, 3], self.goal_matrix[0, 3])
            self.goal_matrix[:3, :3] = np.array([[math.cos(angle), math.sin(angle), 0], [-math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
        # self.goal_matrix[:3, :3] = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])
        # print("goal_matrix:")
        # print(self.goal_matrix)
        return None


    def js_cb(self, msg):
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])

    def ar_cb(self,msg):
        self.ar_pos = np.array([msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z])
        self.ar_quat = np.array([msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w])
        rot = R.from_quat(self.ar_quat)
        self.rotation_matrix = rot.as_matrix()
        self.homogeneous_matrix = np.eye(4)
        self.homogeneous_matrix[:3, :3] = self.rotation_matrix
        self.homogeneous_matrix[:3, 3] = self.ar_pos
        self.homogeneous_matrix[3, 3] = 1.0
        # print("homogeneous_matrix:")
        # print(self.homogeneous_matrix)
        if self.goal_matrix is None:
            self.cam2arm()

    # demo
    def timers_cb(self):
        # self.nav_state = "TEST"#调试机械臂
        match self.nav_state:
            case "TEST":
                print("test")
                if not self.currentOdom is None:
                    self.lastOdom = self.currentOdom
                    rpy = tf_transformations.euler_from_quaternion([self.lastOdom.pose.pose.orientation.x, self.lastOdom.pose.pose.orientation.y, self.lastOdom.pose.pose.orientation.z, self.lastOdom.pose.pose.orientation.w])[2]
                    print(rpy)
                    # self.nav_state = "TEST2"

            case "TEST2":
                print("test2")
                self.cmd_twist.angular.z = 0.15
                self.twist_pub.publish(self.cmd_twist)
                rpy = tf_transformations.euler_from_quaternion([self.currentOdom.pose.pose.orientation.x, self.currentOdom.pose.pose.orientation.y, self.currentOdom.pose.pose.orientation.z, self.currentOdom.pose.pose.orientation.w])
                print(rpy)
                self.startyaw = None

            case "START": #S to A
                if self.release():
                    self.navigator.goToPose(self.goal_pose)
                    self.nav_state = "START1"
            
            case "START1":
                print("START\n")
                if self.navigator.isTaskComplete():
                    self.lastOdom = self.currentOdom
                    print("next1\n")
                    self.nav_state = "ARM_CATCH"

            case "ARM_CATCH": #A
                #修正因为找物块而造成的odom偏移
                # self.lastOdom = self.currentOdom
                if len(self.joint_pos) == 7:
                    # print(self.machine_state)
                    match self.machine_state:
                        case "INIT":#设置pantil角度
                            self.pantil_deg_cmd.pitch = 15.0
                            self.pantil_deg_cmd.yaw = 0.0
                            self.pantil_deg_cmd.speed = 10
                            self.pantil_pub.publish(self.pantil_deg_cmd)
                            self.thred = 0.15
                            # print(self.joint_pos)
                            if self.go_sleep_pos() and self.release():
                                print('go sleep pos done!')
                                self.machine_state = "NEXT1"
                                # time.sleep(1.0)
                        case "NEXT1":
                            toFrameRel = 'px100/base_link'
                            fromFrameRel = 'camera_color_optical_frame'
                            now = rclpy.time.Time()
                            try:
                                transformStamped = self.tf_buffer.lookup_transform(toFrameRel, fromFrameRel, now)
                                tf_quat = np.array([transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w])
                                tf_trans = np.array([transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z])
                                rot_tf = R.from_quat(tf_quat)
                                rot_tf_matrix = rot_tf.as_matrix()
                                self.transform_matrix[:3, :3] = rot_tf_matrix
                                self.transform_matrix[:3, 3] = tf_trans
                                self.transform_matrix[3, 3] = 1.0
                                # print("transform_matrix:")
                                # print(self.transform_matrix)
                                if self.go_head_pos0():
                                    self.machine_state = "NEXT2"
                                    print("go to Next2!")
                                    # time.sleep(1.0)
                            except tf2.ConnectivityException:
                                print("error1")
                            except tf2.ExtrapolationException:
                                print("error2")
                            except tf2.InvalidArgumentException:
                                print("error3")
                            except tf2.LookupException:
                                print("error4")
                            except tf2.TimeoutException:
                                print("error5")
                        case "NEXT2":
                            # if self.set_group_pos([1.0, 0.0, 0.5, -0.6]) == True and self.grasp(-0.5):
                            #     print('NEXT2 control done!')
                            #     self.machine_state = "NEXT3"
                            #     time.sleep(1.0)
                            # pass
                            self.moving = False
                            s = self.ar_pos[2]
                            x_drift = self.ar_pos[0]
                            if self.homogeneous_matrix is None:
                                print("finding...")
                                self.cmd_twist.linear.x = 0.0
                                self.cmd_twist.angular.z = 0.15
                            elif abs(x_drift) > 0.03:
                                print(x_drift)
                                self.cmd_twist.linear.x = 0.0
                                self.cmd_twist.angular.z = -x_drift / abs(x_drift) * 0.15
                            elif s > 0.37:#原先是0.35
                                print("approaching...")
                                self.cmd_twist.linear.x = 0.15
                                self.cmd_twist.angular.z = 0.0
                            else:
                                print("arrived.")
                                self.cmd_twist.linear.x = 0.0
                                self.cmd_twist.angular.z = 0.0
                                if self.homogeneous_matrix is not None:
                                    if self.release():
                                        print('NEXT2 control done!')
                                        self.machine_state = "NEXT3"
                            self.twist_pub.publish(self.cmd_twist)
                        case "NEXT3":
                            # T_sd = np.array([
                            #                 [0.03, 0.99, 0.17, 0.02],
                            #                 [-0.22, 0.17, -0.96, -0.1],
                            #                 [-0.98,  0.0, 0.22, 0.05],
                            #                 [ 0.0, 0.0, 0.0, 1.0]])
                            # print("goal_matrix:", self.goal_matrix)
                            # print("homogeneous_matrix", self.homogeneous_matrix)
                            if self.goal_matrix is None:
                                print("matrix is none")
                                self. machine_state = "NEXT1"
                                return
                            self.thred = 0.1
                            # print(self.goal_matrix)
                            # print(self.goal_matrix[0, 3])
                            in_goal_matrix = np.array([[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0],[0.0, 0.0, 0.0, 0.0]])
                            for i in range(4):
                                for j in range(4):
                                    in_goal_matrix[i, j] = self.goal_matrix[i, j]
                            in_goal_matrix[0, 3] = self.goal_matrix[0, 3] - 0.02
                            in_goal_matrix[1, 3] = self.goal_matrix[1, 3]
                            in_goal_matrix[2, 3] = self.goal_matrix[2, 3] + 0.07
                            # print('goal', in_goal_matrix)
                            self.res = 0.1
                            mlist, mflag, mfinish = self.matrix_control(in_goal_matrix)
                            self.guess_list = mlist
                            if self.goal_matrix is None:
                                self. machine_state = "NEXT1"
                                return
                            if mflag == True:
                                self.moving = True
                            self.fk = self.joint_to_pose(mlist)
                            # print('fk', self.fk)
                            # print('mlist:', mlist)
                            if mflag == True and mfinish == True:
                                print('matrix control1 done!')
                                print('goal1', in_goal_matrix)
                                print('fk1', self.fk)
                                # self.gripper_controller(0.7, 2.0)
                                # check_pos = self.joint_pos
                                # print(check_pos[4])
                                self.machine_state = "FINISHED0"

                        case "FINISHED0":
                            self.thred = 0.1
                            self.res = 0.05
                            mlist, mflag, mfinish = self.matrix_control(self.goal_matrix, custom_guess=self.guess_list)
                            self.fk = self.joint_to_pose(mlist)
                            # print('fk', self.fk)
                            # print('mlist:', mlist)
                            if mflag == True and mfinish == True:
                                print('matrix control2 done!')
                                self.fk = self.joint_to_pose(mlist)
                                print('goal2', self.goal_matrix)
                                print('fk2', self.fk)
                                self.gripper_controller(0.7, 0.5)
                                self.machine_state = "FINISHED1"

                        case "FINISHED1":
                            self.thred = 0.15
                            self.homogeneous_matrix = None
                            self.goal_matrix = None
                            if self.go_head_pos():
                                self.machine_state = "FINISHED2"

                        case "FINISHED2":
                            if self.go_head_pos0():
                                self.machine_state = "FINISHED3"

                        case "FINISHED3":
                            if self.go_sleep_pos():
                                self.machine_state = "NEXT1"
                                # self.nav_state = "GOTO_B"
                                self.nav_state = "TURN_AROUND"
                        

            case "TURN_AROUND":
                start_yaw = tf_transformations.euler_from_quaternion([self.lastOdom.pose.pose.orientation.x, self.lastOdom.pose.pose.orientation.y, self.lastOdom.pose.pose.orientation.z, self.lastOdom.pose.pose.orientation.w])[2]
                current_yaw = tf_transformations.euler_from_quaternion([self.currentOdom.pose.pose.orientation.x, self.currentOdom.pose.pose.orientation.y, self.currentOdom.pose.pose.orientation.z, self.currentOdom.pose.pose.orientation.w])[2]
                turned_yaw = current_yaw - start_yaw
                if turned_yaw < - math.pi:
                    turned_yaw += 2 * math.pi

                if turned_yaw < math.pi / 2 - 0.1:
                    self.cmd_twist.angular.z = 0.55
                    self.twist_pub.publish(self.cmd_twist)
                elif turned_yaw > math.pi / 2 + 0.1:
                    self.cmd_twist.angular.z = -0.55
                    self.twist_pub.publish(self.cmd_twist)
                else:
                    self.cmd_twist.angular.z = 0.0
                    self.twist_pub.publish(self.cmd_twist)
                    self.nav_state = "GOTO_B"


            case "GOTO_B":
                print("goto_b\n")
                self.navigator.lifecycleStartup()
                self.navigator.waitUntilNav2Active()
                self.navigator.clearAllCostmaps()
                self.global_costmap = self.navigator.getGlobalCostmap()
                self.local_costmap = self.navigator.getLocalCostmap()
                self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                self.goal_pose.pose.position.x = 2.7
                self.goal_pose.pose.position.y = -3.2
                self.goal_pose.pose.orientation.z = 0.0
                self.goal_pose.pose.orientation.w = 1.0
                # self.path = self.navigator.getPath(self.initial_pose, self.goal_pose)
                self.navigator.goToPose(self.goal_pose)
                self.nav_state = "TO_B_PATH"

            case "TO_B_PATH":
                if self.navigator.isTaskComplete():
                    if self.go_release_pos() and self.release():
                        print("next2\n")
                        self.currentOdom = None
                        self.nav_state = "READ_ANGLE2"

            case "READ_ANGLE2":
                if self.go_sleep_pos() and not self.currentOdom is None:
                    self.lastOdom = self.currentOdom
                    self.nav_state = "TURN2"

            case "TURN2":
                start_yaw = tf_transformations.euler_from_quaternion([self.lastOdom.pose.pose.orientation.x, self.lastOdom.pose.pose.orientation.y, self.lastOdom.pose.pose.orientation.z, self.lastOdom.pose.pose.orientation.w])[2]
                current_yaw = tf_transformations.euler_from_quaternion([self.currentOdom.pose.pose.orientation.x, self.currentOdom.pose.pose.orientation.y, self.currentOdom.pose.pose.orientation.z, self.currentOdom.pose.pose.orientation.w])[2]
                turned_yaw = current_yaw - start_yaw
                if turned_yaw < - math.pi:
                    turned_yaw += 2 * math.pi

                if turned_yaw < math.pi * 3 / 4 - 0.1:
                    self.cmd_twist.angular.z = 0.55
                    self.twist_pub.publish(self.cmd_twist)
                elif turned_yaw > math.pi * 3 / 4 + 0.1:
                    self.cmd_twist.angular.z = -0.55
                    self.twist_pub.publish(self.cmd_twist)
                else:
                    self.cmd_twist.angular.z = 0.0
                    self.twist_pub.publish(self.cmd_twist)
                    self.nav_state = "BACK"

            case "BACK":
                print("back\n")
                self.navigator.lifecycleStartup()
                self.navigator.waitUntilNav2Active()
                self.navigator.clearAllCostmaps()
                self.global_costmap = self.navigator.getGlobalCostmap()
                self.local_costmap = self.navigator.getLocalCostmap()
                self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
                self.goal_pose.pose.position.x = 0.27
                self.goal_pose.pose.position.y = -0.38
                self.goal_pose.pose.orientation.z = 0.202
                self.goal_pose.pose.orientation.w = 0.979
                self.navigator.goToPose(self.goal_pose)
                self.nav_state = "BACK_PATH"
            
            case "BACK_PATH":
                if self.navigator.isTaskComplete():
                    print("task finished\n")
                    exit(0)
        pass



    def cam_pos_to_arm_pos(self):
        '''
        need to write
        :return:  position from camera coordinate to arm coordinate
        '''
        return None

    def set_single_pos(self, name, pos, blocking=True):
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 

        '''
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]
                        cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        cal_name = 'gripper'
                    case _:
                        print('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos-check_pos)
                        if dis < thred:
                            return True
                        else:
                            # print('single joint moving...')
                            return False                       
                    case "gripper":
                        return True

        pass

    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            print('unexpect length of list!')
        else:
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            self.group_pub.publish(self.arm_group_command)
        
            thred = self.thred
            waist_thred = self.waist_thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    # print('current group pos:', check_pos)
                    # print('current set pos  :', pos_list)
                    if np.abs(pos_list[0] - check_pos[0]) < waist_thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        print('current group pos:', check_pos)
                        print('current set pos  :', pos_list)
                        return True
                    else:
                        # if np.abs(pos_list[0] - check_pos[0]) >= thred:
                        #     print('waist moving...')
                        # if np.abs(pos_list[1] - check_pos[1]) >= thred:
                        #     print('shoulder moving...')
                        # if np.abs(pos_list[2] - check_pos[2]) >= thred:
                        #     print('elbow moving...')
                        # if np.abs(pos_list[3] - check_pos[3]) >= thred:
                        #     print('wrist moving...')
                        return False            
            pass

    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home_pos(self):
        state = self.set_group_pos([0.0, 0.0, 0.0, 0.0])
        return state

    def go_sleep_pos(self):
        state = self.set_group_pos([-1.4, -0.35, 0.7, 1.0])
        return state
    
    def go_head_pos(self):
        state = self.set_group_pos([0.1, 0.0, -1.3, 1.4])
        return state
    
    def go_head_pos0(self):
        state = self.set_group_pos([-1.4, 0.0, -1.3, 1.4])
        return state
    
    def go_release_pos(self):
        state = self.set_group_pos([-1.4, 0.3, 0.3, -0.15])
        return state


    def matrix_control(self, T_sd, custom_guess: list[float]=None, execute: bool=True):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=self.res,
                ev=self.res,
            )
            solution_found = True
            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                # print('success',success)
                theta_list = self._wrap_theta_list(theta_list)
                # solution_found = self._check_joint_limits(theta_list)
                if len(theta_list) == 4:
                    solution_found = True
                else:
                    solution_found = False
            else:
                solution_found = False

            if solution_found:
                if execute:
                    joint_list = [theta_list[0],theta_list[1],theta_list[2], theta_list[3]]
                    finish = self.set_group_pos(joint_list)
                    self.T_sb = T_sd
                return theta_list, True, finish

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        # print("no solution found!")
        self.goal_matrix = None
        self.machine_state = "NEXT2"
        return theta_list, False, False


    def waist_control(self, pos):
        """
        lower limit = -1.5
        upper limit = 1.5
        """
        pos = float(pos)
        state = self.set_single_pos('waist', pos)
        return state
    
    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = ~0.9
        """
        pos = float(pos)
        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)
        state = self.set_single_pos('elbow', pos)
        return state
    
    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)
        state = self.set_single_pos('wrist_angle', pos)
        return state


    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state


    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.
        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def release(self, delay: float = 0.5) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    def grasp(self, pressure: float = 0.5, delay: float = 0.5) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state


    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        """
        Wrap an array of joint commands to [-pi, pi) and between the joint limits.

        :param theta_list: array of floats to wrap
        :return: array of floats wrapped between [-pi, pi)
        """
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        theta_list_new = theta_list
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list_new = np.delete(theta_list, x)
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list_new = np.delete(theta_list, x)
        return theta_list_new



def main():
    rclpy.init(args=None)
    contoller = ArmController()
    rclpy.spin(contoller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()