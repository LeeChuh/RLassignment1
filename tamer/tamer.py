import rclpy
from rclpy.node import Node

from xarm_msgs.srv import PlanJoint
from xarm_msgs.srv import PlanExec
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
import random
import sys
from keras.models import load_model
import argparse
import numpy as np


class xarmJointPlanningClient(Node):

    def __init__(self):
        super().__init__('xarm_joint_planning_client')
        self.plan_cli = self.create_client(PlanJoint, 'xarm_joint_plan')
        self.exec_cli = self.create_client(PlanExec, 'xarm_exec_plan')
         

    def plan_and_execute(self,pose):
        while not self.plan_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.plan_req = PlanJoint.Request()
        self.plan_req.target = pose
        print(pose)
        self.plan_future = self.plan_cli.call_async(self.plan_req)
        rclpy.spin_until_future_complete(self, self.plan_future)
        res = self.plan_future.result()
        if res.success:
            self.get_logger().info('Planning success executing...')
            while not self.exec_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            self.exec_req = PlanExec.Request()
            self.exec_req.wait = True
            self.exec_future = self.exec_cli.call_async(self.exec_req)
            rclpy.spin_until_future_complete(self, self.exec_future)
            return self.exec_future.result()
        else:
            self.get_logger().info('Planning Failed!')
            return False



class PolicyPublisher(Node):

    def __init__(self, env, goal, task, goal_pose, decoy_pose, model):
        super().__init__('policy_publisher')
        self.env = env
        self.goal = goal
        self.task = task
        self.goal_pose = goal_pose
        self.decoy_pose = decoy_pose
        self.model = model
        self.client = xarmJointPlanningClient()
        self.joint_pose = None
        self.prev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.preprev = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.ee_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # We publish end-effector commands to this topic
        #self.publisher_ = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)

        # This will get the next action from the policy every 0.1 seconds
        self.policy_timer = self.create_timer(0.5, self.policy_callback)

        # This will check the robot's current end-effector pose every 0.1 seconds
        self.tf_timer = self.create_timer(0.1, self.eef_callback)

    def joint_callback(self, msg):
        joint_pose = [msg.position[4], msg.position[0], msg.position[1], msg.position[5], msg.position[2], msg.position[3], msg.position[6]]
        self.joint_pose = np.array(joint_pose)

    def square_root_difference(self, values, reference):
        return np.sqrt(np.sum(np.square(np.array(values) - np.array(reference))))

    def calcualte_reward(self, pose, goal, decoy):
        weight1 = np.array([1, 0, 0, 0, 0])
        weight2 = np.array([1, -0.2, -0.2, -0.2, -0.2])
        weight3 = np.array([1, 0.2, 0.2, 0.2, 0.2])
        distance_goal = self.square_root_difference(pose, goal)
        distance_decoy1 = self.square_root_difference(pose, decoy[0])
        distance_decoy2 = self.square_root_difference(pose, decoy[1])
        distance_decoy3 = self.square_root_difference(pose, decoy[2])
        distance_decoy4 = self.square_root_difference(pose, decoy[3])
        raw_score = np.array([distance_goal, distance_decoy1, distance_decoy2, distance_decoy3, distance_decoy4])
        return np.array([np.exp(- np.dot(weight1, raw_score)), np.exp(- np.dot(weight2, raw_score)), np.exp(- np.dot(weight3, raw_score))])

    def eef_callback(self):
        # Look up the end-effector pose using the transform tree
        try:
            t = self.tf_buffer.lookup_transform(
                "link_base", #to_frame_rel,
                "link_eef", #from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not get transform: {ex}')
            return

        self.ee_pose = t.transform.translation

    def sample_actions(self, current_pose):
        action_0 = [-0.5, 0, 0.5]
        action_1 = [-0.5, 0, 0.5]
        action_2 = [-0.5, 0, 0.5]
        action_3 = [-0.5, 0, 0.5]
        action_4 = [-1, 0, 1]
        action_5 = [-0.5, 0, 0.5]
        action_6 = [-0.5, 0, 0.5]
        future_poses = []
        for a in action_0:
            for b in action_1:
                for c in action_2:
                    for d in action_3:
                        for e in action_4:
                            for f in action_5:
                                for g in action_6:
                                    rand = random.random()
                                    action = [a, b, c, d, e, f, g]
                                    pose = current_pose + action
                                    if not (min(pose) < -2 or max(pose) > 2):
                                        future_poses.append(pose)
        return future_poses

    def policy_callback(self):
        if self.ee_pose is None:
            print("Waiting for ee pose...")
            return
        if self.joint_pose is None:
            print("Waiting for joint pose...")
            return
        current_pose = np.array([self.ee_pose.x, self.ee_pose.y, self.ee_pose.z])
        pose_in_world = np.array([-0.2 - current_pose[1], -0.5 + current_pose[0], 1.021 + current_pose[2]])
        distance = np.sqrt(np.sum((pose_in_world - self.goal_pose) ** 2))
        if distance > 0.05:
            reward = []
            current_joint = self.joint_pose
            future_poses = self.sample_actions(current_joint)
            input_states = []
            for future_pose in future_poses:
                input_state = np.append(future_pose, self.goal_pose)
                input_state = np.append(input_state, self.decoy_pose)

                #using the trained model
                input_states.append(input_state)
            input_states = np.array(input_states)
            reward = self.model.predict(input_states)[self.task]

            #using the ground truth reward
            #future = np.array([-0.2-future_pose[1], -0.5+future_pose[0], 1.021+future_pose[2]])
            #r = self.calcualte_reward(future_pose, self.goal_pose, self.decoy_pose)[self.task]

            #reward.append(r)
            index = np.argmax(reward)
            result = future_poses[index]
            # Convert the action vector into a Twist message
            '''
            twist = TwistStamped()
            twist.twist.linear.x = 0.0 #result[0]
            twist.twist.linear.y = 0.0 #result[1]
            twist.twist.linear.z = 0.0 #result[2]
            twist.twist.angular.x = 0.0 #random.uniform(-1, 1)
            twist.twist.angular.y = 0.0 #random.uniform(-1, 1)
            twist.twist.angular.z = 0.0 #random.uniform(-1, 1)
            twist.header.frame_id = "link_base"
            twist.header.stamp = self.get_clock().now().to_msg()
            '''

            # Convert the end_effector_pose to joint_pose
            # target_pose = self.convert(end_effector_pose)

            # some example target_pose
            #target_pose = [-1.261259862292687, 1.0791625870230162, 1.3574703291922603, 1.7325127677684549, -1.0488170161118582, 1.4615630500372134, -1.505248122305602]
            response = self.client.plan_and_execute([result[0], result[1], result[2], result[3], result[4], result[5], result[6]])
            print(response)
            if response:
                self.preprev = self.prev
                self.prev = [result[0], result[1], result[2], result[3], result[4], result[5], result[6]]
            if not response:
                self.client.plan_and_execute(self.preprev)

            #self.publisher_.publish(twist)

def main(args=None):
    env1_goal = np.array([[-0.035, 0.142, 1.245], [-0.15, 0.142, 1.245], [-0.266, 0.142, 1.245], [-0.383, 0.142, 1.245]])
    env2_goal = np.array([[-0.035, 0.042, 1.345], [-0.15, 0.042, 1.245], [-0.266, 0.142, 1.345], [-0.383, 0.042, 1.245]])
    env3_goal = np.array([[-0.035, 0.042, 1.145], [-0.15, 0.142, 1.245], [-0.266, 0.142, 1.245], [-0.383, 0.042, 1.145]])
    goals = np.array([env1_goal, env2_goal, env3_goal])
    env1_decoy = np.array([[-0.383379, 0.165331, 1.14576], [-0.183635, 0.239076, 1.31077], [-0.153111, 0.110957, 1.29395], [0.022497, 0.180345, 1.1708]])
    env2_decoy = np.array([[-0.416498, 0.078665, 1.32583], [-0.196313, 0.310306, 1.34149], [-0.133594, 0.184351, 1.34316], [-0.074792, 0.148929, 1.43017]])
    env3_decoy = np.array([[-0.344683, 0.024459, 1.20518], [-0.16924, 0.226221, 1.2826], [-0.099357, 0.12819, 1.32999], [-0.024455, 0.073603, 1.2148]])
    decoys = np.array([env1_decoy, env2_decoy, env3_decoy])

    parser = argparse.ArgumentParser()

    parser.add_argument("-env", "--environment", dest = "env", default = 1, help="Environment number", type=int)
    parser.add_argument("-g", "--goal", dest = "g", default = 1, help="Goal number", type=int)
    parser.add_argument("-task", "--task",dest ="task", default = 1, help="Task number", type=int)
    parser.add_argument("-number", "--number", dest = "number", default = 435, help = "Number of Training Samples 435, 326, 217, or 108", type = int)

    args = parser.parse_args()

    env = args.env - 1 # 1~3
    g = args.g - 1 # 1~8
    task = args.task - 1 # 1~3
    number = args.number

    model_name = "435_training_sample_model_joint_space.h5"

    if number == 108:
        model_name = "108_training_sample_model_joint_space.h5"
    elif number == 217:
        model_name = "217_training_sample_model_joint_space.h5"
    elif number == 326:
        model_name = "326_training_sample_model_joint_space.h5"


    #if we want to load model
    model = load_model(model_name)

    #if we don't want to load model
    #model = None


    rclpy.init(args=None)

    policy_publisher = PolicyPublisher(env, g, task, goals[env][g], decoys[env], model)

    rclpy.spin(policy_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    policy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()