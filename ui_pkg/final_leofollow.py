
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, PublisherSubscriber, StartNode
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time 
import threading
from std_msgs.msg import Bool,Int32

# 쓰레기통 앞 좌표
front_trash = [17.644,0.22942]
clubroom_front = [15.928,0.032135]
credit_point = [28.539,6.5206] #7.2155 엘베봤을때z = 0.0148, w = 0.9998
#강의실방향 봤을때 z = -0.7296 w = 0.684


trailer_goal_state = [11.831, -0.14858]

charge_pose = [4.45531, -0.3803]

# Shelf positions for picking / pickingup file
shelf_positions = {
    'shelf_A': [-3.829, -7.604],
    'shelf_B': [-3.791, -3.287],
    'shelf_C': [-3.791, 1.254],
    'shelf_D': [-3.24, 5.861]}

# Shipping destination for picked products
shipping_destinations = {
    'recycling': [-0.205, 7.403],
    'pallet_jack7': [28.073, 0.8],
    'conveyer_432': [6.217, 2.153],
    'frieght_bay_3': [-6.349, 9.147]}

#adding2
class DestinationNode(Node):

    def __init__(self, navigator):
        super().__init__('destination')
        self.navigator = navigator  # navigator 객체를 설정
        self.subscription = self.create_subscription(
            Int32,
            'change_destination',
            self.listener_callback,
            10)
        
        self.received = Int32()

    def listener_callback(self, msg):
        self.get_logger().info('Received response: "%s"' % msg.data)
        self.received.data = msg.data 
        if self.received.data == 1:  # 예를 들어, 1을 받으면 클럽룸으로 설정
            self.set_destination_to_clubroom()
        else:  # 그 외의 경우 트래시로 설정
            self.set_destination_to_trash()

    def set_destination_to_clubroom(self):
        clubroom_front = [15.928, 0.032135]
        self.set_destination(clubroom_front)

    def set_destination_to_trash(self):
        front_trash = [17.644, 0.22942]
        self.set_destination(front_trash)

    def set_destination(self, position):
        # PoseStamped 객체 설정
        shipping_destination = PoseStamped()
        shipping_destination.header.frame_id = 'map'
        shipping_destination.header.stamp = self.navigator.get_clock().now().to_msg()
        shipping_destination.pose.position.x = position[0]
        shipping_destination.pose.position.y = position[1]
        shipping_destination.pose.orientation.z = 0.0
        shipping_destination.pose.orientation.w = 1.0
##adding


def main():

    rclpy.init()
    logger = rclpy.logging.get_logger('my_logger')
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = clubroom_front[0]
    initial_pose.pose.position.y = clubroom_front[1]
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)


    talk_with_arduino = PublisherSubscriber()
    ###adding
    start_navigation = StartNode()

    while rclpy.ok():
        rclpy.spin_once(start_navigation)
        start_call = start_navigation.start_received 

        if start_call.data == True: break
        else:
            print("Navigation don't received start call")      
    ##aadding 

    docking_pub = Bool()
    docking_pub.data = False

    #목적지 설정 (PoseStamped로 전달) - credit_point is fixed
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = credit_point[0]
    goal_pose.pose.position.y = credit_point[1]
    goal_pose.pose.orientation.z = -0.6958
    goal_pose.pose.orientation.w = 0.7198

    path1 = navigator.getPath(initial_pose, goal_pose, planner_id="GridBased")

    # navigator.goToPose(goal_pose)
    navigator.followPath(path1)
    result = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Remaining distance: {feedback}")
    
    result = navigator.getResult()
    time.sleep(2)    
    

    while rclpy.ok():
        rclpy.spin_once(talk_with_arduino)

        if result == TaskResult.SUCCEEDED:
            docking_pub.data = True  # 주행 성공 시 True 값 퍼블리시
            talk_with_arduino.publish_docking.publish(docking_pub)
            print("Navigation succeeded, docking started")
        else:
            print("Navigation failed or canceled")       

        task = talk_with_arduino.received
        logger.info(f'docking_state : {docking_pub}, received :{task.data}, result : {result}')

        if task.data==2 : break #trailer_start topic 받아서 트렉-트레일러 주행시작


    # talk_with_arduino.destroy_node()
    logger.info('Gripper success')
    print('Waiting for 5 seconds before moving to the next shelf location...')
    logger.info('Waiting for 3 seconds')
    time.sleep(5)


    destination = DestinationNode(navigator)

    shipping_destination = PoseStamped()
    shipping_destination.header.frame_id = 'map'
    shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    shipping_destination.pose.position.x = front_trash[0]
    shipping_destination.pose.position.y = front_trash[1]
    shipping_destination.pose.orientation.z = 0.0
    shipping_destination.pose.orientation.w = 1.0

    # navigator.goToPose(shipping_destination)
    path2 = navigator.getPath(goal_pose, shipping_destination, planner_id="SmacPlanner")
    navigator.followPath(path2)

    # i = 0
    # while not navigator.isTaskComplete():
        # Do something with the feedback
        # i += 1
        # feedback = navigator.getFeedback()
        # if feedback and i % 20 == 0:
        #     print('Estimated distance remaining to goal position: ' +
        #         '{0:.3f}'.format(feedback.distance_to_goal) +
        #         '\nCurrent speed of the robot: ' +
        #         '{0:.3f}'.format(feedback.speed))
        #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
        #             print('Navigation has exceeded timeout of 180s, canceling request.')
        #             navigator.cancelTask()

    result2 = navigator.getResult()
    if result2 == TaskResult.SUCCEEDED:
        grip_connect = Int32() #그리퍼 여는곳
        grip_connect.data = 3
        talk_with_arduino.trailer_pub.publish(grip_connect)
        print('Route complete! Restarting...')
        pass
    elif result2 == TaskResult.CANCELED:
        print('Security route was canceled, exiting.')
        exit(1)
    elif result2 == TaskResult.FAILED:
        print('Security route failed! Restarting from other side...')

    exit(0)
    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()