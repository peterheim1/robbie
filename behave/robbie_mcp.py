#!/usr/bin/env python
'''


'''


import rospy
import time
from std_msgs.msg import Float32, String, Int16
from geometry_msgs.msg import Twist
from pi_trees_ros.pi_trees_ros import *
from robbie.task_setup import *
from rbx2_msgs.srv import *
from robbie.robbie_tasks import *
from collections import OrderedDict
from math import pi, sqrt
#from face_recognition.msg import *
from festival.srv import *


# A class to track global variables
class BlackBoard():
    def __init__(self):
        # A list to store rooms and tasks
        self.task_list = list()
        
        # The robot's current position on the map
        self.robot_position = Point()

# Initialize the black board
black_board = BlackBoard()

# Create a task list mapping rooms to tasks.
black_board.task_list = OrderedDict([
    ('living_room', [Vacuum(room="living_room", timer=5)]),
    ('kitchen', [Mop(room="kitchen", timer=7)]),
    ('bathroom', [Scrub(room="bathroom", timer=9), Mop(room="bathroom", timer=5)]),
    ('hallway', [Vacuum(room="hallway", timer=5)])
    ])

class UpdateTaskList(Task):
    def __init__(self, room, task, *args, **kwargs):
        name = "UPDATE_TASK_LIST_" + room.upper() + "_" + task.name.upper()
        super(UpdateTaskList, self).__init__(name)    
        
        self.name = name
        self.room = room
        self.task = task

    def run(self):
        try:
            black_board.task_list[self.room].remove(self.task)
            if len(black_board.task_list[self.room]) == 0:
                del black_board.task_list[self.room]
        except:
            pass
        
        return TaskStatus.SUCCESS


class Patrol():
    def __init__(self):
        rospy.init_node("mcp_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        # Initialize a number of parameters and variables
        setup_task_environment(self)

        #+n_waypoints = 4
        self.person = "peter"
        
       
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        
        # Assign the docking station pose to a move_base action task
        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, reset_after=True)

  
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        NAVIGATION = Selector("NAVIGATION")
      
        # Create the patrol loop decorator
        LOOP_PATROL = Loop("LOOP_PATROL", iterations= 4)
        
        # Add the two subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(LOOP_PATROL)
        
        # Create the patrol iterator
        PATROL = Iterator("PATROL")
        goal1 = MoveBaseGoal()
        goal1.target_pose.header.frame_id = 'map'
        goal1.target_pose.header.stamp = rospy.Time.now()
        goal1.target_pose.pose = self.work_station_pose
        NAV_WORK_TASK = SimpleActionTask("NAV_WORK_TASK", "move_base", MoveBaseAction, goal1, reset_after=False)
        GUARD = Guard()
        
        # Add the move_base tasks to the patrol task
        #for task in n_waypoints:
        PATROL.add_child(NAV_WORK_TASK)
        #PATROL.add_child(GUARD)
  
        # Add the patrol to the loop decorator
        #LOOP_PATROL.add_child(PATROL)

        #with NAVIGATION:
            
            #NAVIGATION.add_child(NAV_WORK_TASK)


        # Add the battery check and recharge tasks to the "stay healthy" task
        with STAY_HEALTHY:
            # The check battery condition (uses MonitorTask)
            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
            
            # The charge robot task (uses ServiceTask)
            AUTODOCK = AutoDock()#
            CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)

      
            # Build the recharge sequence using inline construction
            RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, AUTODOCK, CHARGE_ROBOT])
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)

       
        

        # Display the tree before beginning execution
        print "Robbie Behavior Tree"
        print_tree(BEHAVE)
        #online call
        #self.speak_text_service(self.noon1 + "  Robbie is on line" + " the time is   " + self.local)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)


    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < 12.5:
                #rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
                #self.speak_text_service("my battery level is low     moving to charger")
                return TaskStatus.FAILURE
            else:
                return TaskStatus.SUCCESS
    
    def recharge_cb(self, msg):
        rospy.loginfo("BATTERY CHARGED!")
        


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #self.move_base.cancel_all_goals()
        #self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Patrol()


