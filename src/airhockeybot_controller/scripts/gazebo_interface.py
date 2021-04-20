#!/usr/bin/env python
#-*- coding: utf-8 -*-

import time

import rospy

from   gazebo_msgs.msg             import ModelState, LinkStates, LinkState
from   gazebo_msgs.srv             import SetModelState, SetLinkState
from   std_srvs.srv                import Empty, EmptyRequest
from   std_msgs.msg                import Float64
from   sensor_msgs.msg             import JointState

# map vision units (mm) 
# to gazebo units (m)
MM_TO_M             = 0.001
# initial PUK position (link 2)
PUK_LINK_INIT_X     = 1
PUK_LINK_INIT_Y     = 1
PUK_LINK_INIT_Z     = 0.776
# initial PUK position (model)
PUK_MODEL_INIT_X    = PUK_LINK_INIT_X
PUK_MODEL_INIT_Y    = PUK_LINK_INIT_Y
PUK_MODEL_INIT_Z    = 1.0
# PUK identifiers in LinkStates/ ModelStates
PUK_MODEL_NAME      = "puk"
PUK_LINK_NAME       = "puk::link2"
PUK_REF_FRAME       = "world"
# initial BOT position
BOT_INIT_X          = 0.1   
BOT_INIT_Y          = 0.442
# goal location
X_POS_PLAYER_GOAL   = 0.4
X_POS_BOT_GOAL      = 0.0
Y_POS_GOAL_LEFT     = 0.4
Y_POS_GOAL_RIGHT    = 0.4
# index for lists
PUK_LINK_IDX        = 2
PLAYER_STATES_X_IDX = 2
PLAYER_STATES_Y_IDX = 3
X_POS_IDX           = 0
Y_POS_IDX           = 1
Z_POS_IDX           = 2

# dicts referenced in class, 
# initially contains init values
joint_states = {"joint_names": ['joint_x',
                                'joint_y',
                                'joint_player_x',
                                'joint_player_y'],
                "joint_pos":   [BOT_INIT_X, BOT_INIT_Y, 0.0, 0.0],
                "joint_vel":   [0.0, 0.0, 0.0, 0.0],
                "joint_eff":   [0.0, 0.0, 0.0, 0.0]
                }

# puk's link 2 pose:       position x,y,z
puk_state   = {"pose"  : [PUK_LINK_INIT_X, PUK_LINK_INIT_Y, PUK_LINK_INIT_Z,
#                          orientation x, y, z, w
                          0.0, 0.0, 0.0, 0.0],
#                          lin twist x,y,z, ang twist x, y, z
               "twist" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
               }

def joint_callback(data):
    """Update the joint state dict referenced in class."""
    joint_states.update({"joint_names": data.name,
                         "joint_pos":   data.position,
                         "joint_vel":   data.velocity,
                         "joint_eff":   data.effort
                        })

def link_callback(data):
    """Update the puk state dict referenced in class."""
    puk_state.update({  "pose": [data.pose[PUK_LINK_IDX].position.x,    
                                 data.pose[PUK_LINK_IDX].position.y,    
                                 data.pose[PUK_LINK_IDX].position.z,   
                                 data.pose[PUK_LINK_IDX].orientation.x, 
                                 data.pose[PUK_LINK_IDX].orientation.y, 
                                 data.pose[PUK_LINK_IDX].orientation.z, 
                                 data.pose[PUK_LINK_IDX].orientation.w
                                ],
                       "twist": [data.twist[PUK_LINK_IDX].linear.x,
                                 data.twist[PUK_LINK_IDX].linear.y,
                                 data.twist[PUK_LINK_IDX].linear.z,
                                 data.twist[PUK_LINK_IDX].angular.x,
                                 data.twist[PUK_LINK_IDX].angular.y,
                                 data.twist[PUK_LINK_IDX].angular.z
                                ]
                    })

class GazeboInterface:
    """Connection to Gazebo simulation."""

    def __init__(self):
         # Service Clients
        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/gazebo/set_link_state')
        self.link_state_client  = rospy.ServiceProxy('/gazebo/set_link_state',  SetLinkState)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.pause              = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # Start simulation
        self.unpause_physics()
        # Subscriber
        rospy.Subscriber("/airhockeybot/joint_states", JointState,  joint_callback)
        rospy.Subscriber("/gazebo/link_states",        LinkStates,  link_callback)
        # Publisher
        self.pub_x        = rospy.Publisher('/airhockeybot/joint_x_position/command',        Float64, queue_size=1)
        self.pub_y        = rospy.Publisher('/airhockeybot/joint_y_position/command',        Float64, queue_size=1)
        self.pub_player_x = rospy.Publisher('/airhockeybot/player_joint_x_position/command', Float64, queue_size=1)
        self.pub_player_y = rospy.Publisher('/airhockeybot/player_joint_y_position/command', Float64, queue_size=1)
        # Member vars updated by callbacks
        self.joint_states = joint_states
        self.puk_state    = puk_state

    def get_goal(self):
        """ If PUK x position is 
            off the table -> reset and
            return goal hits """ 
        bot_goal    = False
        player_goal = False
        # get PUK's x,y,z position
        puk_x = puk_state["pose"][X_POS_IDX]
        # PUK in player goal
        if puk_x > X_POS_PLAYER_GOAL:
            self.reset_puck_state()
            player_goal = True
            # wait to not double count goal
            time.sleep(1)
        # PUK in bot goal
        elif puk_x < X_POS_BOT_GOAL:
            self.reset_puck_state()
            bot_goal = True
            # wait to not double count goal
            time.sleep(1)
        
        return bot_goal, player_goal

    def reset_puck_state(self):
        """Set puk to init pose."""
        model_msg = ModelState()
        model_msg.model_name      = PUK_MODEL_NAME
        model_msg.reference_frame = PUK_REF_FRAME
        model_msg.pose.position.x = PUK_MODEL_INIT_X
        model_msg.pose.position.y = PUK_MODEL_INIT_Y
        model_msg.pose.position.z = PUK_MODEL_INIT_Z
        link_msg = LinkState()
        link_msg.link_name        = PUK_LINK_NAME
        link_msg.reference_frame  = PUK_REF_FRAME
        link_msg.pose.position.x  = PUK_LINK_INIT_X
        link_msg.pose.position.y  = PUK_LINK_INIT_Y
        link_msg.pose.position.z  = PUK_LINK_INIT_Z
        try:
            self.model_state_client(model_msg)
            self.link_state_client(link_msg)
        except rospy.ServiceException:
            rospy.logerr("Service call puk model failed!!!")

    def move_puck(self,
                  twist_lin_x=0.0, twist_lin_y=0.0, twist_lin_z=0.0,
                  twist_ang_x=0.0, twist_ang_y=0.0, twist_ang_z=0.0):
        """Move puk in ang direction with lin velocity."""
        msg = LinkState()
        msg.link_name          = PUK_LINK_NAME
        msg.reference_frame    = PUK_REF_FRAME
        msg.pose.position.x    = self.puk_state["pose"][X_POS_IDX]
        msg.pose.position.y    = self.puk_state["pose"][Y_POS_IDX]
        msg.pose.position.z    = self.puk_state["pose"][Z_POS_IDX]
        # movement commands 
        msg.twist.linear.x     = twist_lin_x
        msg.twist.linear.y     = twist_lin_y
        msg.twist.linear.z     = twist_lin_z
        msg.twist.angular.x    = twist_ang_x
        msg.twist.angular.y    = twist_ang_y
        msg.twist.angular.z    = twist_ang_z
        try:
            self.link_state_client(msg)
        except rospy.ServiceException:
            rospy.logerr("Service call puk model failed!!!")

    def unpause_physics(self):
        """Simulation started paused due to joint initializations."""
        rospy.loginfo("Gazebo interface unpauses simulation in 1s!")
        time.sleep(1)
        try:
            self.pause(EmptyRequest())
        except rospy.ServiceException:
            rospy.logerr("Failed to unpause simulation!!!")

    def bot_pos_xy(self, action):
        """Move bot to action = (x pos, y pos)."""
        self.pub_x.publish(Float64(action[X_POS_IDX] * MM_TO_M))
        self.pub_y.publish(Float64(action[Y_POS_IDX] * MM_TO_M))

    def bot_add_pos_x(self, action):
        """Increments actual joint position."""
        pos = joint_states['joint_pos'][X_POS_IDX]
        self.pub_x.publish(Float64(pos + action))

    def bot_add_pos_y(self, action):
        """Increments actual joint position."""
        pos = joint_states['joint_pos'][Y_POS_IDX]
        self.pub_y.publish(Float64(pos + action))

    def player_pos_xy(self, action):
        """Move player bat to action = (x pos, y pos)."""
        self.pub_player_x.publish(Float64(action[X_POS_IDX]))
        self.pub_player_y.publish(Float64(action[Y_POS_IDX]))

    def player_add_pos_xy(self, action):
        """Move player bat to incremented action
            = (x pos + action x, y pos + action y)."""
        x_pos = joint_states['joint_pos'][PLAYER_STATES_X_IDX]
        y_pos = joint_states['joint_pos'][PLAYER_STATES_Y_IDX]
        self.pub_player_x.publish(Float64(action[X_POS_IDX] + x_pos))
        self.pub_player_y.publish(Float64(action[Y_POS_IDX] + y_pos))

    def player_add_pos_x(self, action):
        """Increments actual joint position."""
        pos = joint_states['joint_pos'][PLAYER_STATES_X_IDX]
        self.pub_player_x.publish(Float64(pos + action))

    def player_add_pos_y(self, action):
        """Increments actual joint position."""
        pos = joint_states['joint_pos'][PLAYER_STATES_Y_IDX]
        self.pub_player_y.publish(Float64(pos + action))

    def check_periodic_messages(self, waittime=0.1):
        """Not used here."""
        pass


if __name__ == "__main__":
    pass