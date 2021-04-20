#!/usr/bin/env python

from std_msgs.msg import Int8
import rospy

# winner flag
PLAYING          = 0
BOT_WON          = 1
PLAYER_WON       = 2

class GoalState():
    """ Holds and publishes current
        state of goals and current winner. """

    def __init__(self):
        self.bot_goals        = 0
        self.player_goals     = 0
        # publisher
        self.pub_bot_goals    = rospy.Publisher('/airhockeybot/bot_goals',    Int8, queue_size=1)
        self.pub_player_goals = rospy.Publisher('/airhockeybot/player_goals', Int8, queue_size=1)
        self.pub_game_state   = rospy.Publisher('/airhockeybot/game_state',   Int8, queue_size=1)


    def update_goal_state(self, bot_goal_hit, player_goal_hit, reset):
        """ 
            Update and publish goal state

            bot_goal_hit:    boolean, flag if bot goal was hit
            player_goal_hit: boolean, flag if player goal was hit
            reset:           boolean, flag if state should be reset

        """
        # increment goals or reset state
        if bot_goal_hit:
            self.player_goals += 1
        elif player_goal_hit:
            self.bot_goals += 1
        if reset is True:
            self.reset_state()

        # publish game state and reset goals
        if self.bot_goals == 10:
            self.pub_game_state.publish(Int8(BOT_WON))
            self.reset_state()
        elif self.player_goals == 10:
            self.pub_game_state.publish(Int8(PLAYER_WON))
            self.reset_state()
        else:
            self.pub_game_state.publish(Int8(PLAYING))

        self.pub_bot_goals.publish(Int8(self.bot_goals))  
        self.pub_player_goals.publish(Int8(self.player_goals))

    def reset_state(self):
        """
            Reset goal state
        """
        self.bot_goals = 0
        self.player_goals = 0

if __name__ == "__main__":
    pass
