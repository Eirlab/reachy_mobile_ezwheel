#!/usr/bin/env python3

import rospy
from reachy_mobile_games.srv import TicTacToe, TicTacToeResponse


def handle_TicTacToe(data):
    rospy.loginfo(data)
    return TicTacToeResponse(True)

def TicTacToe_server():
    rospy.init_node('TicTacToe_server')
    s = rospy.Service('TicTacToe', TicTacToe, handle_TicTacToe)
    print("Ready to play TicTacToe.")
    rospy.spin()

if __name__ == "__main__":
    TicTacToe_server()