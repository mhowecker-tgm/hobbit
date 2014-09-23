#!/usr/bin/python
# -*- coding: utf-8 -*-

PKG = 'hobbit_smach'
NAME = 'test_mmui'

import rospy

from hobbit_msgs import MMUIInterface as MMUI


def main():
    rospy.init_node(NAME, anonymous=False)

    menues = ['M_Audio, M_Entertain', 'M_Game',
              'M_Memory', 'M_Solitaire', 'M_Chess',
              'M_Simon', 'M_Sudoku', 'M_Muehle',
              'M_eBook', 'M_Picture', 'M_Internet',
              'M_Social', 'M_Checklist', 'M_Manual']

    mmui = MMUI.MMUIInterface()
    for menu in menues:
        rospy.loginfo(menu)
        resp = mmui.GoToMenu(menu)
        rospy.loginfo(str(resp))
        rospy.sleep(2)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        return
