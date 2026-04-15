#!/usr/bin/env python3
from ruikang_recon_baseline.competition_perception_node import CompetitionPerceptionNode
import rospy

if __name__ == '__main__':
    try:
        CompetitionPerceptionNode().spin()
    except rospy.ROSInterruptException:
        pass
