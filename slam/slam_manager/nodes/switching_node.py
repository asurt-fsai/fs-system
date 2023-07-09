#!/usr/bin/python3
"""
switching_node used to switch between aloam and hdl after the first lap
"""
import rospy

from slam_manager import SwitchingNode  # type: ignore[attr-defined]


def main() -> None:
    """
    Main function
    """
    rospy.init_node("switching_node")
    lidarTopic = rospy.get_param("/switching_node/lidar_topic")
    slamLidarTopic = rospy.get_param("/switching_node/slam_lidar_topic")
    roadStateTopic = rospy.get_param("/switching_node/road_state_topic")
    aloamOdomTopic = rospy.get_param("/switching_node/aloam_odom_topic")
    hdlOdomTopic = rospy.get_param("/switching_node/hdl_odom_topic")
    initPoseTopic = rospy.get_param("/switching_node/init_pose_topic")
    odomPubTopic = rospy.get_param("/switching_node/odom_pub_topic")
    loadMapTopic = rospy.get_param("/switching_node/load_map_topic")
    initPosePubTopic = rospy.get_param("/switching_node/init_pose_pub_topic")
    dataDir = rospy.get_param("/switching_node/data_dir")
    minPcCountToStart = rospy.get_param("/switching_node/min_pc_count_to_start")
    switchingNode = SwitchingNode(
        lidarTopic,
        slamLidarTopic,
        roadStateTopic,
        aloamOdomTopic,
        hdlOdomTopic,
        initPoseTopic,
        odomPubTopic,
        loadMapTopic,
        initPosePubTopic,
        dataDir,
        minPcCountToStart,
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        switchingNode.run()
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        pass
