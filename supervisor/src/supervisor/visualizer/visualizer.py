#!/usr/bin/python3
"""
.

"""
from typing import List, Any
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
import rospy
from supervisor.supervisor_functions.module import Module


class Visualizer:
    """
    Visualizes the data on rviz.
    """

    def __init__(self, publishName: str) -> None:
        self.counter = 0
        self.publishName = publishName
        self.publisher = rospy.Publisher(publishName, MarkerArray, queue_size=1)

    def detsToMarker(
        self, x: float, y: float, text: str, colType: int, frameId: str, idx: int
    ) -> Marker:
        """
        Converts the detections to a marker.
        """
        msg = Marker()
        msg.header.frame_id = frameId
        msg.header.stamp = rospy.Time.now()
        msg.ns = str(self.counter)
        msg.id = idx
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.ADD
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.scale.z = 0.5
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 1
        msg.text = text

        color = [1, 1, 1]
        if colType in [0, 1]:  # Yellow
            color = [0, 1, 1]
        elif colType in [3, 4, 5]:
            color = [1, 0, 0]

        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0
        return msg

    def delMsg(self, frameId: str) -> Marker:
        """
        Creates a delete message.
        """
        msgToDel = Marker()
        msgToDel.header.frame_id = frameId
        msgToDel.header.stamp = rospy.Time.now()
        msgToDel.type = Marker.TEXT_VIEW_FACING
        msgToDel.action = Marker.DELETEALL
        return msgToDel

    def visualizeText(self, textArr: List[Any]) -> None:
        """
        Visualizes the text on rviz.

        Parameters
        ----------
        textArr : list
            The list of text to visualize.
        """
        frameId = "map"
        markers = [self.delMsg(frameId)]
        self.counter += 1

        for idx, det in enumerate(textArr):
            markers.append(self.detsToMarker(det[0], det[1], det[2], det[3], frameId, idx))
        msg = MarkerArray()
        msg.markers = markers
        self.publisher.publish(msg)


class IntMarkers:  # pylint: disable=too-few-public-methods
    """
    Creates interactive markers for the supervisor on rviz.
    """

    def __init__(self, btnTopic: str) -> None:
        self.server = InteractiveMarkerServer(btnTopic)

    def addButton(self, x: float, y: float, module: Module) -> None:
        """
        Creates an interactive marker button for the module.
        Parameters
        ----------
        x : float
            The x coordinate of the marker.
        y : float
            The y coordinate of the marker.
        module : Module
            The module object.
        """
        intMarker = InteractiveMarker()
        intMarker.header.frame_id = "map"
        intMarker.name = module.pkg
        intMarker.description = ""
        intMarker.pose.position.x = x
        intMarker.pose.position.y = y
        intMarker.pose.orientation.w = 1

        boxMarker = Marker()
        boxMarker.type = Marker.CUBE
        boxMarker.pose.orientation.w = 1
        boxMarker.scale.x = 0.8
        boxMarker.scale.y = 0.45
        boxMarker.scale.z = 0.45
        boxMarker.color.r = 0
        boxMarker.color.g = 1
        boxMarker.color.b = 0
        boxMarker.color.a = 1

        buttonControl = InteractiveMarkerControl()
        buttonControl.interaction_mode = InteractiveMarkerControl.BUTTON
        buttonControl.always_visible = True
        buttonControl.markers.append(boxMarker)
        intMarker.controls.append(buttonControl)

        def handleInput(markerFeedBack: InteractiveMarkerFeedback) -> None:
            """
            Handles the input from the interactive marker.
            Parameters
            ----------
            input : InteractiveMarkerFeedback
                The input from the interactive marker.
            """
            if markerFeedBack.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
                module.scheduleRestart = 1

        self.server.insert(intMarker, handleInput)
        self.server.applyChanges()
