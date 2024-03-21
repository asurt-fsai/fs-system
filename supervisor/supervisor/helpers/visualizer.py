"""
Visualizer class for visualizing text data on rviz and adding restart buttons for modules
"""
from __future__ import annotations
from typing import List, Any
import time

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from .module import Module


class Visualizer(Node):
    """
    Visualizes text data on rviz using MarkerArray and adds restart buttons for modules using
    interactive markers

    Parameters
    ----------
    markersTopic : str
        The topic to publish the text markers to
    btnTopic : str
        The topic to publish the buttons to
    """

    def __init__(self, markersTopic: str, btnTopic: str) -> None:
        self.markersTopic = markersTopic
        self.btnTopic = btnTopic

        self.textPublisher = self.create_publisher(MarkerArray,markersTopic, 10 )
        #rospy.Publisher(markersTopic, MarkerArray, queue_size=10)
        self.btnServer = InteractiveMarkerServer(btnTopic)
        self.counter = 0

    def textToMarker(
        self, x: float, y: float, text: str, colType: str, frameId: str, idx: int
    ) -> Marker:
        """
        Converts the given text, location, and color to a marker

        Parameters
        ----------
        x : float
            The x coordinate of the text
        y : float
            The y coordinate of the text
        text : str
            The text to display
        colType : str
            The color of the text
            One of "r", "g", "y", default is white
        frameId : str
            The frame id of the marker
        idx : int
            The id of the marker in the marker array it will be part of

        Returns
        -------
        Marker
            The marker representing the text
        """
        msg = Marker()
        msg.header.frame_id = frameId
        msg.header.stamp = time.time()
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

        color = [1, 1, 1]  # White (Default)
        if colType == "y":
            color = [0, 1, 1]  # Yellow
        elif colType == "r":
            color = [1, 0, 0]  # Red
        elif colType == "g":
            color = [0, 1, 0]  # Green

        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0
        return msg

    def delMsg(self, frameId: str) -> Marker:
        """
        Creates a delete message

        Parameters
        ----------
        frameId : str
            The frame id of the message

        Returns
        -------
        Marker
            The delete message
        """
        msgToDel = Marker()
        msgToDel.header.frame_id = frameId
        msgToDel.header.stamp = time.time()
        msgToDel.type = Marker.TEXT_VIEW_FACING
        msgToDel.action = Marker.DELETEALL
        return msgToDel

    def visualizeText(self, textArr: List[List[Any]]) -> None:
        """
        Visualizes the text on rviz.

        Parameters
        ----------
        textArr : list
            The list of text to visualize each row is one text element
            The row should contain four elements:
            [x:float, y:float, text:str, color:str]
            Color can be "y" for yellow, "r" for red, "g" for green
            Otherwise, color will be white
        """
        frameId = "map"
        markers = [self.delMsg(frameId)]
        self.counter += 1

        for idx, text in enumerate(textArr):
            markers.append(self.textToMarker(*(*text[:4], frameId, idx)))
        msg = MarkerArray()
        msg.markers = markers

        self.textPublisher.publish(msg)

    def addButton(self, x: float, y: float, module: Module) -> None:
        """
        Creates an interactive marker button for restarting the module.
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
                module.restart()

        self.btnServer.insert(intMarker, handleInput)
        self.btnServer.applyChanges()

    def deleteAndReturnNewVisualizer(self) -> Visualizer:
        """
        Deletes the current visualizer and returns a new one.
        """
        markersTopic = self.markersTopic
        btnTopic = self.btnTopic
        del self
        return Visualizer(markersTopic, btnTopic)
