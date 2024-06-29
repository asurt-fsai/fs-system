import unittest
from unittest.mock import patch, MagicMock
from asurt_msgs.msg import NodeStatus
import rclpy
from rclpy.node import Node
from .helpers.module import Module  

class TestModule(unittest.TestCase):

    @patch('rclpy.create_node')
    def setUp(self, mock_create_node):
        mock_node = MagicMock(spec=Node)
        mock_create_node.return_value = mock_node
        self.pkg = "test_pkg"
        self.launch_file = "test_launch_file"
        self.heartbeat_topic = "heartbeat_topic"
        self.module = Module(pkg=self.pkg, launchFile=self.launch_file, heartbeat=self.heartbeat_topic)

    def test_initialization(self):
        self.assertEqual(self.module.pkg, self.pkg)
        self.assertEqual(self.module.launchFile, self.launch_file)
        self.assertEqual(self.module.state, NodeStatus.SHUTDOWN)
        self.assertIsNone(self.module.moduleHandle)
        self.assertFalse(self.module.scheduleRestart)
        self.assertTrue(self.module.hasHeartbeat)

    def test_repr(self):
        self.assertEqual(repr(self.module), f"{self.pkg} {self.launch_file}")

    @patch('launch.LaunchDescription')
    @patch('launch.actions.IncludeLaunchDescription')
    @patch('launch.substitutions.LaunchConfiguration')
    def test_launch(self, mock_launch_config, mock_include_launch_desc, mock_launch_desc):
        self.module.launch()
        self.assertEqual(self.module.state, NodeStatus.STARTING)

    @patch('intervalTimer.IntervalTimer.stop')
    def test_shutdown(self, mock_timer_stop):
        self.module.moduleHandle = MagicMock()
        self.module.hasHeartbeat = True
        self.module.heartbeartRateThread = MagicMock()
        self.module.shutdown()
        self.assertEqual(self.module.state, NodeStatus.SHUTDOWN)
        self.assertIsNone(self.module.moduleHandle)
        mock_timer_stop.assert_called_once()

    def test_heartbeat_callback(self):
        msg = NodeStatus()
        msg.status = NodeStatus.RUNNING
        self.module.heartbeatCallback(msg)
        self.assertEqual(self.module.state, NodeStatus.RUNNING)
        self.assertEqual(self.module.heartbeatCount, 1.0)

    @patch('time.time', return_value=1000)
    def test_update_heartbeat_rate(self, mock_time):
        self.module.lastHeartbeatTime = 500
        self.module.heartbeatCount = 10
        self.module.rate = 0.5
        self.module.updateHeartbeatRate()
        self.assertEqual(self.module.heartbeatCount, 0)
        self.assertEqual(self.module.rate, 3.001)  # calculated as 0.3*0.5 + (1-0.3)*(10/(500.001-500))

    def test_del(self):
        self.module.shutdown = MagicMock()
        del self.module
        self.module.shutdown.assert_called_once()

if __name__ == "__main__":
    unittest.main()
