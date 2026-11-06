"""Runs against real ROS 2 when installed; no substitute ROS objects."""
import importlib.util
import subprocess
import time
import unittest


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS 2 rclpy is not installed')
class RosSmokeTests(unittest.TestCase):
    def test_simulation_topics_frames_and_clean_exit(self):
        import rclpy
        from nav_msgs.msg import Odometry, Path
        from geometry_msgs.msg import Twist
        from visualization_msgs.msg import MarkerArray

        rclpy.init()
        observer = rclpy.create_node('amr_lab_test_observer')
        received = {}
        subscriptions = []
        for suffix, message_type in (('odom', Odometry), ('planned_path', Path),
                                      ('cmd_vel', Twist), ('obstacles', MarkerArray)):
            subscriptions.append(observer.create_subscription(
                message_type, '/amr_lab/sim/' + suffix,
                lambda message, key=suffix: received.__setitem__(key, message), 10))
        process = subprocess.Popen(['ros2', 'run', 'amr_research_lab', 'sim_node',
                                    '--ros-args', '-p', 'max_steps:=80'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        published_topics = set()
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(observer, timeout_sec=0.1)
                if len(received) == 4:
                    published_topics.update(name for name, _ in
                        observer.get_publisher_names_and_types_by_node('amr_lab_simulation', '/'))
            output, _ = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, output)
            self.assertEqual(set(received), {'odom', 'planned_path', 'cmd_vel', 'obstacles'})
            self.assertEqual(received['odom'].header.frame_id, 'amr_lab/map')
            self.assertEqual(received['odom'].child_frame_id, 'amr_lab/base_link')
            self.assertEqual(received['planned_path'].header.frame_id, 'amr_lab/map')
            self.assertGreater(len(received['planned_path'].poses), 2)
            self.assertNotIn('/cmd_vel', published_topics)
            self.assertIn('/amr_lab/sim/cmd_vel', published_topics)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
