import threading
import unittest
from urllib.request import urlopen
from scratch_hci_bridge.command_state import CommandState
from scratch_hci_bridge.http_server import create_http_server
from scratch_hci_bridge.trajectory_scheduler import TrajectoryScheduler


class MotionStatusTest(unittest.TestCase):
    def test_building_running_and_failure(self):
        s=TrajectoryScheduler(batch_wait_sec=0,initial_mode_command_wait_sec=0)
        self.assertEqual(s.motion_status(),0)
        s.update_http_target('boom_angle',-50,now=1)
        self.assertEqual(s.motion_status(),1)
        s.update_from_joint_state(['swing_joint','boom_joint','arm_joint','bucket_joint'],[0,-1,2,1],now=1)
        goal=s.pending_snapshot(base_duration_sec=1,now=1)
        self.assertIsNotNone(goal)
        self.assertTrue(s.mark_submitted(goal))
        self.assertEqual(s.motion_status(),1)
        self.assertTrue(s.mark_completed(goal,successful=False))
        self.assertEqual(s.motion_status(),-1)

    def test_http_reporter(self):
        server=create_http_server('127.0.0.1',0,CommandState(),status_callback=lambda:1)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            with urlopen(f'http://127.0.0.1:{server.server_port}/?type=reporter&cmd=motion_status&val=0') as response:
                self.assertEqual(response.read(),b'busy')
                self.assertEqual(response.headers['Access-Control-Allow-Origin'],'*')
        finally:
            server.shutdown();server.server_close();thread.join()
