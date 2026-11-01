"""A stand-in for ROS's rospy (Python 2 and 3, as on the robot), enough to run robot_stream.REMOTE (the robot link's node) on the laptop in tests.
Events come from the file named in FAKE_ROS_FEED, one per line, appended while the node runs:
  gait [U]           a firmware gait frame on /hal_angles_set (from /gait_node): every servo U raw units, or 65535
  imu EX EY GX GY GZ an IMU report on /hal_gyro_report: euler-x, euler-y, gyro x y z
  turn start NAME    hal starts NAME.hts (a line on /rosout_agg); turn finish NAME: it ends"""
import os
import threading
import time

from rosgraph_msgs.msg import Log
from ubt_msgs.msg import angles_set, gyro_report

_subs, _down = {}, threading.Event()


def init_node(name, anonymous=False, disable_signals=False):
    th = threading.Thread(target=_feed)
    th.daemon = True
    th.start()


def get_name():
    return "/yanshee_sway_stream_test"


def is_shutdown():
    return _down.is_set()


def signal_shutdown(reason):
    _down.set()


class Publisher(object):
    def __init__(self, topic, typ, queue_size=1):
        self.topic = topic

    def publish(self, msg):
        pass

    def get_num_connections(self):
        return 1


def Subscriber(topic, typ, cb, queue_size=1):
    _subs.setdefault(topic, []).append(cb)


def ServiceProxy(name, typ):
    def call(*a):
        raise RuntimeError("no service %s here" % name)
    return call


def _emit(topic, msg):
    for cb in _subs.get(topic, []):
        cb(msg)


def _feed():
    path = os.environ.get("FAKE_ROS_FEED")
    if not path:
        return
    pos = 0
    while not _down.is_set():
        try:
            with open(path) as f:
                f.seek(pos)
                lines = f.readlines()
                pos = f.tell()
        except OSError:
            lines = []
        for line in lines:
            p = line.split()
            if p and p[0] == "gait":
                m = angles_set()
                m.time, m.angles = 20, [int(p[1]) if len(p) > 1 else 65535] * 17
                m._connection_header = {"callerid": "/gait_node"}
                _emit("/hal_angles_set", m)
            elif p and p[0] == "imu" and len(p) == 6:
                m = gyro_report()
                v = [float(x) for x in p[1:]]
                m.euler_data, m.gyro_data, m.accel_data, m.compass_data = (v[0], v[1], 0.0), tuple(v[2:]), (0, 1, 0), (0, 0, 0)
                _emit("/hal_gyro_report", m)
            elif len(p) == 3 and p[0] == "turn":
                _emit("/rosout_agg", Log("Publish action[/opt/yanshee/lib/move/hts/%s.hts] status[%s] " % (p[2], p[1]) +
                                         "timestamp[0]"))
        time.sleep(0.02)
