from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True, PX4=False)
drone = Drone(conn)
drone.start()
drone.take_control()
drone.arm()
drone.set_home_as_current_position()