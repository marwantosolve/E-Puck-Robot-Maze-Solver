"""my_controller_follow_theWall controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robot(robot):
  """Wall following robot"""

  # get the time step of the current world.
  timestep = int(robot.getBasicTimeStep())
  max_speed = 6.28

  # Enable motors
  left_motor = robot.getDevice('left wheel motor')
  right_motor = robot.getDevice('right wheel motor')

  left_motor.setPosition(float('inf'))
  left_motor.setVelocity(0.0)

  right_motor.setPosition(float('inf'))
  right_motor.setVelocity(0.0)

  # Enable proximity sensors
  prox_sensors = []
  for ind in range(8):
    sensor_name = 'ps' + str(ind)
    prox_sensors.append(robot.getDevice(sensor_name))
    prox_sensors[ind].enable(timestep)

  # Main loop:
  # - perform simulation steps until Webots is stopping the controller
  while robot.step(timestep) != -1:
    # Read the sensors:
    for ind in range(8):
      print("ind:{}, val: {}".format(ind, prox_sensors[ind].getValue()))

    # Process sensor data here.
    left_wall = prox_sensors[5].getValue() > 80
    left_corner = prox_sensors[6].getValue() > 80
    front_wall = prox_sensors[7].getValue() > 80

    left_speed = max_speed
    right_speed = max_speed

    if front_wall:
      print("Turn right in place")
      left_speed = max_speed
      right_speed = max_speed/8

    else:

      if left_wall:
        print("Drive forward")
        left_speed = max_speed
        right_speed = max_speed

      else:
        print("Turn left")
        left_speed = max_speed/8
        right_speed = max_speed

      if left_corner:
        print("came too close, drive right")
        left_speed = max_speed
        right_speed = max_speed/8

    # Enter here functions to send actuator commands, like:
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

  # Enter here exit cleanup code.


if __name__ == "__main__":
  # create the Robot instance.
  my_robot = Robot()
  run_robot(my_robot)
