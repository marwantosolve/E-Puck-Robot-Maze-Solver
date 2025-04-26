"""my_controller_follow_theWall controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# Constants
MAX_SPEED = 6.28
OBSTACLE_THRESHOLD = 100  # Threshold to detect obstacles in front
WALL_THRESHOLD = 150      # Threshold to detect being too close to the left wall
NO_WALL_THRESHOLD = 100   # Threshold to detect being too far from the left wall


def run_robot(robot):
  """Wall following robot"""

  # get the time step of the current world.
  timestep = int(robot.getBasicTimeStep())

  # Enable motors
  left_motor = robot.getDevice('left wheel motor')
  right_motor = robot.getDevice('right wheel motor')

  left_motor.setPosition(float('inf'))
  left_motor.setVelocity(0.0)

  right_motor.setPosition(float('inf'))
  right_motor.setVelocity(0.0)

  # Enable proximity sensors
  ps = []
  sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
  for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

  # Main loop:
  # - perform simulation steps until Webots is stopping the controller
  while robot.step(timestep) != -1:
    # Read specific sensors
    ps_values = {name: sensor.getValue()
                 for name, sensor in zip(sensor_names, ps)}

    front_obstacle = ps_values['ps7'] > OBSTACLE_THRESHOLD or ps_values['ps0'] > OBSTACLE_THRESHOLD
    too_close_to_wall = ps_values['ps5'] > WALL_THRESHOLD
    too_far_from_wall = ps_values['ps5'] < NO_WALL_THRESHOLD

    left_speed = MAX_SPEED * 0.8  # Default speed
    right_speed = MAX_SPEED * 0.8  # Default speed

    if front_obstacle:
      # Obstacle detected in front, turn right sharply
      print("Obstacle detected! Turning right.")
      left_speed = MAX_SPEED * 0.7
      right_speed = -MAX_SPEED * 0.3
    else:
      # Wall following logic
      if too_close_to_wall:
        # Too close to the left wall, turn right gently
        print("Too close to wall. Turning right gently.")
        left_speed = MAX_SPEED * 0.8
        right_speed = MAX_SPEED * 0.6
      elif too_far_from_wall:
        # Too far from the left wall, turn left gently
        print("Too far from wall. Turning left gently.")
        left_speed = MAX_SPEED * 0.6
        right_speed = MAX_SPEED * 0.8
      else:
        # Maintain distance, drive straight
        print("Following wall.")
        # Speeds remain at default

    # Enter here functions to send actuator commands, like:
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

  # Enter here exit cleanup code.


if __name__ == "__main__":
  # create the Robot instance.
  my_robot = Robot()
  run_robot(my_robot)
