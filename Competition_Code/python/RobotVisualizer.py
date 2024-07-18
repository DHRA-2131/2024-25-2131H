import pygame as pg
import numpy as np
import math
import time

# START OF CONFIG --------------------------
# =========== ROBOT VARIABLES =========== #


# X, Y, Theta
START_STATE = (10, 10, 0) 
# X, Y
DESIRED_STATE = (72, 72)
# Inches
TRACK_WIDTH = 20.0
# MaxVel, MaxAccel, MaxDecel
ROBOT_PARAM_LINEAR = (64.8, 64.8*2, 64.8*2)
ROBOT_PARAM_ANGULAR = (ROBOT_PARAM_LINEAR[0] / TRACK_WIDTH, ROBOT_PARAM_LINEAR[1] / TRACK_WIDTH, ROBOT_PARAM_LINEAR[2] / TRACK_WIDTH)
# =========== PYGAME VARIABLES =========== #


WINDOW_SIZE = (800.0, 800.0) # Px * Px
LINE_WIDTH = 2 # Px
NODE_RADIUS = 4 # Px
DESIRED_RADIUS = 6 # Px
START_RADIUS = 6


WINDOW_COLOR = pg.Color("#000000") # Light Grey (#667475), white (#ffffff), black
LINE_COLOR = pg.Color("#393a3b") # Medium Grey 
PATH_NODE_COLOR = pg.Color("#e86107") # Black (#000000), purple (#9F2B68), Green (#69e807), Orange
DECAY_NODE_COLOR = pg.Color("#17cde6") # Medium Grey (#393a3b), Pink (#e81cb2), Steel blue

TRAJECTORY_NODE_COLOR = pg.Color("#FFFF00") # Yellow

DESIRED_COLOR = pg.Color("#00FF00") # Green
START_COLOR = pg.Color("#ED0A3F") # Red

# Window Size in Inches
WINDOW_X_INCHES = 144
WINDOW_Y_INCHES = 144


FPS = 60

# --------------------------- END OF CONFIG


class Point:
  def __init__(self, x, y):
    self.x = x
    self.y = y;

  def magnitude(self, PointB):
    return math.sqrt((PointB.x - self.x)*(PointB.x - self.x) + (PointB.y - self.y)*(PointB.y - self.y))
  

  def amplitude(self, PointB):
    return math.atan2( PointB.y - self.y ,  PointB.x - self.x)
  

  def rotate(self, Angle):
  
  
    # Calculate rotated point
    newX = self.y * math.sin(Angle) + self.x * math.cos(Angle)
    newY = self.y * math.cos(Angle) + self.x * -math.sin(Angle)

    # Set new Point
    self.x = newX
    self.y = newY
    return self
  
  def __iadd__(self, PointB):
    self.x += PointB.x
    self.y += PointB.y
    return self
  
  def __isub__(self, PointB):
    self.x -= PointB.x
    self.y -= PointB.y
    return self
    
  def __imul__(self, Scalar):
    self.x *= Scalar
    self.y *= Scalar
    return self
  
  def __mul__(self, Scalar):
    self.x *= Scalar
    self.y *= Scalar
    return self
  
  def __itruediv__(self, Scalar):
    self.x /= Scalar
    self.y /= Scalar
    return self
  
class robot:
  def __init__(self, Pos: Point, theta, linearVelocity, angularVelocity, maxVelocities: tuple):
    self.pos = Pos
    self.theta = theta
    self._linearVelocity = linearVelocity
    self._angularVelocity = angularVelocity
    self._maxLinearVelocity, self._maxAngularVelocity = maxVelocities
      
  def setVelocities (self, linearVelocity, angularVelocity):
    max(-self._maxLinearVelocity, min(linearVelocity, self._maxLinearVelocity))
    max(-self._maxAngularVelocity, min(angularVelocity, self._maxAngularVelocity))
        
    self._linearVelocity = linearVelocity
    self._angularVelocity = angularVelocity
  
  def moveStep(self, fps):
    self.theta += self._angularVelocity / fps
    self.pos += Point(self._linearVelocity  / fps, 0).rotate(-self.theta)
    

class MotionProfile:
  
  def __init__(self, distance, maxVelocity, maxAccel, maxDecel):
    self._distance = distance
    self._maxVelocity = maxVelocity
    self._maxAccel = math.fabs(maxAccel)
    self._maxDecel = math.fabs(maxDecel)
    self._minDistance = maxVelocity * maxVelocity / maxAccel
    self._accelDistance = maxVelocity * maxVelocity / (2 * math.fabs(maxAccel))
    self._decelDistance = maxVelocity * maxVelocity / (2 * math.fabs(maxDecel))
    self._coastDistance = self._distance - self._accelDistance - self._decelDistance
    self._accelTime = maxVelocity / maxAccel
    self._decelTime = maxVelocity / maxDecel
    self._coastTime = self._coastDistance / self._maxVelocity
    self._totalTime = self._accelTime + self._coastTime + self._decelTime
    self._interceptTime = math.inf
      
    if (self._coastTime < 0):
      self._interceptTime = self._accelTime + self._coastTime / 2
      self._maxVelocity = self._maxAccel * self._interceptTime
      self._accelDistance = self._maxVelocity * self._maxVelocity / (2 * math.fabs(self._maxAccel))
      self._decelDistance = self._maxVelocity * self._maxVelocity / (2 * math.fabs(self._maxDecel))
      self._coastDistance = 0.0
      self._accelTime = self._interceptTime
      self._decelTime = self._totalTime - self._interceptTime
      self._coastTime = 0.0
      self._totalTime = self._accelTime + self._coastTime + self._decelTime
    
  def getTotalTime(self) : 
    return self._totalTime
  
  def isTrapezoid(self) : 
    return math.isinf(self._interceptTime)

  def getDistance(self, time):
    if (0 < time and time < self._accelTime):
      return 0.5 * self._maxAccel * (time)**2
    
    elif (self._accelTime < time and time < self._accelTime + self._coastTime):
      return self._maxVelocity * (time - self._accelTime / 2)
    
    elif (self._accelTime + self._coastTime < time and time < self._totalTime):
      return self._maxVelocity * (time - self._accelTime / 2) - 0.5 * self._maxDecel * (time - self._accelTime - self._coastTime)**2
  
    else: 
      return self._accelDistance + self._decelDistance + self._coastDistance
  

  def getVelocity(self, time):
    if (0 < time and time < self._accelTime): 
      return self._maxAccel * time
      
    elif (self._accelTime < time and time < self._accelTime + self._coastTime):
      return self._maxVelocity
    
    elif (self._accelTime + self._coastTime < time and time < self._totalTime):
      return self._maxVelocity - self._maxDecel * (time - self._accelTime - self._coastTime);
    else:
      return 0.0; 
  

  def getAcceleration(self, time):
    if (0 < time and time < self._accelTime) :
      return self._maxAccel
    elif (self._accelTime < time and time < self._accelTime + self._coastTime):
      return 0.0
    elif (self._accelTime + self._coastTime < time and time < self._totalTime):
      return -self._maxDecel
    else:
      return 0.0
  
  
class PID:
  def __init__(self, kP, kI, kD):
    self._kP = kP
    self._kI = kI
    self._kD = kD

    self._P = 0
    self._I = 0
    self._D = 0
    
    self._P_last = 0
    self._I_last = 0
    self._D_last = 0


  def calc(self, error, deltaTime):
  
    self._P = error;
    self._I += (self._P_last + self._P) * deltaTime * 0.5;
    
    # _I Limit
    if (self._I_last > 0 and self._I < 0): 
      self._I = 0 
    
    self._D = (self._P - self._P_last) / deltaTime;

    self._P_last = self._P
    self._I_last = self._I
    self._D_last = self._D

    return self._P * self._kP + self._I * self._kI + self._D * self._kD
  
  def reset(self):
    self._P = 0
    self._I = 0
    self._D = 0

class PIDController:
  def __init__(self, linearPID: PID, angularPID: PID):
    self._linearPID = linearPID
    self._angularPID = angularPID
    self._turnLockDist = 8
  
  def calculate(self, Robot: robot, targetPos: Point, fps, direction = 1):  
      
    linearError = Robot.pos.magnitude(targetPos)
    linear = self._linearPID.calc(linearError, 1/fps) * direction
    
    if (direction == -1):
      angularError = Robot.pos.amplitude(targetPos * -1) - Robot.theta
    else:
      angularError = Robot.pos.amplitude(targetPos) - Robot.theta
      
      
    angular = self._angularPID.calc(angularError, 1/fps) 
    
    if (linearError < self._turnLockDist):
      angular = 0
      linear *= math.cos(angularError)
    else:       
      if (math.fabs(angularError) >  math.pi / 2):
        print(angularError)
        print("REVERSE")
        angularError = angularError - (angularError / math.fabs(angularError)) * math.pi
        linear *= -1
    
    return (linear, angular)

  def reset(self):
    self._angularPID.reset()
    self._linearPID.reset()
    
if __name__ == "__main__":
  targetPoint = Point(DESIRED_STATE[0], DESIRED_STATE[1])
  startPoint = Point(START_STATE[0], START_STATE[1])

  # Define Robot and Controller
  Robot = robot(startPoint, START_STATE[2], 0, 0, (ROBOT_PARAM_LINEAR[0], ROBOT_PARAM_ANGULAR[0]))
  RobotController = PIDController(PID(2, 0.05, 0), PID(2,0,0))
  
  # Initialize Pygame  
  pg.init()
  
  # Pygame Window and Clock
  Window = pg.display.set_mode(WINDOW_SIZE)
  Clock = pg.Clock()
  
  # Path of Previous Robot Locations
  Path = []
  TrajectoryPath = []
  
  # Start of Runtime
  runtime = True
  while runtime:
    Clock.tick(FPS) # Run at FPS
    Time = pg.time.get_ticks() / 1000
    # Clear Screen
    Window.fill(WINDOW_COLOR)
    
    # Controller Output
    Out = RobotController.calculate(Robot, targetPoint, FPS)
    
    Robot.setVelocities(Out[0], Out[1])
          
    Path.append(((Robot.pos.y) * WINDOW_SIZE[0] / WINDOW_X_INCHES,(WINDOW_Y_INCHES - Robot.pos.x)*WINDOW_SIZE[1]/WINDOW_Y_INCHES))
    
    # Update Robot Position
    Robot.moveStep(FPS)
    
    # Event Loop
    events = pg.event.get()
    for event in events:
      if event.type == pg.QUIT:
        runtime = False
      if event.type == pg.MOUSEBUTTONDOWN:
        MousePos = pg.mouse.get_pos()
        DESIRED_STATE = (MousePos[0] *  WINDOW_Y_INCHES / WINDOW_SIZE[0], (WINDOW_SIZE[1] - MousePos[1]) * WINDOW_X_INCHES / WINDOW_SIZE[1])
        targetPoint = Point(DESIRED_STATE[1], DESIRED_STATE[0])
        RobotController.reset()
      
        
    # Draw Path
    for index, point in enumerate(Path):
      pg.draw.circle(Window, PATH_NODE_COLOR.lerp(DECAY_NODE_COLOR, 1 - index / len(Path)), point, NODE_RADIUS)
        
    for point in TrajectoryPath:
      pg.draw.circle(Window, TRAJECTORY_NODE_COLOR, point, NODE_RADIUS)
    
    # Draw Start Position
    pg.draw.circle(Window, START_COLOR, (START_STATE[0]  * WINDOW_SIZE[0] / WINDOW_X_INCHES, 
                                        (WINDOW_Y_INCHES - START_STATE[1]) * WINDOW_SIZE[1] / WINDOW_Y_INCHES), DESIRED_RADIUS)
    # Draw Desired Position
    pg.draw.circle(Window, DESIRED_COLOR, (DESIRED_STATE[0]  * WINDOW_SIZE[0] / WINDOW_X_INCHES, 
                                        (WINDOW_Y_INCHES - DESIRED_STATE[1]) * WINDOW_SIZE[1] / WINDOW_Y_INCHES), DESIRED_RADIUS)
    
    # Update Screen
    pg.display.update()
      
  pg.quit()