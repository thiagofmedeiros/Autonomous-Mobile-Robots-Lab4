"""lab2_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

WHEEL_DIST = 1.05
WHEEL_DIAMETER = 1.6
MAX_PHI = 3.2
MAX_SIMULATION_TIME = 30 * 1000
MAX_MEASURED_DISTANCE = 1.27
ACCEPTED_ERROR = 0.0001
K = 10

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
frontDistanceSensor = robot.getDevice('front_ds')
leftDistanceSensor = robot.getDevice('left_ds')
rightDistanceSensor = robot.getDevice('right_ds')
frontDistanceSensor.enable(timestep)
leftDistanceSensor.enable(timestep)
rightDistanceSensor.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# get handler to motors and set target position to infinity
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

imu = robot.getDevice('inertial unit')
imu.enable(timestep)


def inchesToMeters(x):
    return x / 39.37


def metersToInches(x):
    return x * 39.37


def getYawDegrees():
    Yaw = math.degrees(getYawRadians())

    if Yaw < 0:
        Yaw = Yaw + 360

    return Yaw


def getYawRadians():
    return imu.getRollPitchYaw()[2]


def getSensors():
    fdsVal = metersToInches(frontDistanceSensor.getValue())
    ldsVal = metersToInches(leftDistanceSensor.getValue())
    rdsVal = metersToInches(rightDistanceSensor.getValue())

    print("FrontDist {0:.3f} inches".format(fdsVal))
    print("LeftDist {0:.3f} inches".format(ldsVal))
    print("RightDist {0:.3f} inches\n".format(rdsVal))

    return fdsVal, ldsVal, rdsVal


def setSpeedsRPS(rpsLeft, rpsRight):
    if rpsLeft > MAX_PHI:
        print("Saturating left speed to {0:.3f} rad/s".format(MAX_PHI))
        leftMotor.setVelocity(MAX_PHI)
    elif rpsLeft < -MAX_PHI:
        print("Saturating left speed to {0:.3f} rad/s".format(-MAX_PHI))
        leftMotor.setVelocity(-MAX_PHI)
    else:
        print("Left motor velocity: {0:.3f} rad/s".format(rpsLeft))
        leftMotor.setVelocity(rpsLeft)

    if rpsRight > MAX_PHI:
        print("Saturating right speed to {0:.3f} rad/s".format(MAX_PHI))
        rightMotor.setVelocity(MAX_PHI)
    elif rpsRight < -MAX_PHI:
        print("Saturating right speed to {0:.3f} rad/s".format(-MAX_PHI))
        rightMotor.setVelocity(-MAX_PHI)
    else:
        print("Right motor velocity: {0:.3f} rad/s\n".format(rpsRight))
        rightMotor.setVelocity(rpsRight)


def rotateUntilObject(find, clockwise):
    global time

    recognized_object_array = camera.getRecognitionObjects()

    if clockwise:
        setSpeedsRPS(MAX_PHI, -MAX_PHI)
    else:
        setSpeedsRPS(-MAX_PHI, MAX_PHI)

    if find:
        while len(recognized_object_array) < 1:
            robot.step(timestep)
            time += timestep

            recognized_object_array = camera.getRecognitionObjects()
    else:
        while len(recognized_object_array) > 0:
            robot.step(timestep)
            time += timestep

            recognized_object_array = camera.getRecognitionObjects()


def getObjectDirection():
    recognized_object_array = camera.getRecognitionObjects()

    # If There is no object rotates until finds it
    # then rotates until it is no longer on view
    if len(recognized_object_array) < 1:
        rotateUntilObject(find=True, clockwise=True)
        firstYaw = getYawRadians()

        rotateUntilObject(find=False, clockwise=True)
        secondYaw = getYawRadians()
    # If There is an object rotates until it is no longer on view
    # Rotates in the opposite direction until finds it again
    # than rotates until it is no longer on view again
    else:
        rotateUntilObject(find=False, clockwise=True)
        firstYaw = getYawRadians()

        rotateUntilObject(find=True, clockwise=False)

        rotateUntilObject(find=False, clockwise=False)
        secondYaw = getYawRadians()

    # The direction of the object is the average between both angles
    direction = (firstYaw + secondYaw) / 2

    return direction


def correctDirection(desiredDirection):
    global time
    error = getYawRadians() - desiredDirection

    print("\nCorrecting Direction\n")

    while abs(error) > ACCEPTED_ERROR:
        speed = K * error

        setSpeedsRPS(speed, -speed)
        robot.step(timestep)
        time += timestep

        error = getYawRadians() - desiredDirection

    print("\nDirection Corrected\n")


def move(distance):
    global time

    lastCorrectionTime = MAX_SIMULATION_TIME

    error = 1

    while abs(error) > ACCEPTED_ERROR and time < MAX_SIMULATION_TIME:

        recognized_object_array = camera.getRecognitionObjects()

        # If There is no object rotates until finds it
        # Also reorients each 10 seconds
        if len(recognized_object_array) < 1 or abs(time - lastCorrectionTime) > 10000:
            desiredYaw = getObjectDirection()

            if abs(getYawRadians() - desiredYaw) > ACCEPTED_ERROR:
                correctDirection(desiredYaw)

                lastCorrectionTime = time

        recognized_object = camera.getRecognitionObjects()[0]
        position = camera.getRecognitionObjects()[0].get_position()
        orientation = camera.getRecognitionObjects()[0].get_orientation()

        frontSensor, leftSensor, rightSensor = getSensors()
        error = frontSensor - distance

        if abs(error) > ACCEPTED_ERROR:
            speed = K * error

            setSpeedsRPS(speed, speed)

            robot.step(timestep)
            time += timestep
            print("Time {0:.3f} seconds\n".format(time / 1000))


DESIRED_DISTANCE = 5

time = 0

clockwise = True

robot.step(timestep)
time += timestep

move(DESIRED_DISTANCE)

setSpeedsRPS(0, 0)
print("\nSimulation Stopped\n")

# Enter here exit cleanup code.
