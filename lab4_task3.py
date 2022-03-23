"""lab2_task2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

WHEEL_DIST = 1.05
WHEEL_DIAMETER = 1.6
MAX_PHI = 3
MAX_SIMULATION_TIME = 3 * 60 * 1000
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

    return fdsVal, ldsVal, rdsVal


def setSpeedsRPS(rpsLeft, rpsRight):
    if rpsLeft > MAX_PHI:
        leftMotor.setVelocity(MAX_PHI)
    elif rpsLeft < -MAX_PHI:
        leftMotor.setVelocity(-MAX_PHI)
    else:
        leftMotor.setVelocity(rpsLeft)

    if rpsRight > MAX_PHI:
        rightMotor.setVelocity(MAX_PHI)
    elif rpsRight < -MAX_PHI:
        rightMotor.setVelocity(-MAX_PHI)
    else:
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


def convertTo2pi(radians):
    if radians < 0:
        radians += math.pi * 2

    return radians


def convertToPiPi(radians):
    if radians < -math.pi:
        radians += math.pi * 2

    if radians > math.pi:
        radians -= math.pi * 2

    return radians


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

    firstYaw = convertTo2pi(firstYaw)
    secondYaw = convertTo2pi(secondYaw)
    # The direction of the object is the average between both angles
    direction = (firstYaw + secondYaw) / 2

    direction = convertToPiPi(direction)

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


def turn90degrees(direction):
    global time

    yaw = getYawDegrees()

    if direction:
        yaw -= 90
    else:
        yaw += 90

    if yaw > 360:
        yaw -= 360

    correctDirection(convertToPiPi(math.radians(yaw)))


def correctToObjectDirection():
    desiredYaw = getObjectDirection()

    if abs(getYawRadians() - desiredYaw) > ACCEPTED_ERROR:
        correctDirection(desiredYaw)


def executeTurn():
    global time

    for i in range(60):
        robot.step(timestep)
        time += timestep
    turn90degrees(not clockwise)


def move(desired_distance):
    global time

    reached = False
    lastCorrectionTime = MAX_SIMULATION_TIME
    unobstructed = True
    cameraDistance = 100
    turns = 0

    correctToObjectDirection()

    while not reached and time < MAX_SIMULATION_TIME:
        frontSensor, leftSensor, rightSensor = getSensors()

        recognized_object_array = camera.getRecognitionObjects()

        if len(recognized_object_array) > 0:
            if unobstructed and abs(time - lastCorrectionTime) > 5000:
                correctToObjectDirection()
                frontSensor, leftSensor, rightSensor = getSensors()

                lastCorrectionTime = time
            recognized_object = camera.getRecognitionObjects()[0]
            cameraDistanceMeters = recognized_object.get_position()[0]
            cameraDistance = metersToInches(cameraDistanceMeters)
            error = cameraDistance - desired_distance
        else:
            print("Lost sight of goal")
            error = frontSensor - (MIN_DISTANCE_WALL - 0.1)

        if cameraDistance < 24:
            if abs(frontSensor - desired_distance) < ACCEPTED_ERROR:
                reached = True
                return

            error = frontSensor - desired_distance

        if frontSensor < MIN_DISTANCE_WALL:
            print("Obstructed")
            unobstructed = False
            turn90degrees(clockwise)
            turns += 1
            frontSensor, leftSensor, rightSensor = getSensors()

        if not unobstructed:
            if clockwise:
                if leftSensor > MIN_DISTANCE_WALL * 3:
                    executeTurn()
                    frontSensor, leftSensor, rightSensor = getSensors()
                    turns -= 1
                    if turns == 0:
                        unobstructed = True
            else:
                if rightSensor > MIN_DISTANCE_WALL * 3:
                    executeTurn()
                    frontSensor, leftSensor, rightSensor = getSensors()
                    turns -= 1
                    if turns == 0:
                        unobstructed = True

        if not reached:
            speed = K * error

            setSpeedsRPS(speed, speed)

            robot.step(timestep)
            time += timestep
            print("Time {0:.3f} seconds\n".format(time / 1000))


DESIRED_DISTANCE = 5
MIN_DISTANCE_WALL = 2

time = 0

clockwise = False

robot.step(timestep)
time += timestep

move(DESIRED_DISTANCE)

setSpeedsRPS(0, 0)
print("\nSimulation Stopped\n")

# Enter here exit cleanup code.
