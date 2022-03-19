import asyncio,time,mavsdk.offboard,math
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw,VelocityNedYaw)

# point_rotator function is created for points in a rotated cartesian system. Theta is degree between North and forward direction of second cartesion system. Takes three parameters for calculation (north,east,theta)
# and a rotation value which returns north and east values in NED configuration.
def point_rotater(n,e,theta):
    radians = math.radians(theta)
    new_n = n*math.cos(radians) + e*math.sin(radians)
    new_e = e*math.cos(radians) - n*math.sin(radians)
    return [new_n,new_e]

# mission_list_maker function is created for waypoint lists in a rotated cartesian system. Theta is degree between North and forward of second cartesian system. Takes two parameters for calculation (list,theta)
# which returns a list that has waypoints in NED configuration.
def mission_list_maker(waypoints_list, theta):
    mission_list = []
    for k in range(len(waypoints_list)):
        nw,ew,zw = waypoints_list[k]
        smth = point_rotater(nw,ew,theta)
        item = [smth[0],smth[1],waypoints_list[k][2]]
        mission_list.append(item)
    return mission_list

# angle_finder function is created for calculation of heading value in PX4 autopilot system. Takes two points as input and returns yaw degree of drone which will cause drone to face resulted direction.
async def angle_finder(point1, point2):
    n1 = point1[0]
    e1 = point1[1]
    n2 = point2[0]
    e2 = point2[1]
    slope = math.fabs((e2 - e1) / (n2 - n1))
    alpha = math.degrees(math.atan(slope))
    if n2 > n1:
        if e2 > e1:
            return alpha
        else:
            return -alpha
    else:
        if e1 < e2:
            return 180 - alpha
        else:
            return -180 + alpha

# line_createor function is used for creating lists that will provide neccessary information of line between any two points of triangle. This function could be used for any kind of shape because there is no restriction about
#  positions. Takes two points as start of line and the end, takes an interest point so it can create neccessary yaw values too and passes them to PositionNedYaw. line_creator creates 60 points in a line that drone
# will move to. Points are created by seperation of the line in 60 pieces. After each point calculation heading value is calculated by passing position values. Finally item is created and passed to a list.
# List is being appended so queue of positions are secured.
async def line_creator(P1,P2,Pin):
    n1, e1 = P1[0], P1[1]
    n2, e2 = P2[0], P2[1]
    nin, ein = Pin[0], Pin[1]
    random_list = []
    for i in range(0, 60):
        rate_e = (e2 - e1) / 60
        rate_n = (n2 - n1) / 60
        pos_n = n1 + rate_n * i
        pos_e = e1 + rate_e * i
        yaw = await angle_finder((pos_n, pos_e), (nin, ein))
        item = [pos_n, pos_e, yaw]
        random_list.append(item)

    return random_list

# main async function that will run until ends.
async def run():
    #-------------- Variable Declarations --------------------*
    #------------Variables for all missions -----------------*
    rotation_degree = 40
    # ------------ First Mission Variables ------------ *
    waypoints_list = [[0, 0, -2.5],  # 0
                      [0, 0, -7.5],  # 1
                      [0, 5, -7.5],  # 2
                      [5, 5, -7.5],  # 3
                      [5, 5, -2.5],  # 4
                      [0, 5, -2.5],  # 5
                      [0, 0, -2.5],  # 6
                      [5, 0, -2.5],  # 7
                      [5, 0, -7.5],  # 8
                      [0, 0, -7.5],  # 9
                      [0, 5, -7.5],  # 10
                      [0, 5, -2.5],  # 11
                      [5, 5, -2.5],  # 12
                      [5, 0, -2.5],  # 13
                      [5, 0, -7.5],  # 14
                      [5, 5, -7.5]  # 15
                      ] # Coordinates of corners of 5x5x5 cube for mission waypoints in NE (NE = North,East)
    mission_list = []                                                   # It will be list used for first mission, declared empty.
    mission_list = mission_list_maker(waypoints_list,rotation_degree)   # Filling mission_list with waypoint datas.
    tbc = 3                                                             # Time until next command exexutes

    #------------ Second Mission Variables ------------*
    mis_alt_D = -2.5                                                    # Target mission altitude in NE
    Tri1 = [0, 0]                                                    # First point of triangle in NE
    Tri2 = [4.33, 2.5]                                                   # Second point of triangle in NE
    Tri3 = [0, 5]                                                     # Third point of triangle in NE
    Cen_Mass = [1.44, 2.5]                                               # Interest point of triangle (center of mass) in NE
    tbp = 0.095                                                           # Time interval until next command executes

    P1 = point_rotater(Tri1[0], Tri1[1], rotation_degree)               # Calculating value of first point of triangle with rotation value of cartesian system.
    P2 = point_rotater(Tri2[0], Tri2[1], rotation_degree)               # Calculating value of second point of triangle with rotation value of cartesian system.
    P3 = point_rotater(Tri3[0], Tri3[1], rotation_degree)               # Calculating value of third point of triangle with rotation value of cartesian system.
    Pin = point_rotater(Cen_Mass[0], Cen_Mass[1], rotation_degree)      # Calculating value of interest point of triangle with rotation value of cartesian system.

    first_list = await line_creator(P1,P2,Pin)                          # List of first line of triangle
    second_list = await line_creator(P2, P3, Pin)                       # List of second line of triangle
    third_list = await line_creator(P3, P1, Pin)                        # List of third line of triangle

    print(f"first starts at {first_list[0]} and ends at {first_list[len(first_list)-1]}")
    print(f"second starts at {second_list[0]} and ends at {second_list[len(second_list)-1]}")
    print(f"first starts at {third_list[0]} and ends at {third_list[len(first_list) - 1]}")

    print("Connecting...")                                              # Connecting to autopilot with proper port and ID
    drone = System()
    await drone.connect(system_address="udp://:14540")                  # sitl: "udp://:14540"       irl: "/dev/ttyACM0"

    print("-- Arming")                                                  # Arming motors
    await drone.action.arm()
    await asyncio.sleep(4)

    print("TAking off")                                                 # Takeoff before our missions start
    await drone.action.takeoff()
    await asyncio.sleep(6)

    print("-- Setting initial setpoint")                                # Setting initial point for PositionNedYaw ( Selecting takeoff position as (0,0,0) in NED)
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))

    print("-- Starting offboard")                                       # Starting Offboard mode of PX4 and checking if there is an issue. If there is, print it out.
    try:
        await drone.offboard.start()
    except mavsdk.offboard.OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    for i in range(0, 16):                                              # This for loop sends commands for making drone move to positions according mission_list data thus
        n = mission_list[i][0]                                          # drone is finishing first mission.
        e = mission_list[i][1]
        d = mission_list[i][2]
        await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, float("NaN")))
        await asyncio.sleep(tbc)
        print(f"Gone to waypoint[{i}]")

    print("First Mission Is Succesfull. Continuing With Second Mission") # Giving a four seconds of break until second mission starts.
    print("Mission starts in:")
    for j in range(4):
        print(4 - j)
        await asyncio.sleep(1)
    print("Mission_started")

    await drone.offboard.set_position_ned(PositionNedYaw(first_list[0][0], first_list[0][1], mis_alt_D, first_list[0][2]))          # Going to first point of triangle to start second mission there so it draws edges of
    await asyncio.sleep(2)                                                                                                          # first line and heading of drone stays at interest point.
                                                                                                                                    # Average time of one loop of triangle is calculated from SITL-Airsim experiments and
    for tours in range(14):                                                                                                         # loop count is sat according to that data. System is looped so drone makes 13 full turns.
        print("first line")
        for a in range(len(first_list)):
            await drone.offboard.set_position_ned(PositionNedYaw(first_list[a][0], first_list[a][1], mis_alt_D, first_list[a][2]))
            await asyncio.sleep(tbp)

        print("second line")
        for b in range(len(second_list)):
            await drone.offboard.set_position_ned(PositionNedYaw(second_list[b][0], second_list[b][1], mis_alt_D, second_list[b][2]))
            await asyncio.sleep(tbp)

        print("third line")
        for c in range(len(third_list)):
            await drone.offboard.set_position_ned(PositionNedYaw(third_list[c][0], third_list[c][1], mis_alt_D, third_list[c][2]))
            await asyncio.sleep(tbp)

        print(f"Tour {tours+1} is done")                                                        # Taking output of how much turns made.

    print(f"{tours + 1} full tours taken. Finishing mission")                                       # Confirming output that second mission is finished.

    print("-- Stopping offboard")                                                               # Stopping offboard and checking if there would be an error. If so printing it out.
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("returning to launch")                                                                # Finally taking mode to RTL and finishing whole code.
    await drone.action.return_to_launch()
    await asyncio.sleep(10)
    print("Mission Is Successfull.Bravo!")

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(run())                                          # Running run() loop until it ends successfully. Mission code is executed by this call.