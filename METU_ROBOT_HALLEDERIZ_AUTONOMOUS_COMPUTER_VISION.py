import asyncio
import mavsdk.telemetry
from mavsdk import System
import numpy as np
import cv2, math,time

import sys
from threading import Thread
from mavsdk.offboard import VelocityBodyYawspeed
from mavsdk.offboard import (OffboardError, PositionNedYaw,VelocityNedYaw)
from mavsdk.telemetry import FlightMode


cam=cv2.VideoCapture(0)
########################## Arrow Detection ###############################
arrow_center_stop = False
tip_point_global_tuple = []
tip_point_global = list(tip_point_global_tuple)
bottom_point_global_tuple = []
vx_arrow,vy_arrow,yaw_arrow=None,None,None
counter_arrow=0
######################## Letter Detection ##############################
letter_type = "L"
letter_temp=None
center_flag = True
letter_centered = True
again_letter_flag = False
x_counter=0
l_counter=0
t_counter = 0
vx_letter,vy_letter,vz_letter,yaw_letter=None,None,None,None
############################ Line Detection ########################
vx_line,vy_line,vz_line,yaw_line=None,None,None,None
line_yaw=0
start_eighty,start_onehundredsixty=0,0
lead_point_global,slope_point_global = [],[]
#-----------------------------------------------------------------

#vx,vy,vz,v_yaw,t = None,None,None,0,1

############################# GLOBAL VARIABLE DECLARATIONS

took_off = False
factor = 600
frameWidth = 640
frameHeight = 480
counter=0
counter_arrow=0
check=False
vehicle_mode=None
altitude_play = None
hold_mode = None

def angle_finder(point1, point2, width_of_image=640, height_of_image=480):
    x1 = point1[0] - width_of_image // 2
    y1 = point1[1] - height_of_image // 2
    x2 = point2[0] - width_of_image // 2
    y2 = point2[1] - height_of_image // 2

    slope = ((x2 - x1) / (y2 - y1))
    result = math.degrees(math.atan(slope))

    if y1 > y2:
        if x1 > x2:
            # print("one")
            return -result

        else:
            # print("two")
            return -result

    else:
        if x1 < x2:
            # print("three")
            return 180 - result

        else:
            # print("four")
            return -(result + 180)

def find_arrow_points(points, convex_hull):
    global tip_point_global_tuple, bottom_point_global_tuple
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)

    for i in range(2):
        j = indices[i] + 2
        if j > length - 1:
            j = length - j
        if np.all(points[j] == points[indices[i - 1] - 2]):
            bottom_indice = int((indices[0] + indices[1]) / 2)
            tip_point_global_tuple = tuple(points[j])
            bottom_point_global_tuple = tuple(points[bottom_indice])

def arrow_yaw_ardupilot(colored_image, canny_image, contour_area=2000):
    global vehicle_mode,tip_point_global,arrow_tip,letter_type,again_letter_flag,vx_arrow,vy_arrow,yaw_arrow
    forward_counter = 0

    cv2.putText(colored_image, "Arrow Mode", (0, 180), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255),2)
    contours, hierarchy = cv2.findContours(canny_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if contours != ():
        total_area = 0
        for cnt in contours:
            contour = cv2.contourArea(cnt)
            total_area += contour
        if total_area >= int(len(contours))*contour_area/5:
            for cnt in contours:
                if cv2.contourArea(cnt) > contour_area:
                    peri = cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
                    hull = cv2.convexHull(approx, returnPoints=False)
                    x, y, w, h = cv2.boundingRect(approx)
                    global check
                    check = False
                    global counter_arrow,arrow_center_stop
                    arrow_center_stop = False
                    cx = int(x + (w / 2))
                    cy = int(y + (h / 2))
                    vx = cx - (frameWidth / 2)
                    vy = (frameHeight / 2) - cy
                    x1_global, y1_global = x + w // 2, y + h // 2
                    sides = len(hull)
                    if sides != 0:
                        if 6 > sides > 3 and sides + 2 == len(approx):
                            points_variable = approx[:, 0, :]
                            hull_variable = hull.squeeze()
                            find_arrow_points(points_variable, hull_variable)
                            arrow_tip = list(tip_point_global_tuple)
                            cv2.drawContours(colored_image, cnt, -1, (0, 255, 0), 3)

                            if arrow_tip != []:
                                vehicle_mode = "Arrow"
                                cv2.circle(colored_image, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
                                # client.hoverAsync()
                                result = angle_finder([x1_global, y1_global], arrow_tip)
                                cv2.putText(colored_image, str(result), arrow_tip, cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 0),2)
                                cv2.circle(colored_image, (x + w // 2, y + h // 2), 3, (0, 255, 255), 3)
                                if not arrow_center_stop:
                                    if not check:
                                        if 45 >= vx >= -45 and vy <= 45 and vy >= -45:
                                            counter_arrow += 1
                                            if counter_arrow >= 50:
                                                check = True
                                                arrow_center_stop = True
                                                vx_arrow=1
                                                vy_arrow=0
                                                yaw_arrow=result
                                                again_letter_flag = True
                                                counter_arrow = 0
                                                arrow_tip.clear()
                                                vehicle_mode=None

                                        else:
                                            if vy >= 0 and vx <= 0 or vy <= 0 and vx >= 0:
                                                vx_arrow=-vx
                                                vy_arrow=-vy
                                                yaw_arrow = result
                                                print("Arrow",vx_arrow," ",vy_arrow)


                                            else:
                                                vx_arrow=vx
                                                vy_arrow=vy
                                                yaw_arrow = result
                                                print("Arrow", vx_arrow, " ", vy_arrow)

    #     else:
    #         print("ileri git = düşük alan")
    #         vx_arrow=0.2
    #         vy_arrow=0
    #
    # else:
    #     print("ileri git = kontür yok")
    #     vx_arrow = 0.2
    #     vy_arrow = 0

def detect_letter(img_canny,img_copy):
    global letter_type,center_flag,letter_centered,x_counter,l_counter,t_counter,vehicle_mode,letter_temp,vx_letter,vy_letter,vz_letter,yaw_letter,altitude_play,hold_mode

    cv2.putText(img_copy, "Letter Mode", (0, 120), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255),
                2)
    contour_position = None
    contours ,hierarchy =cv2.findContours(img_canny ,cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        length = int(len(contours))
        area =cv2.contourArea(cnt)
        if area >5500:
            for i in range(length):
                if hierarchy[0][i][2] == -1 and hierarchy[0][i][3] != -1 :
                    contour_position = i
                else:
                    pass
            peri = cv2.arcLength(contours[contour_position], True)
            approx = cv2.approxPolyDP(contours[contour_position], 0.02 * peri, True)
            objCor = len(approx)
            x, y, w, h = cv2.boundingRect(approx)
            global check
            global counter

            if center_flag:
                check = False
                cx = int(x + (w / 2))
                cy = int(y + (h / 2))
                vx = cx - (frameWidth / 2)
                vy = (frameHeight / 2) - cy
                # await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
                # await asyncio.sleep(1)
                vx_letter, vy_letter, vz_letter = 0, 0, 0
                if not check:
                    if 45 >= vx >= -45 and vy <= 45 and vy >= -45:
                        counter += 1
                        if counter >= 50:
                            print(counter)
                            check = True
                            center_flag = False
                            counter = 0
                            #hold_mode = True
                            altitude_play = True
                            letter_centered = True
                            letter_type = "X"
                            vx_letter, vy_letter, vz_letter, yaw_letter = 0, 0, 1, 0
                            print(vx_letter, vy_letter, vz_letter)

                    else:
                        letter_centered = False
                        if vy >= 0 and vx <= 0 or vy <= 0 and vx >= 0:
                            vx_letter = -vx
                            vy_letter = -vy
                            yaw_letter = 0
                            print("daha ortalamadım 1")
                        else:
                            vx_letter = vx
                            vy_letter = vy
                            yaw_letter=0
                            print("daha ortalamadım 2")

                if objCor == 6:
                    l_counter += 1
                    # vehicle_mode=None
                    letter_type = None
                    letter_temp = "L"
                    if l_counter >= 15:
                        # client.moveToZAsync(-3.5, 1).join()
                        # client.rotateToYawAsync(45)
                        if letter_centered:
                            letter_type = "L"
                            l_counter = 0
                            letter_centered = False
                            center_flag = True
                            counter = 0
                            vehicle_mode = "Line"

                            break
                            # print(letter_type)
                elif objCor == 12:
                    x_counter += 1
                    letter_type = None
                    letter_temp = "X"
                    if x_counter >= 15:
                        if letter_centered:
                            letter_type = "X"
                            x_counter = 0
                            letter_centered = False
                            center_flag = True
                            counter = 0
                            vehicle_mode = "Arrow"
                            break
                            # print(letter_type)
                elif objCor == 8 and int(peri / w) > 3:
                    t_counter += 1
                    letter_type = None
                    if t_counter >= 15:
                        letter_type = "T"
                        t_counter = 0
                        letter_centered = False
                        center_flag = True
                        counter = 0
                        break

                        # print(letter_type)
                if letter_temp is not None:
                    cv2.drawContours(img_copy, contours[contour_position], -1, (255, 0, 0), 3)
                    cv2.rectangle(img_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(img_copy, letter_temp,
                                (x + (w // 2) - 10, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                                (0, 256, 256), 2)

def line_picker_w_yaw(colored_image, thresholded, lead_height, slope_height, screen_width=640,
                      radius=2, color=(0, 0, 255), thickness=cv2.FILLED, contour_area=500):
    cv2.putText(colored_image, "Line Mode", (0, 60), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 255),
                2)
    global line_yaw, slope_point_global, lead_point_global, start_eighty,vehicle_mode,vx_line,vy_line,vz_line,yaw_line,start_onehundredsixty
    black_canvas = cv2.imread("black_canvas_640480.png")
    contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    solid_x = screen_width // 2
    solid_y = [lead_height, slope_height]

    cv2.circle(colored_image, (solid_x, solid_y[0]), radius, color, thickness)
    cv2.circle(colored_image, (solid_x, solid_y[1]), radius, color, thickness)


    if contours is not None:
        for cnt in contours:
            if cv2.contourArea(cnt) > contour_area:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                objCor = len(approx)

                if objCor < 6:
                    vehicle_mode = "Line"
                    cv2.drawContours(colored_image, cnt, -1, (0, 255, 0), 2)
                    cv2.drawContours(black_canvas, cnt, -1, (255, 255, 255), 2)
                    thresholded=np.array(thresholded)
                    rows, columns= thresholded.shape[:2]
                    start_eighty, start_onehundredsixty, angle = 0, 0, 0
                    for i in range(columns):
                        if np.any(black_canvas[lead_height, i] == 255):
                            start_eighty = i
                            lead_point_global = [start_eighty, lead_height]
                            cv2.circle(colored_image, (start_eighty, lead_height), radius, (255, 0, 0), thickness)
                            break

                    for j in range(columns):
                        if np.any(black_canvas[slope_height, j] == 255):
                            start_onehundredsixty = j
                            cv2.circle(colored_image, (start_onehundredsixty, slope_height), radius, (255, 0, 0),
                                       thickness)
                            line_yaw = angle_finder((start_onehundredsixty, slope_height), (start_eighty, lead_height))
                            cv2.putText(colored_image, f"yaw:{line_yaw}", (60, 60), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0),
                                      1)
                            break

                    cx = start_eighty
                    cy = 80
                    vx = cx - (frameWidth / 2)
                    vy = (frameHeight / 2) - cy

                    if vx and vy and line_yaw !=None:
                        vx_line=vx
                        vy_line=vy
                        yaw_line=line_yaw
                        print(vx_line,vy_line,yaw_line)

                else:
                    vehicle_mode=None
                    break

def display(img):
    pt1 = (275,195)
    pt2 = (365,285)
    cv2.line(img,(int(frameWidth/2),0),(int(frameWidth/2),frameHeight),(0,0,255),2)
    cv2.line(img, (0, int(frameHeight / 2) ), (frameWidth, int(frameHeight / 2) ), (0,0,255), 2)
    cv2.rectangle(img,pt1,pt2,(51,255,51),2)

async def cam():
    global took_off


    while True:
        if took_off:

            global vehicle_mode, letter_type

            _,img=cam.read()
            imgContour = img.copy()
            imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 1)
            imgCanny = cv2.Canny(imgBlur, 50, 250)
            kernel = np.ones((5, 5), np.uint8)
            dilated = cv2.dilate(imgCanny, kernel, iterations=1)
            _, thresholded = cv2.threshold(imgBlur, 40, 255, cv2.THRESH_BINARY_INV)

            if vehicle_mode == None:
                detect_letter(imgCanny, imgContour)
            if letter_type == "L":
                line_picker_w_yaw(imgContour, thresholded, 225, 250)
            elif letter_type == "X":
                arrow_yaw_ardupilot(imgContour, dilated)
            elif letter_type == "T":
                pass
            display(imgContour)
            cv2.imshow("contour", imgContour)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



async def vektor_verici():
    factorx=2300
    factory=1000
    factorx_center=1250
    factory_center=600

    while True:
        global took_off,altitude_play,hold_mode,vx_letter,vx_arrow,vx_line,vy_letter,vy_line,vy_arrow,vz_letter,vz_line,yaw_line,yaw_letter,yaw_arrow
        if took_off == False:
            drone = System()
            await drone.connect(system_address="serial:///dev/ttyACM0:57600")

            print("Arming")
            await drone.action.arm()
            await asyncio.sleep(5)

            await drone.action.set_takeoff_altitude(1.5)
            print("Taking off")
            await drone.action.takeoff()
            await asyncio.sleep(10)


            print("Setting initial setpoint")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))

            try:
                await drone.offboard.start()
                print("Offboard Başladı")

            except mavsdk.offboard.OffboardError as error:
                print(f"Starting offboard mode failed with error code: {error._result.result}")
                print("Disarming")

                await drone.action.disarm()
                return

            if await drone.offboard.is_active():
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.8,0,0,0))
                await asyncio.sleep(1)
                took_off = True
                print("Kalkış Başarılı")

        else:
            if vehicle_mode == None:
                if vx_letter and vy_letter !=None:
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx_letter /factorx_center, vy_letter / factory_center, vz_letter, 0))
                    await asyncio.sleep(0.2)

                    vx_letter=None
                    vy_letter=None

            #elif vehicle_mode=="Arrow":
            if vx_arrow and vy_arrow !=None:
                print("Arrow Değeler:",vx_arrow," ",vy_arrow," ",yaw_arrow,)
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx_arrow / factorx_center, vy_arrow / factory_center, 0, yaw_arrow/10))
                await asyncio.sleep(0.2)
                print("Arrowcu:", vx_line, " ", vy_line, " ", yaw_line)
                vx_arrow=None
                vy_arrow=None
                yaw_arrow=None

            #elif vehicle_mode == "Line":
            if vx_line and vy_line and yaw_line != None:
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-(vx_line / factorx), -(vy_line / factory), 0, yaw_line/5.75))
                await asyncio.sleep(0.2)
                print("Vektörcü:", vx_line, " ", vy_line, " ", yaw_line)
                vx_line=None
                vy_line=None
                yaw_line=None

            if altitude_play == True:

                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, -0.2, 0))
                await asyncio.sleep(4)
                #await drone.telemetry.flight_mode(FlightMode(3))
                print("Hold")
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
                await asyncio.sleep(2)
                print("Letter Yükselme")
                altitude_play = False


Thread(target=asyncio.run,args=(vektor_verici(),)).start()
Thread(target=asyncio.run,args=(cam(),)).start()
