# coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from math import radians
import cv2
from robot import Robot
from img_processing import findObstacle,findGoal
import time
##linear reminder
##Red : front       linear.x
##Green:l or r      linear.y
##Z no use

## angular reminder
## Red : roll       angular.x
## Green : pitch    angular.y
## Blue : yaw       angular.z

# the dimention of picture
img_x = 0
img_y = 0
# region of interst of where the center point of goal should be 
# left_roi = 3/7*img_x
# right_roi = 4/7*img_x

goal_roi_left = 0
goal_roi_right = 0

# region of interst of where the obstacles should be out of 
obstacle_roi_left = 0
obstacle_roi_right = 0


twist = None
# define how large every step goes
step_distance = 1
# used to replace the .rest functions
sleep_time = 0.5
last_rotate_left = True

# define if a obstacle is still too far to avoid
def far_obstacle(obstacle):
    if obstacle[3] < img_y/2 :
        return True

# detect and return obstacles if avoidence require
def obstacle_detect(include_far):
    obstacles = findObstacle(robot.get_image())
    print(len(obstacles),"obstacles detected")
    avoid_obstacles = []
    avoid = False
    for obstacle in obstacles:
        if far_obstacle(obstacle) and include_far is False:
            print("far obstacle")
        elif obstacle[0] < img_x/2 and obstacle[0]+obstacle[2] < img_x/4:
            print("out of left RIO line")
        elif obstacle[0] > obstacle_roi_right:
            print("out of right RIO line")
        else:
            print("avoidence required")
            avoid_obstacles.append(obstacle)
            avoid = True
    if avoid:
        return avoid_obstacles
    return None
# initial the global variables
def init():
    # Get image and save image
    image=robot.get_image()

    cv2.imwrite("image.jpg",image)
    # Get compressed image and save iamge
    comImage=robot.get_comImage()
    cv2.imwrite("comImage.jpg",comImage)

    #get frame length and height
    global img_x,img_y
    img_y, img_x, _ = image.shape
    global goal_roi_left,goal_roi_right,obstacle_roi_left,obstacle_roi_right
    goal_roi_left = int(float(2)/float(5) * img_x)
    goal_roi_right = int(float(3)/float(5) * img_x)

    obstacle_roi_left = int(float(1)/float(4) * img_x)
    obstacle_roi_right = int(float(3)/float(4) * img_x)

    print("length: ",img_x,"height: ",img_y)
    global twist
    rate=rospy.Rate(10)
    twist=Twist()

# avoid the obstacles 
def obstacle_avoid(obstacles):
    index = 0
    max_area = 0
    # only consider the biggest (closest) obstacle at once
    if len(obstacles) > 1:
        for i in range(len(obstacles)):
            area = obstacles[i][2] * obstacles[i][3]
            if area > max_area:
                max_area = area
                index = i
    obstacle = obstacles[index]
    # decide turn left or turn right
    left = False
    if (obstacle[0] > (img_x - obstacle[0] - obstacle[2])):
        left = True
    global last_rotate_left
    last_rotate_left = left
    print("rotate +- 15 to avoid")
    # rotate to avoid obstacle
    while obstacles is not None:
        if left:
            angle = 15
        else:
            angle = -15
        twist.linear.x=0
        twist.angular.z=radians(angle)
        robot.publish_twist(twist)
        time.sleep(sleep_time)
        obstacles = obstacle_detect(False)
    time.sleep(sleep_time)
    # after rotate, go forward to avoid obstacles
    print("go foward to avoid obstacle")
    go_forward()

def search_goal():
    print("search goal")
    # get the center point of goal
    center_x = findGoal(robot.get_image())
    angle = 30
    # do a opposite rotate angle from last rotate, this is smarter way to search
    if last_rotate_left:
        angle = -angle
    print('try rotate',angle,' to search Goal...')
    # approxmatly turn 360 degree
    for x in range(30):
        if center_x is not -1:
            #search success
            print('search success')
            return
        twist.linear.x=0
        twist.angular.z=radians(angle)
        robot.publish_twist(twist)
        time.sleep(sleep_time)
        center_x = findGoal(robot.get_image())
    print('search fail, dead end') 
    # if search of goal fail (vision blocked by obstacles), then do the dead_end precedure
    dead_end()
    pass

# in dead end, turn 360 degree to find the closest obstacle(consider far obstacle)
def dead_end():
    obstacles = obstacle_detect(True)
    while obstacles is None:
        twist.linear.x=0
        twist.angular.z=radians(45)
        robot.publish_twist(twist)
        time.sleep(sleep_time)
        obstacles = obstacle_detect(True)
        #if its stuck, it might still able to find the goal
        center_x = findGoal(robot.get_image())
        if center_x is not None and center_x is not -1:
            return None
    #now avoid its closest obstacle
    obstacle_avoid(obstacles)
    return None


# find goal, three type of return
def find_goal():
    center_x = findGoal(robot.get_image())
    if center_x is None:
        # arrive
        return True
    else:
        if center_x is -1:
            search_goal()
            return False
        else:
            print('found, now rotate to goal')
            rotate_left = False
            if goal_roi_left > center_x:
                rotate_left = True
            while True:
                # when found, rotate its angle to the center_x of the goal
                if goal_roi_left < center_x < goal_roi_right:
                    break
                if center_x == -1:
                    search_goal()
                    return False
                rotate_to_goal(rotate_left)
                center_x = findGoal(robot.get_image())
            # no obstacles? good to go
            if obstacle_detect(False) is None:
                print("go foward to goal")
                go_forward()
                newcenter_x = findGoal(robot.get_image())
                if newcenter_x == center_x:
                #stuck
                    print("go forward stuck")
                    go_back()
            return False
        

# a go_back function for another version of dead_end deal, no use for now
def go_back():
    print("go backward...")
    twist.linear.x=-step_distance
    robot.publish_twist(twist)
    time.sleep(sleep_time)
    pass

#simply rotate a angle, set to 10, kind of slow, but more acrruate 
def rotate_to_goal(rotate_left):
    angle = 10
    if not rotate_left:
        angle = -angle
    #print('rotate ', {angle}, ' degree')
    twist.linear.x=0
    twist.angular.z=radians(angle)
    robot.publish_twist(twist)
    time.sleep(sleep_time)

#simply go forward
def go_forward():
    print("go forward...")
    twist.linear.x=step_distance
    robot.publish_twist(twist)
    time.sleep(sleep_time)
    pass

# first look strategy for the first look at the map
def first_look():
    if  -1 == findGoal(robot.get_image()):
        dead_end()
    pass

# the main loop, see my report for detail flow
def Run():
    first_look()
    while True:
        obstacles = obstacle_detect(False)
        if obstacles is not None:
            print("now avoid")
            obstacle_avoid(obstacles)
            continue
        else:
            print("now find goal")
            arrive = find_goal()
            if arrive:
                print("arrive")
                break
    rospy.loginfo("exiting from Run()")

if __name__ == '__main__':
    #instant a robot object
    global robot
    robot=Robot()
    init()
    Run()