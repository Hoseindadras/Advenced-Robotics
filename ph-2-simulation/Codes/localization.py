#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import map
import numpy as np
from math import *
from os.path import expanduser
import matplotlib.pyplot as plt
import scipy


num_prtcls = 1000

laser_var = 0.0037
v_var = 0.00342
w_var = 0.0354

x = 0.0
y = 0.0
theta = 0.0

PI = 3.1415926535897

laser_data = 0
laser = 0

desired_time = 3

rotation_list = [PI / 2, 3 * (PI / 2), 0, PI]

home = expanduser("~")
map_address = home + '/catkin_ws/src/anki_description/world/sample4.world'
rects, global_map_pose, map_boundry = map.init_map(map_address)
x_min, x_max, y_min, y_max = map_boundry
x0, y0 = float(global_map_pose[0]), float(global_map_pose[1])
all_map_lines = map.convert_point_to_line(rects)

rects = map.add_offset(rects, [x0, y0])
all_map_lines = map.convert_point_to_line(rects)
polygan = map.convert_to_poly(rects)



def callback_laser(msg):
    global laser
    laser = msg.range


def new_odometry(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def motion_model(prtcl_weight, v, w, dt, v_var=v_var, w_var=w_var):
    for i in range(prtcl_weight.shape[0]):

        v_hat = v + np.random.normal(0, v_var)
        w_hat = w + np.random.normal(0, w_var)
        if w_hat == 0:
            w_hat = 1e-9

        prtcl_weight[:, 0][i] = prtcl_weight[:, 0][i] + (v_hat / w_hat) * (
                - sin(prtcl_weight[:, 2][i]) + sin(
            prtcl_weight[:, 2][i] + w_hat * dt))

        prtcl_weight[:, 1][i] = prtcl_weight[:, 1][i] + (v_hat / w_hat) * (
                cos(prtcl_weight[:, 2][i]) - cos(prtcl_weight[:, 2][i] + w_hat * dt))

        prtcl_weight[:, 2][i] = prtcl_weight[:, 2][i] + dt * w_hat + 10 * PI
        prtcl_weight[:, 2][i] = prtcl_weight[:, 2][i] - int(
            prtcl_weight[:, 2][i] / (2 * PI)) * 2 * PI

    return prtcl_weight


def measurment_model(prtcl_weight, z, laser_var=laser_var, map=map,
                     all_map_lines=all_map_lines):
    max_laser = 0.435
    sensor_var = laser_var

    for i in range(prtcl_weight.shape[0]):

        laser_available = False

        prtcl_start = [prtcl_weight[:, 0][i], prtcl_weight[:, 1][i]]
        prtcl_end = [prtcl_weight[:, 0][i] + max_laser * cos(prtcl_weight[:, 2][i]),
                     prtcl_weight[:, 1][i] + max_laser * sin(prtcl_weight[:, 2][i])]
        min_distance = 10
        col = False
        for line in all_map_lines:
            intersection_point = map.find_intersection(line[0], line[1], prtcl_start,
                                                       prtcl_end)
            if intersection_point:
                d_laser = ((intersection_point[0] - prtcl_weight[:, 0][i]) ** 2 + (
                        intersection_point[1] - prtcl_weight[:, 1][i]) ** 2) ** 0.5
                if min_distance >= d_laser:
                    min_distance = d_laser
                    col = intersection_point

        if not col:
            min_distance = max_laser

        d_laser = min_distance

        prtcl_weight[:, 3][i] = scipy.stats.norm(d_laser, sensor_var).pdf(z + 0.035)

    avg = np.mean(prtcl_weight[:, 3])
    summ = np.sum(prtcl_weight[:, 3])
    prtcl_weight[:, 3] = prtcl_weight[:, 3] / summ
    sum2 = np.sum(prtcl_weight[:, 3] ** 2)

    return prtcl_weight, avg, sum2


def collision_handler(prtcl_weight, polygan=polygan, x_min=x_min, x_max=x_max, y_min=y_min,
                    y_max=y_max, x0=x0, y0=y0, map_boundry=map_boundry, delete=False):
    out = 0
    delete_list = []
    x_list = np.arange(x_min, x_max, 0.02)
    y_list = np.arange(y_min, y_max, 0.02)
    for i in range(prtcl_weight.shape[0]):

        if map.check_is_collision([prtcl_weight[:, 0][i], prtcl_weight[:, 1][i]],
                                  polygan) or map.out_of_range(
            [prtcl_weight[:, 0][i], prtcl_weight[:, 1][i]], [x0, y0], map_boundry):
            out = out + 1
            if delete:
                delete_list.append(i)

            else:

                while map.check_is_collision(
                        [prtcl_weight[:, 0][i], prtcl_weight[:, 1][i]],
                        polygan) or map.out_of_range(
                    [prtcl_weight[:, 0][i], prtcl_weight[:, 1][i]], [x0, y0],
                    map_boundry):
                    prtcl_weight[:, 0][i] = np.round(np.random.choice(x_list) + x0, 2)
                    prtcl_weight[:, 1][i] = np.round(np.random.choice(y_list) + y0, 2)

    if delete:
        prtcl_weight = np.delete(prtcl_weight, delete_list, axis=0)
    summ = prtcl_weight[:, 3].sum()
    prtcl_weight[:, 3] = prtcl_weight[:, 3] / summ
    return prtcl_weight, out


def generate_particles(x_min=x_min, x_max=x_max, x0=x0, y_min=y_min, y_max=y_max, y0=y0,
              rotation_list=rotation_list, num_prtcls=num_prtcls):
    x_list = np.arange(x_min, x_max, 0.02)
    y_list = np.arange(y_min, y_max, 0.02)
    prtcl_x = (np.random.choice(x_list, num_prtcls) + x0).reshape(-1, 1)
    prtcl_y = (np.random.choice(y_list, num_prtcls) + y0).reshape(-1, 1)
    prtcl_theta = (np.random.choice(rotation_list, num_prtcls)).reshape(-1, 1)
    weights = (np.ones(num_prtcls) / num_prtcls).reshape(-1, 1)
    prtcl_weight = np.concatenate([prtcl_x, prtcl_y, prtcl_theta, weights], axis=1)
    return prtcl_weight


def plotter(prtcl_weight, x_val, y_val, theta_val, map=map, all_map_lines=all_map_lines):
    import matplotlib.pyplot as plt
    plt.clf()
    plt.gca().invert_yaxis()
    map.plot_map(all_map_lines)
    index = np.argsort(-prtcl_weight[:, 3])[:int(prtcl_weight.shape[0] * 1)]
    prtcl_weight = prtcl_weight[:, :][index]
    plt.arrow(y_val, x_val, 1e-5 * sin(theta_val), 1e-5 * cos(theta_val), color='cyan',
              head_width=0.03, overhang=0.5)
    circle1 = plt.Circle((y_val, x_val), 0.05, color='r')
    plt.gca().add_patch(circle1)
    for i in range(prtcl_weight.shape[0]):
        plt.arrow(prtcl_weight[:, 1][i], prtcl_weight[:, 0][i],
                  1e-5 * sin(prtcl_weight[:, 2][i]), 1e-5 * cos(prtcl_weight[:, 2][i]),
                  color='black', head_width=0.02, overhang=0.6)
    plt.draw()
    plt.pause(0.1)


def get_movement(key):
    dist = 0
    angle = 0
    if key in ['w', 's']:
        angle = 0
        if key == 'w':
            dist = 0.1
        elif key == 's':
            dist = -0.1
    elif key in ['a', 'd']:
        dist = 0
        if key == 'a':
            angle = PI / 2
        elif key == 'd':
            angle = -PI / 2
    return dist, angle


if __name__ == '__main__':
    rospy.init_node("Localization")
    Odometry_reader = rospy.Subscriber("/odom", Odometry, new_odometry)
    velocity_publisher = rospy.Publisher('/vector/cmd_vel', Twist, queue_size=1)
    laser_reader = rospy.Subscriber('/vector/laser', Range, callback_laser, queue_size=1)
    vel_msg = Twist()

    X_W = generate_particles()

    X_W, num_out = collision_handler(X_W)

    X_W_update = np.copy(X_W)
    q = 0  # iteration counter
    x_real, y_real, theta_real = x, y, theta
    list_portion = []

    alpha_slow = 0.3
    alpha_fast = 0.7
    w_slow = 0
    w_fast = 0
    w_avg = 0
    portion = 0
    while not rospy.is_shutdown():
        q = q + 1
        print("**************")
        print("iteration:" + str(q))

        #  Movement
        print("Press w, s, a, or d to move:")

        key = input()
        while key not in ['w', 'a', 's', 'd']:
            key = input()
        dist, angle = get_movement(key)

        t0 = rospy.Time.now().to_sec()
        while t0 <= 0:
            t0 = rospy.Time.now().to_sec()

        dt = 0
        speed_t = (dist / desired_time)
        speed_r = (angle / desired_time)
        laser_reading = laser

        update = False
        move = False
        while dt < desired_time:
            if laser_reading > 0.04:
                vel_msg.linear.x = speed_t
                vel_msg.angular.z = speed_r * 2
                velocity_publisher.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                dt = t1 - t0
                update = True
                move = True

            elif (laser_reading <= 0.04) and ((dist < 0) or (angle != 0)):
                vel_msg.linear.x = speed_t
                vel_msg.angular.z = speed_r * 2
                velocity_publisher.publish(vel_msg)
                t1 = rospy.Time.now().to_sec()
                dt = t1 - t0
                update = True
                move = True
            else:
                break
            laser_reading = laser

        laser_reading = laser
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        # Updating
        if update:
            X_W = motion_model(X_W, v=speed_t, w=speed_r, dt=dt)
            X_W, num_out = collision_handler(X_W, delete=True)
            laser_reading = laser
            x_real, y_real, theta_real = x, y, theta
            X_W, w_avg, sum2 = measurment_model(X_W, z=laser_reading)
            X_W_update = np.copy(X_W)
            xy_prtcl = X_W[:, :2]
            xy_real = np.array([x_real, y_real]).reshape(1, -1)
            close_prtcls_index = np.where(np.linalg.norm(xy_prtcl - xy_real, axis=1) <= 0.05)
            portion = ((len(close_prtcls_index[0]) / X_W_update.shape[0]) * 100)
            list_portion.append(portion)
            print("Portion of close particles =" + str(portion) + "%")

            # AMCL
            w_slow = w_slow + alpha_slow * (w_avg - w_slow)
            w_fast = w_fast + alpha_fast * (w_avg - w_fast)

            rand_sz = int((np.max([0, 1 - (w_fast / w_slow)])) * num_prtcls)
            rand_sz = np.min([int(0.1 * num_prtcls), rand_sz])

            valid_prtcl_sz = X_W.shape[0]

            k1 = 0.8  # Percentage of particles randomly chosen considering weights
            k2 = 0.15  # Percentage of best particles
            k3 = 0.05  # Percentage of random particles

            X_W_random = generate_particles(num_prtcls=int(k3 * num_prtcls + rand_sz))
            X_W_random, num_out = collision_handler(X_W_random)

            if valid_prtcl_sz > 0:
                index_best = np.random.choice(valid_prtcl_sz, int(num_prtcls * k1),
                                              p=X_W[:, 3])
                random_best = X_W[:, :][index_best]
                index_best = (-X_W[:, 3]).argsort()[:int(k2 * num_prtcls)]
                all_best = X_W[:, :][index_best]

            else:
                random_best = generate_particles(num_prtcls=int(num_prtcls * (k1)))
                random_best, num_out = collision_handler(random_best)
                all_best = generate_particles(num_prtcls=int(num_prtcls * k2))
                all_best, num_out = collision_handler(all_best)

            X_W = np.concatenate([all_best, random_best, X_W_random], axis=0)

        # Show result
        plotter(X_W_update, x_real, y_real, theta_real)
        xw_sz = X_W_update.shape[0]
        index_best = (-X_W_update[:, 3]).argsort()[:int(0.8 * xw_sz)]
        X_W_top_estimation = X_W_update[:, :][index_best]
        mno = X_W_top_estimation
        X_W_top_estimation = X_W_top_estimation[:, :3]
        X_W_mean_estimation = np.mean(X_W_top_estimation, axis=0)

        print("real pose: ", x, y, theta)
        print("estimated pose: ", X_W_mean_estimation)
        print("number of particles: ", X_W.shape[0])

        kidnapped = False
        if portion > 80 and kidnapped:
            plt.figure()
            plt.plot(list_portion)
            plt.title("Portion of Close Particles - kidnapping")
            plt.show()
            break

        elif portion > 80:
            plt.figure()
            plt.plot(list_portion)
            plt.title("Portion of Close Particles")
            plt.show()
            kidnapped = True
