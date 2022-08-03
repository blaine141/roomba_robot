#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped, Twist
from std_msgs.msg import Float32
import numpy as np
from scipy.optimize import minimize
from math import atan2, sin, cos
import rospkg
import yaml
import matplotlib.pyplot as plt

PUBLISH_RATE = 20
SAMPLING_DURATION = 20

VELOCITIES = [
    4,
    2,
    -2, 
    -4
]

compass_queue = []

def compass_cb(data: Vector3Stamped):
    vector = data.vector
    compass_queue.append([vector.x, vector.y, vector.z])

def collect_data_spin():
    compass_queue.clear()
    rate = rospy.Rate(PUBLISH_RATE)
    turn_msg = Twist()
    for velocity in VELOCITIES:
        turn_msg.angular.z = velocity
        for _ in range(PUBLISH_RATE * SAMPLING_DURATION):
            if rospy.is_shutdown():
                exit()
            vel_pub.publish(turn_msg)
            rate.sleep()

    vel_pub.publish(Twist())
    return compass_queue.copy()

def collect_data_stationary(num_points):
    compass_queue.clear()
    while(len(compass_queue) < num_points):
        rospy.sleep(0.1)
    return compass_queue.copy()


def ransac(data,model,n,k,t,d,debug=False,return_all=False):

    iterations = 0
    bestfit = None
    besterr = np.inf
    best_inlier_idxs = None
    while iterations < k:
        maybe_idxs, test_idxs = random_partition(n,data.shape[0])
        maybeinliers = data[maybe_idxs,:]
        test_points = data[test_idxs]
        maybemodel = model.fit(maybeinliers)
        test_err = model.get_error( test_points, maybemodel)
        also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
        alsoinliers = data[also_idxs,:]
        if debug:
            print('test_err.min()',test_err.min())
            print('test_err.max()',test_err.max())
            print('numpy.mean(test_err)',np.mean(test_err))
            print('iteration %d:len(alsoinliers) = %d'%(
                iterations,len(alsoinliers)))
        if len(alsoinliers) > d:
            betterdata = np.concatenate( (maybeinliers, alsoinliers) )
            bettermodel = model.fit(betterdata)
            better_errs = model.get_error( betterdata, bettermodel)
            thiserr = np.mean( better_errs )
            if thiserr < besterr:
                bestfit = bettermodel
                besterr = thiserr
                best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )
        iterations+=1
    if bestfit is None:
        raise ValueError("did not meet fit acceptance criteria")
    if return_all:
        return bestfit, {'inliers':best_inlier_idxs}
    else:
        return bestfit

def random_partition(n,n_data):
    """return n random rows of data (and also the other len(data)-n rows)"""
    all_idxs = np.arange( n_data )
    np.random.shuffle(all_idxs)
    idxs1 = all_idxs[:int(n)]
    idxs2 = all_idxs[int(n):]
    return idxs1, idxs2

class MagnetometerModel:

    def __init__(self):
        self.x0 = None

    def parse_params(self, parameters):
        offsets = parameters[:3]
        stretch = s = parameters[3]
        stretch_sin_cos = np.array(parameters[4:6])
        stretch_sin_cos /= np.linalg.norm(stretch_sin_cos)
        ss, sc = stretch_sin_cos
        f1, f2 = parameters[6:8]
        radius = parameters[8]

        matrix = np.identity(3)
        matrix[0, 0] = s*sc**2 + ss**2
        matrix[1, 0] = matrix[0, 1] = ss*sc*(1-s)
        matrix[1, 1] = s*ss**2 + sc**2
        matrix[0, 2] = f1
        matrix[1, 2] = f2
        matrix /= radius

        return offsets, matrix
    
        
    def fit(self, data):
        data = np.array(data)

        if self.x0 is None:
            self.x0 = np.array([
                *np.mean(data, axis=0),                 # Offsets
                1,                                      # Stretch
                0, 1,                                   # Stretch Sin Cos
                0, 0,                                   # Flattening params
                np.mean(np.std(data[:,:2], axis=0))     # Radius
            ])

        bounds = [
            (None, None),  (None, None), (None, None),  # Offsets
            (1, None),                                  # Stretch
            (-1, 1), (-1, 1),                           # Stretch Sin Cos
            (None, None), (None, None),                 # Flattening params
            (0, None)                                   # Radius
        ]
        ans = minimize(lambda parameters: np.mean(self.get_error(data, parameters)), self.x0, method = 'Nelder-Mead', options={'maxiter': 10000}, bounds=bounds)
        print(ans)
        return ans.x

    def get_error(self, data, parameters):
        offsets, matrix = self.parse_params(parameters)

        transformed_data = (data-offsets) @ matrix
        return (np.sqrt(transformed_data[:, 0] ** 2 + transformed_data[:, 1] ** 2) - 1) ** 2 + transformed_data[:, 2] ** 2


if __name__ == '__main__':
    rospy.init_node("calibrate_irons")

    compass_sub = rospy.Subscriber("compass_raw", Vector3Stamped, compass_cb)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    main_brush_pub = rospy.Publisher("main_brush", Float32, queue_size=10)

    # Wait for publishers to connect
    rospy.sleep(10)

    brush_off_data = []
    brush_on_data = []

    # Collect data at the same angle with main brush on and off
    main_brush_pub.publish(0.75)
    rospy.sleep(5)
    brush_on_data += collect_data_stationary(200)
    main_brush_pub.publish(0)
    rospy.sleep(5)
    brush_off_data += collect_data_stationary(200)

    # Spin around to collect data at every angle
    brush_off_data += collect_data_spin()
    main_brush_pub.publish(.75)
    rospy.sleep(5)
    brush_on_data += collect_data_spin()

    # Stop brush and save data for debug
    main_brush_pub.publish(0)
    np.savetxt("brush_off.csv", brush_off_data, delimiter=",")
    np.savetxt("brush_on.csv", brush_on_data, delimiter=",")



    # Create models and initialize parameters for faster convergence
    brush_off_model = MagnetometerModel()
    brush_off_model.x0 = brush_off_model.fit(brush_off_data)
    brush_on_model = MagnetometerModel()
    brush_on_model.x0 = brush_on_model.fit(brush_on_data)

    # Get mean error and use that to fit the model with ransac
    brush_off_mean_error = np.mean(brush_off_model.get_error(brush_off_data, brush_off_model.x0))
    transforms = ransac(np.array(brush_off_data), brush_off_model, 0.1 * len(brush_off_data), 100, brush_off_mean_error, 0.6 * len(brush_off_data))
    off_hard, off_soft = brush_off_model.parse_params(transforms)

    brush_on_mean_error = np.mean(brush_on_model.get_error(brush_on_data, brush_on_model.x0))
    transforms = ransac(np.array(brush_on_data), brush_on_model, 0.1 * len(brush_on_data), 100, brush_on_mean_error, 0.6 * len(brush_on_data))
    on_hard, on_soft = brush_on_model.parse_params(transforms)

    # Average first 200 points which are all at the same heading
    on_start = np.mean(brush_on_data[:200], axis=0)
    off_start = np.mean(brush_off_data[:200], axis=0)

    # Calculate heading
    off_start_corrected = (off_start - off_hard) @ off_soft
    on_start_corrected = (on_start - on_hard) @ on_soft
    on_start_angle = atan2(on_start_corrected[1], on_start_corrected[0])
    off_start_angle = atan2(off_start_corrected[1], off_start_corrected[0])
    print("On angle: %f" % on_start_angle)
    print("Off angle: %f" % off_start_angle)

    # Rotate the on transform so that the headings match with both models
    dt = off_start_angle - on_start_angle
    heading_correct = np.identity(3)
    heading_correct[0,0] = heading_correct[1,1] = cos(dt)
    heading_correct[0,1] = sin(dt)
    heading_correct[1,0] = -sin(dt)
    on_soft = on_soft @ heading_correct

    # Save models
    transforms = {
        "main_brush_on": {"hard": on_hard.tolist(), "soft": on_soft.tolist()},
        "main_brush_off": {"hard": off_hard.tolist(), "soft": off_soft.tolist()}
    }
    with open(rospkg.RosPack().get_path("roomba_hardware") + "/cfg/magnetometer_calibration.yaml", 'w') as f:
        yaml.dump(transforms, f)

    brush_on_data_corrected = (brush_on_data - on_hard) @ on_soft
    brush_off_data_corrected = (brush_off_data - off_hard) @ off_soft

    data = np.vstack((brush_off_data_corrected, brush_on_data_corrected))
    data = np.array(data)


    angles = np.arctan2(data[:,1], data[:,0])
    radii = np.linalg.norm(data[:,0:2], axis=1)

    plt.scatter(angles, radii)
    plt.show()


    # Creating figure
    fig = plt.figure(figsize = (10, 7))
    ax = plt.axes(projection ="3d")
    
    # Creating plot
    ax.scatter3D(data[:,0], data[:,1], data[:,2], color = "green")
    plt.title("simple 3D scatter plot")
    
    # show plot
    plt.show()