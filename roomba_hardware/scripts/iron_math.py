from scipy.optimize import minimize
import numpy as np
import csv

data = []
with open('/home/blaine/compass.csv', newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        data.append([float(x) for x in row])
data = np.array(data)[:, :2]


def transform(x):
    offsets = x[:2]
    matrix = x[2:].reshape(2,2)
    return (data-offsets) @ matrix


def loss(x):
    transformed_data = transform(x)
    return np.mean((transformed_data[:, 0] ** 2 + transformed_data[:, 1] ** 2 - 1) ** 2)



x0 = np.array([
    np.mean(data[:,0]), 
    np.mean(data[:,1]), 
    1/np.std(data[:,0]), 0, 
    0, 1/np.std(data[:,1])
])
ans = minimize(loss, x0, method = 'Nelder-Mead', options={'maxiter': 10000})

import rospkg
import yaml
params = list([float(x) for x in ans.x])
with open(rospkg.RosPack().get_path("roomba_hardware") + "/cfg/iron_offsets.yaml", 'w') as f:
    yaml.dump({"hard": params[:2], "soft": params[2:]}, f)

transformed_data = transform(ans.x)
with open('/home/blaine/compass_corrected.csv', 'w') as f:
    for row in transformed_data:
        f.write("%f,%f\n" % (row[0], row[1]))

print(ans)