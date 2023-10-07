import numpy as np
import matplotlib.pyplot as plt

data = np.array(np.genfromtxt('data.txt', delimiter=','))
# print(data.shape)

plt.figure()
plt.plot(data[:,[0, 4, 8, 12]], data[:,[1, 5, 9, 13]], '-o')
plt.legend(['True Position','Meas Position', 'EKF Estimate', 'UKF Estimate'])
plt.xlabel('x')
plt.ylabel('y')

plt.figure()
plt.plot(data[:,[2, 6, 10, 14]])
# plt.plot(data[:,[3, 7, 11, 15]])
plt.legend(['True Position','Meas Position', 'EKF Estimate', 'UKF Estimate'])

plt.figure()
# plt.plot(data[:,[2, 6, 10, 14]])
plt.plot(data[:,[3, 7, 11, 15]])
plt.legend(['True Position','Meas Position', 'EKF Estimate', 'UKF Estimate'])

# plt.figure()
# ekf_error = np.sqrt((data[:,0]-data[:,6])**2 + (data[:,1]-data[:,7])**2)
# ukf_error = np.sqrt((data[:,0]-data[:,9])**2 + (data[:,1]-data[:,10])**2)
# plt.plot(ekf_error)
# plt.plot(ukf_error)
# plt.legend(['EKF Error', 'UKF Error'])
# plt.xlabel('Iteration')
# plt.ylabel('Error')

plt.show()