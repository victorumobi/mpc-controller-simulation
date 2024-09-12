import matplotlib.pyplot as plt

# Example time and cross-track error data
time = [0, 1, 2, 3, 4, 5]
cross_track_error = [0.5, 0.4, 0.3, 0.2, 0.1, 0.0]

plt.plot(time, cross_track_error)
plt.xlabel('Time (s)')
plt.ylabel('Cross-track Error (m)')
plt.title('Cross-track Error Over Time')
plt.show()
