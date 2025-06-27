import numpy as np
import matplotlib.pyplot as plt

data = np.load("observer_validation.npz")
t = data["time"]
true = data["true"]
est = data["estimated"]

labels = ["x [m]", "y [m]", "θ [rad]", "v [m/s]"]

plt.figure(figsize=(12, 10))
for i in range(4):
    plt.subplot(4, 1, i+1)
    plt.plot(t, true[:, i], label="True")
    plt.plot(t, est[:, i], label="Estimated", linestyle='--')
    plt.ylabel(labels[i])
    plt.grid(True)
    if i == 0:
        plt.title("Observer Validation: True vs Estimated States")
    if i == 3:
        plt.xlabel("Time [s]")
    plt.legend()

plt.tight_layout()
plt.show()
