import numpy as np
import matplotlib.pyplot as plt

data = np.load("trust_validation.npz")["trust_data"]
t = data[:, 0] - data[0, 0]
trust = data[:, 1]
gamma = data[:, 2]
v_score = data[:, 3]
d_score = data[:, 4]

plt.figure(figsize=(10, 8))
plt.subplot(4,1,1)
plt.plot(t, trust)
plt.ylabel("Trust Score")
plt.grid(True)

plt.subplot(4,1,2)
plt.plot(t, gamma)
plt.ylabel("Gamma")
plt.grid(True)

plt.subplot(4,1,3)
plt.plot(t, v_score)
plt.ylabel("Velocity Score")
plt.grid(True)

plt.subplot(4,1,4)
plt.plot(t, d_score)
plt.ylabel("Distance Score")
plt.xlabel("Time [s]")
plt.grid(True)

plt.tight_layout()
plt.show()
