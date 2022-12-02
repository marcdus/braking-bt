# Simulation of Mass-Spring-Damper System
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID

# x_set = 2
# pid = PID(70, 1, 2.5, setpoint = x_set)

params = {
    "tunings": (1, 0, 0),
    "setpoint": 4,
    "output_limits": (0, 1000),
    "pid_on": True,
    "prop_on_measurement": False,
}

pid = PID()
pid.tunings = params["tunings"]
pid.setpoint = params["setpoint"]
pid.output_limits = params["output_limits"]
pid.auto_mode = params["pid_on"]
pid.proportional_on_measurement = params["prop_on_measurement"]


# Model Parameters
c = 4 # Damping constant
k = 4 # Stiffness of the spring
m = 10 # Mass
F = 0.0 # Force (INPUT)
F_update = 0.0
# Simulation Parameters
Ts = 0.05

pid.sample_time = Ts

Tstart = 0
Tstop = 60
N = int((Tstop-Tstart)/Ts) # Simulation length
x1 = np.zeros(N+2)
x2 = np.zeros(N+2)
x1[0] = 0 # Initial Position
x2[0] = 0 # Initial Speed
a11 = 1
a12 = Ts
a21 = -(Ts*k)/m
a22 = 1 - (Ts*c)/m
b1 = 0
b2 = Ts/m
# Simulation
for k in range(N+1):
    x1[k+1] = a11 * x1[k] + a12 * x2[k] + b1 * F # position
    x2[k+1] = a21 * x1[k] + a22 * x2[k] + b2 * F # velocity
    F = pid(x1[k+1], Ts)

# Plot the Simulation Results
t = np.arange(Tstart,Tstop+2*Ts,Ts)
#plt.plot(t, x1, t, x2)
plt.plot(t,x1)
plt.plot(t,x2)
plt.title('Simulation of Mass-Spring-Damper System')
plt.xlabel('t [s]')
plt.ylabel('x(t)')
plt.grid()
plt.legend(["x1", "x2"])
plt.show()

print(pid.tunings)
