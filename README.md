# CarND-Controls-MPC

## Model Description

Kinematic model of the car is used consisting of position, velocity, orientation, cross track and orientation angle errors.

### State of the system

State of the system consists of the following:

|Variable|Description|
|---|---|
|px| x coordinate of the car|
|py| y coordinate of the car|
|Ψ (psi)| Orientation of the car (in radians)|
|v|Velocity of the car|
|CTE| Cross track error|
|EPSI|Oreintation error|

### Actuators

The following are the 2 actuators:

|Actuator|Description|Range|
|---|---|---|
| 𝛿 (delta)| Steering angle| -1 to 1 (corresponds to -25 to +25 degrees)|
| a| Throttle to be applied at each stage| -1 to 1 (Corresponding to -100 to +100 mph, and -1 implies breaking/reverse)

### State Update Equations

|State|Update Equation|
|---|---|
| px | px(t) = px(t-1) + v(t-1) * cos(Ψ(t-1)) * dt|
| py | px(t) = py(t-1) + v(t-1) * sin(Ψ(t-1)) * dt|
| Ψ | Ψ(t) = Ψ(t-1) + v(t-1) / Lf * 𝛿(t-1) * dt|
| v | v(t) = v(t-1) + a * dt|
| CTE | CTE(t) = (f(x(t-1)) - y(t-1)) + (v(t-1) * sin(EΨ(t-1)) * dt)|
| EΨ | EΨ(t) = Ψ(t-1) - Ψdes(t-1) + (v(t-1) / Lf * 𝛿(t-1) * dt)|


f(x) = a + bx + cx^2 + dx^3 (3rd order polynomial)   
Ψdes = atan(f'(x))   
f'(x) = b + 2*x + 3*x^2


*Note: Lf = 2.67*

## Model Predictive Control

At each time step, the simualtor sends the x and y coordinates of the path (in global coordinate system) that the car should follow. The path is transformed to the car coordinate system and then a polynomial is fitted. IPOPT / CppAD is used for finding out a minimized cost solution in liue of the constraints (model update equations) in mind, such that the steering angle and car speed is tweeked to follow the given polynomial path.

## Timestamp lenght and Elapsed Duration (N & Dt)

**N = 16** & **Dt = 0.1** has been kept. Earlier N = 10 and Dt = 0.05 as well as N = 26 and Dt = 0.1 was also tried. Keep a higher N resulted in longer calculation time where as keeping it lower resulted in the car not being able to follow the path.

## Preprocessing

Before handing data to the MPC solver, given points were tranformed from the global coordinate system to the car coordinate system using the following equations for each of the given points:

```
double shift_x = ptsx[i] - px;
double shift_y = ptsy[i] - py;

ptsx_transform[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
ptsy_transform[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
```

As per this transformation, px, py and Ψ ended up as 0. CTE and EΨ held information regarding how far the car was from the intended path and the orientation that it should be following.

## Polynomial Fitting

A 3rd order polynomial was fitted to the transformed points and that along with its derivative was used as part of optimization equations that were setup for IPOPT.

## Cost Function

The following factors were considered as part of cost:

|Variable|Factor|Description|
|---|---|---|
|CTE|4000|A high cost value was associated with the CTE so as to keep the car on track
|EΨ|2000|Second priority is given to the orientation error|
|Δv|1|Difference between reference velocity and current velocity|
|𝛿|5|Steering angle|
|a|5|Acceleration|
|ΔSteering|200|Change in steering angle from one timestep to the next. This was done to smooth out steering changes|
|ΔAcceleration|10|Change in throttle from one timestep to the next. This was done to smooth out throttle changes|

*Note: A basic cost factor was chosen from the classroom lesson and then it was further tweeked to make the car follow the track*

## Latency

In order to deal with latency, once the points had been transformed and px, py and Ψ were considered 0, then the state update equations were used to move the car ahead 100ms, CTE and EΨ were recomupted and then solver was called for the updated car state, CTE and EΨ.

The equations used were:

```
const double latency = 0.1;

px = v * latency;
py = 0;
psi = -v / Lf * delta * latency;
epsi += psi;
cte += v * sin(epsi) * latency;
v += a * latency;
```

*Note: These equations were mainly derived from the forum discussion: https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/64*

**Other Ideas Tried**: Another idea, that did not turn out to be successful, was to handle latency using constraints within the solver. New constraints were setup in the solver so that at 0th timestep it was told to keep the same throttle / steering angle as was given by the simulator and the rest of the throttle / steering time steps were kept open for the solver to choose a value for them. I was hoping it will choose some value for the 2nd steering / throttle that would overall minimize the cost, but that resulted in the car not being able to follow the path.