# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
This project shows how the steering angle and gas throttle could be controlled using the model predictive control technique where the model try to optimize the controlled variables (steering & throttle) through predicting the optimal waypoints car shall go through depending on a set of model constraints.

## Model Description
Mainly we're utilizing the global kinematic model for this problem. 
Initially the system state consists of the following parameters: 
1- X-position of the vehicle.
2- Y-position of the vehicle.
3- Orientation angle of the vehicle heading.
4- Vehicle velocity.

The inputs/actuations where this state will be update upon which are:
1- Steering angle "psi".
2- Gas throttle/acceleration "a"

To update the following state we're using the folloing equations set:
 ![State Update Eqns](eqns.png "State Update Eqns")

Lf is a physical characteristic of the vehicle measures the distance between the front of the vehicle and its center of gravity. The larger the vehicle , the slower the turn rate.

### Errors
We can capture how the errors we are interested in change over time by deriving our kinematic model around these errors as our new state vector.

The new state is [x,y,ψ,v,cte,eψ].

#### Cross Track Error
![Cross Track Error](07-l-errors-03.png "Cross Track Error")
We can express the error between the center of the road and the vehicle's position as the cross track error (CTE). The CTE of the successor state after time t is the state at t + 1, and is defined as:

ctet+1=ctet+vt∗sin(eψt)∗dt

In this case ctetcte_tctet​ can be expressed as the difference between the line and the current vehicle position y. Assuming the reference line is a 1st order polynomial f, f(xt) is our reference line and our CTE at the current state is defined as:

ctet=yt−f(xt)

If we substitute ctetcte_tctet​ back into the original equation the result is:

ctet+1=yt−f(xt)+(vt∗sin(eψt)∗dt)

This can be broken up into two parts:

    yt−f(xt) being current cross track error.
    vt∗sin(eψt)∗dt being the change in error caused by the vehicle's movement.

#### Orientation Error
![Orientation Error](07-l-errors-02.png "Orientation Error")
eψt​ is the desired orientation subtracted from the current orientation:

eψt=ψt−ψdest

We already know ψt\psi_tψt​, because it’s part of our state. We don’t yet know ψdest​ (desired psi) - all we have so far is a polynomial to follow. ψdest​ can be calculated as the tangential angle of the polynomial f evaluated at xt​*arctan(f′(xt))*f′ is the derivative of the polynomial.

eψt+1=ψt−ψdest+(vt*Lf*δt*dt)

Similarly to the cross track error this can be interpreted as two parts:

    ψt−ψdest being current orientation error.
    vt*Lf*δt*dt being the change in error caused by the vehicle's movement.

### Time Horizon & Step

I've decided to select the horizon to be one second since this suits the simulated scenario here (i.e. to actuate a vehicle drived on highway for example). 

Then I started tuning the duration between the time steps firstly selected (N = 10, dt = 0.1) and observed the behavior of the model and ended up that I need to actuate more frequently to overcome the discretization problem while keeping the model not computationally heavy as possible, finally I selected (N = 20, dt = 0.05) keeping the horizon as it's but with more frequent actuations. 

### Polynomial Fitting and MPC Preprocessing
We're receiving 6 waypoints represented in the map coordinate system every time step.
Firstly we're transforming them into the vehicle coordinate system to make it easire in all the successive calculations. 
Using the homogenous transformation equations to handle the axes rotation and the origin translations as follows: 

* ptsx_veh[i] = (ptsx[i] - px)*cos(-(psi)) -  (ptsy[i] - py)*sin(-(psi));
* ptsy_veh[i] = (ptsy[i] - py)*cos(-(psi)) +  (ptsx[i] - px)*sin(-(psi)); 

Then fitting the transformed waypoints into a 3rd order polynomial since it's the most suitable for the reference trajectory. 
After that we set the X,Y positions of the vehicle and psi angle to zero according to the vehicle coordinate system.

We then calculate the CTE and the psi error using the fitted polynomial and its derivative. 
Now the state vector and the polynomial coeffs are ready to be passed to the optimizer to calculate new set actuations with minimal cost.

### Model Predictive Control with Latency
The following segment of code uses the kinematic equations to predict the states for after 100ms before sending them to MPC. 
We've placed this update just after the polynomial fitting so the state params were firstly updated according to the vehicle coordinate system.

		px = 0;
          py = 0;
          psi = 0;
          double fx = polyeval(coeffs, 0); // px = py = 0
          double cte = fx;
          double des_psi = atan(coeffs[1]);
          double epsi = -des_psi; //psi = 0

- Latency = 0.1 sec:

	  
		px = px + v*cos(psi)*latency;
          py = py + v*sin(psi)*latency;
          psi = psi + v*delta*latency/Lf;
          v = v + a*latency;
          cte= cte + v*sin(epsi)*latency;
          epsi = epsi + v*epsi*latency/Lf;

### Speed and Track Completion Tuning
In order to complete one lap over the driveable portion of the track, I had to do the following tunings: 

#### 1- Cost components tuning as following: 
The major issue observed after completing the model implementation was the erratic/aggressive steering actuations (The car completed one lap but with aggressive behavior), so I tried a set of multiple tunings some of them did the job as follows:
- Increased the weight of the CTE & Epsi error components but this caused more erratic actuations (Rejected tuning).
- Increased the weight of actuation components equally to force a smoother actuation and actually this degraded the system aggressiveness (Accepted tuning).
- Increased the weight of actuation delta components equally to eliminate the erratic actuation over time and actually this enhanced the actuations stability (Accepted tuning).

#### 2- Speed Tuning
- Started with a speed of 25 mph, then increased it to 40 mph and still the car do the job autonomously but the speed in the simulator doesn't exceed 20 mph.
- Then increased it to 60 mph while the weights of the actuation components and the actuation deltas in the cost were ~50, the vehicle actuations were erratic enough to get it out of the drivable lane in one of the sharp turns.
- Then set it to 60 mph and increased the weights of the actuation components and the actuation deltas in the cost to ~100 & ~75 respectively, the vehicle could steer smoothly and the speed in the simulator increased to 30 mph.

Finally I've set the reference velocity to 60 mph.


