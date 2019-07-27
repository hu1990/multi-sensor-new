Here are something about sensor bias estimation.

`smart_sensor_simulation.m`: 

Simulation scenario is as follows.

Two bearing-only sensors cooperate to track a constant velocity target.
The sensors are moving in a constant speed, too, while:
- in Simulation 1, Sensor 2 moves as same as Sensor 1;
- in Simulation 2, Sensor 2 decides an optimal heading angle.

Filter method used is OTSEKF.

Simulation results are plotted.

`fim_max.mlx`:

A realtime script showing how to solve the optimal heading angle.

`ekf_predict_update.m`:

An implementation of EKF.

`test_asekf.m`:

An EKF demo.

2D scenario.
Use two bearing-only sensors to track a constant velocity target.

Filter model:
  - system model:
    CV model + constant sensor bias
  - measurement model:
    two bearing-only sensors + constant bias

`otsekf_predict_update.m`:

An implementation of OTSEKF (optimal two stage extended Kalman filter).
*There exists a linearization approximation.*

`test_otsekf.m`:

An OTSEKF demo.

The simulation scenario is as same as in `test_asekf.m`.

`EM.m`:

Used for inplement EM method for linear system identification.
This example is from `An explanation of the Expectation Maximization Algorithm`.
The MATLAB code in the paper and the relavent toolbox are referenced.
