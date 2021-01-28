# CBF-CLF-Helper
Matlab Interface for Control Barrier Function (CBF) and Control Lyapunov Function (CLF) based control methods. The library is designed to let users easily implement safety controller based on CBFs and CLFs with Matlab. We provide:
- An easy interface for construction and simulation of nonlinear control-affine systems.
- Safety controllers including CLF-QP, CBF-QP, and CBF-CLF-QP as built-in functions.
- Demonstrations on toy examples.


### Requirements
- Matlab
- [Symbolic Math Toolbox](https://www.mathworks.com/products/symbolic.html)

### Usage
1. Create a class that inherit `CtrlAffineSys`.
2. Create a class function `defineSystem` and define your dynamics using the symbolic toolbox.
3. Create class functions `defineClf` and `defineCbf` and define your CLF and CBF in each function respectively using the same symbolic expressions.
4. To run the simulation or run the controller, create a class instance with parameters specified as a Matlab structure array, and use the built-in functions—dynamics and other controllers such as `ctrlCbfClfQp`, `ctrlClfQp`, etc.

Please checkout the [Manual](https://github.com/HybridRobotics/CBF-CLF-Helper/blob/master/Manual_v1.pdf) for more details.

### Demos
Run files in the directory `demos` in Matlab.
