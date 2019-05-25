# PPQL
This is the package for motion planning and control for a quadrotor with a suspended load

# Prerequisites
- [Yalmip](https://yalmip.github.io/) - Modelling language for optimization in Matlab
- [Ipopt](https://github.com/coin-or/Ipopt) - Optimization solver

# Usage
Download this package into the folder you want.
## Optimization solver
### Matlab Solver
- run `setup-matlab.m` to add matlab folders and files.
- run offline path planning using different example files. The optimization results will be stored in the folder `/data_offline`
### Python Interface
- After the offline optimization results were stored, the python interface help to extract the desired trajectory for further usage.

