A temporal logic controller synthesis approach for controlling a wind farm and an energy storage system for frequency regulation with provable probabilistic safety guarantees in the stochastic environment of wind power generation.

Installing Dependencies:

This code depends on YALMIP, which can be obtained with the Multi-Parametric Toolbox, or MPT3, see http://control.ee.ethz.ch/~mpt/3/Main/Installation. MPT is also required for plotting polyhedras.

We use the Gurobi solver as back-end to solve the optimization problem, though other solvers might work as well. For the user-interactive example to work without modifications, Gurobi needs to be installed and configured for Matlab. See http://www.gurobi.com.

Example:

After everything is installed, you should also change the path in loading the two mat files in generate_critical_constraints_Single_Agent_MILPpower2.m to the current path. Then you can
run the file 'windfullorderMILPstorage_acc.m' to generate the optimal input signals and run the file 'SDE_feedforward.m' to simulate the trajectories of the grid frequency deviations and the wind turbine generator rotor speed deviations with the stochastic environment of wind power generation.

Contact Us:

You can contact xuz8@rpi.edu for any queries or to report any bugs in the code.