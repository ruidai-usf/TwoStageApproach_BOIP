# TwoStageApproach_BOIP
A two-stage approach for solving bi-objective pure integer linear programs

This is an algorithm to solve a class of Bi-objective Pure Integer Linear Programs (BOILP).

A BOILP can be stated as follows:

min {f1(x), f2(x)} 
s.t. x ∈ D

where f1(x) and f2(x) are two objectives to minimize, x is non-negative integer, D is the feasible region for x to draw its value.

This project is a Netbeans IDE 8.2 C++ project which was written in Linux (Ubuntu).

To compile the code, CPLEX (default version 12.7) must be installed on your computer. The default directory for CPLEX used is /opt/CPLEX/12.7/. Changing the directory of CPLEX to your preferred directory can be done either in the Makefile or through Netbeans. If you would like to do it in the Makefile you should go to nbproject/Makefile-Debug.mk and nbproject/Makefile-Release.mk and change all instances of /opt/CPLEX/12.7/ to your preferred directory. If you would like to do it through Netbeans, you can open the project in Netbeans and right click on the name of the project and choose Properties. You can then change the directory in the Include Directories box which is located in the C++ Compiler sub-menu. Moreover, you should also change the directory in the Libraries box which is located in the Linker sub-menu.

# Data Files
The data file should be written as a CPLEX LP file. 

Please set the objective funcion as "min 0", and the first and second constraint as f1(x)<=0 and f2(x)<=0, respectively. The code will automatically process your LP file and tranform it to the format of BOILP.
