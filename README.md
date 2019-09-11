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

Please set the objective funcion as "min 0", and the first and second constraint as "f1(x)<=0" and "f2(x)<=0", respectively. The code will automatically process your LP file and tranform it to the format of BOILP.

# Compiling and Running
Compiling the project can be done using the Terminal by going to the directory in which the project is saved and typing ”make clean” followed by ”make” (you can also compile through Netbeans).

An instance can be solved by typing

./the_combined_method  <*address*>/<*instance*>  <*address*>/<*summary*>  <*address*>/<*frontier*>  <*name of instance*>

where “instance” is the original .lp file of the instance, “summary” is the report of the solution about runtime and number of integer programs, and “frontier” is the nondominated frontier for the BOILP instance.

For better understanging, we have provided a sample shell script named "script" and two sets of instances in the repository. The folder named "2DKP_Instance" contains 20 instances for bi-objective 2-Dimensional Knapsack Problem, and "AP_Instance" for Bi_objective Assignment Problem. In order to solve the first instance in the folder of "2DKP_Instance", one can use the following code in terminal

make clean                       
make          
./the_combined_method 2DKP_Instance/"2DKP_1.lp" "Report_2DKP.txt" "ND_sets_2DKP_1.txt" "2DKP_1"

or directly run the script in the terminal.

# Supporting and Citing
The software in this ecosystem was developed as part of academic research. If you would like to help support it, please star the repository as such metrics may help us secure funding in the future. We would be grateful if you could cite:

[Dai, R., Charkhgard, H., A Two-stage Approach for Bi-objective Integer Linear Programming, Operations Research Letters 46 (1), 81-87, 2018.] (https://doi.org/10.1016/j.orl.2017.11.011)
