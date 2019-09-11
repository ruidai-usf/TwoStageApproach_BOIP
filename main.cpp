/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: david
 *
 * Created on November 1, 2016, 10:41 PM
 */

#include <cstdlib>
#include <iostream> /*This is a package for reading/writing from/in screen in C++ */
#include <fstream> /*This is a package for reading/writing files C++ */
#include <ilcplex/ilocplex.h>
#include <vector> /*This is a package for creating dynamic lists */
#include <ctime>
#include <math.h>

#define Box_Constraints_Enabling 0
#define Fesible_Solution_Enabbling 1
#define E_Cons_Feasible_Solution_Enabling 1
#define Box_Initializing_Enabling 0
#define Set_Para_Threads 1
#define Num_of_Threads 1
#define Set_Para_Time 0
#define Num_of_Rectangles_Checking 15
#define IP_type_check_Num 3
#define Cplex_Time_Limit 30
#define All_Obj_Parameters_Int 1
#define Largest_Area_First 1
#define Time_limit 100000
#define Hypervolumn_Gap_Stopping 0
#define epsilon 0.5
#define Optimality_Gap 0.00001
#define Positive_infinity 100000000
#define Negative_infinity -100000000
#define Combining_result_number 8

using namespace std;
double start_time(0);
double finish_time(0);

ILOSTLBEGIN
IloEnv env;
IloModel model(env);
IloObjective obj_fun;
IloRangeArray Extra_Constraints(env);

IloNumVarArray startVar(env);
IloNumArray startVal(env);

IloRangeArray rngs(env);
IloSOS1Array sos1(env);
IloSOS2Array sos2(env);
IloNumVarArray dec_var(env);
IloObjective object(env);

IloExprArray obj(env);
IloCplex BBM_cplex(model);


int N_Variables;
int N_Objectives(2);
int Is_one_ND_point(0);
int Num_of_ND_points(0);
int Num_of_Rectangles(0);

int Num_of_IPs(0);
int E_Cons_Num_of_IPs(0);
int IP_type;

int Is_bottom_empty;
int Is_top_empty;
int Is_bottom_one;
int Is_top_one;
int Is_one_num(0);

double Total_Runtime(0);
double E_Cons_Runtime(0);

double IP_Runtime(0);
double E_Cons_IP_Runtime(0);

/*We Now Declare Our Nondominated points set*/
vector <double*> ND_points_set;
vector <double*> E_cons_ND_points_set;
vector <double> Single_IP_Runtime_set;
vector <int> IP_type_set;

/*We Now Declare Our Classes*/

class Box {
public:

    /*The top and bottom vertexes of the box*/

    double* vertex_B;
    double* vertex_T;

    /*The feasible solutions*/
    double* solution_vertex_B;
    double* solution_vertex_T;

    /*The area of the rectangle*/
    double area;
    double length;
    double width;

    /*The following is a constructor of a node that initialize a value for variables of the node */
    Box() {
        vertex_B = new double [N_Objectives];
        vertex_T = new double [N_Objectives];

        if (Box_Initializing_Enabling == 1) {
            for (int i = 0; i < N_Objectives; i++) {
                vertex_B[i] = 0;
                vertex_T[i] = 0;
            }
        }

        solution_vertex_B = new double [N_Variables];
        solution_vertex_T = new double [N_Variables];

        if (Box_Initializing_Enabling == 1) {
            for (int i = 0; i < N_Variables; i++) {
                solution_vertex_B[i] = 0;
                solution_vertex_T[i] = 0;
            }

            area = 0;
            length = 0;
            width = 0;
        }
    }

    /*We now update the information about this node based on its parent's information */

    virtual ~Box() {
        delete [] vertex_B;
        delete [] vertex_T;
        delete [] solution_vertex_B;
        delete [] solution_vertex_T;
    }
private:

};

/*We Now Declare Our Box Tree*/
vector <Box*>Tree_of_Boxes;

void Generate_Two_New_Box(Box* Parent, Box* New_bottom_box, Box* New_top_box) {

    /*return Z_1 of the top vertex in the bottom area*/
    obj_fun = IloMinimize(env, obj[0]);
    model.add(obj_fun);
    if (Box_Constraints_Enabling == 1) {
        Extra_Constraints.add(Parent->vertex_T[0] - obj[0] <= 0);
        Extra_Constraints.add(obj[0] - Parent->vertex_B[0] <= 0);
        Extra_Constraints.add(Parent->vertex_B[1] - obj[1] <= 0);
    }
    Extra_Constraints.add(obj[1] - (Parent->vertex_T[1] + Parent->vertex_B[1]) / 2 <= 0);

    model.add(Extra_Constraints);

    BBM_cplex.extract(model);

    if (Fesible_Solution_Enabbling == 1) {
        for (int j = 0; j < N_Variables; j++) {
            startVal.add(Parent->solution_vertex_B[j]);
        }

        BBM_cplex.addMIPStart(startVar, startVal);
    }

    BBM_cplex.setOut(env.getNullStream());

    if (Set_Para_Threads == 1) {
        BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
    }

    if (Set_Para_Time == 1) {
        BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
    }

    BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);

    BBM_cplex.resetTime();
    if (BBM_cplex.solve() == 0) {
        cout << "Infeasible" << endl;
    }

    Num_of_IPs++;
    IP_Runtime += BBM_cplex.getTime();

    double bottom_returned_point_1;
    bottom_returned_point_1 = BBM_cplex.getObjValue();

    if (Fesible_Solution_Enabbling == 1) {
        startVal.clear();
    }

    if (bottom_returned_point_1 <= Parent->vertex_B[0] - epsilon) {
        Is_bottom_empty = 0;

        IP_type = 0;
        IP_type_set.push_back(IP_type);

        /*generate part informations of the new bottom box*/
        New_bottom_box->vertex_T[0] = BBM_cplex.getObjValue();

        for (int i = 0; i < N_Objectives; i++) {
            New_bottom_box->vertex_B[i] = Parent->vertex_B[i];
        }

        /*recollect the feasible solution*/
        if (Fesible_Solution_Enabbling == 1) {
            for (int j = 0; j < N_Variables; j++) {
                startVal.add(BBM_cplex.getValue(dec_var[j]));
            }
        }

        BBM_cplex.clear();
        model.remove(obj_fun);
        model.remove(Extra_Constraints);
        Extra_Constraints.clear();

        //        BBM_cplex.end();
        //        BBM_cplex = IloCplex(model);

        /*return z_2 of the top vertex in the bottom area*/
        obj_fun = IloMinimize(env, obj[1]);
        model.add(obj_fun);
        Extra_Constraints.add(obj[0] - bottom_returned_point_1 <= epsilon);
        model.add(Extra_Constraints);

        BBM_cplex.extract(model);

        if (Fesible_Solution_Enabbling == 1) {
            BBM_cplex.addMIPStart(startVar, startVal);
        }

        BBM_cplex.setOut(env.getNullStream());

        if (Set_Para_Threads == 1) {
            BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
        }

        if (Set_Para_Time == 1) {
            BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
        }
        BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);

        BBM_cplex.resetTime();
        if (BBM_cplex.solve() == 0) {
            cout << "Infeasible" << endl;
        }

        Num_of_IPs++;
        IP_Runtime += BBM_cplex.getTime();

        if (Fesible_Solution_Enabbling == 1) {
            startVal.clear();
        }

        IP_type = 0;
        IP_type_set.push_back(IP_type);

        /*generate part informations of the new bottom box*/
        New_bottom_box->vertex_T[1] = BBM_cplex.getObjValue();

        New_bottom_box->area = (New_bottom_box->vertex_B[0] - New_bottom_box->vertex_T[0])
                *(New_bottom_box->vertex_T[1] - New_bottom_box->vertex_B[1]);

        if (All_Obj_Parameters_Int == 1) {
            New_bottom_box->length = New_bottom_box->vertex_B[0] - New_bottom_box->vertex_T[0];
            New_bottom_box->width = New_bottom_box->vertex_T[1] - New_bottom_box->vertex_B[1];
        }

        if (All_Obj_Parameters_Int == 1 && (New_bottom_box->length <= 1 + epsilon
                || New_bottom_box->width <= 1 + epsilon)) {
            Is_bottom_one = 1;
            Is_one_num++;

        } else {
            Is_bottom_one = 0;

            /*recollect the feasible solution*/
            if (Fesible_Solution_Enabbling == 1) {
                for (int j = 0; j < N_Variables; j++) {
                    New_bottom_box->solution_vertex_B[j] = Parent->solution_vertex_B[j];
                    New_bottom_box->solution_vertex_T[j] = BBM_cplex.getValue(dec_var[j]);
                }
            }

        }
    } else {
        Is_bottom_empty = 1;

        IP_type = 1;
        IP_type_set.push_back(IP_type);

    }


    BBM_cplex.clear();
    model.remove(obj_fun);
    model.remove(Extra_Constraints);
    Extra_Constraints.clear();

    //    BBM_cplex.end();
    //    BBM_cplex = IloCplex(model);

    /*return Z_2 of the bottom vertex in the top area*/
    if ((Parent->vertex_T[1] - Parent->vertex_B[1]) / 2 <= 1 || (New_bottom_box->vertex_T[0]
            - Parent->vertex_T[0]) <= 1) {
        Is_top_empty = 1;
    } else {
        obj_fun = IloMinimize(env, obj[1]);
        model.add(obj_fun);
        if (Box_Constraints_Enabling == 1) {
            Extra_Constraints.add(Parent->vertex_T[0] - obj[0] <= 0);
            Extra_Constraints.add(obj[1] - Parent->vertex_T[1] <= 0);
            Extra_Constraints.add((Parent->vertex_T[1] + Parent->vertex_B[1]) / 2 - obj[1] <= 0);
        }
        Extra_Constraints.add(obj[0] - bottom_returned_point_1 + epsilon <= 0);

        model.add(Extra_Constraints);

        BBM_cplex.extract(model);

        if (Fesible_Solution_Enabbling == 1) {
            for (int j = 0; j < N_Variables; j++) {
                startVal.add(Parent->solution_vertex_T[j]);
            }

            BBM_cplex.addMIPStart(startVar, startVal);
        }

        BBM_cplex.setOut(env.getNullStream());

        if (Set_Para_Threads == 1) {
            BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
        }

        if (Set_Para_Time == 1) {
            BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
        }

        BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);

        BBM_cplex.resetTime();
        if (BBM_cplex.solve() == 0) {
            cout << "Infeasible" << endl;
        }

        Num_of_IPs++;
        IP_Runtime += BBM_cplex.getTime();

        double top_returned_point_2;
        top_returned_point_2 = BBM_cplex.getObjValue();

        if (Fesible_Solution_Enabbling == 1) {
            startVal.clear();
        }

        if (top_returned_point_2 <= Parent->vertex_T[1] - epsilon) {
            Is_top_empty = 0;

            IP_type = 0;
            IP_type_set.push_back(IP_type);

            /*generate part informations of the new bottom box*/
            New_top_box->vertex_B[1] = BBM_cplex.getObjValue();

            for (int i = 0; i < N_Objectives; i++) {
                New_top_box->vertex_T[i] = Parent->vertex_T[i];
            }

            /*recollect the feasible solution*/
            if (Fesible_Solution_Enabbling == 1) {
                for (int j = 0; j < N_Variables; j++) {
                    startVal.add(BBM_cplex.getValue(dec_var[j]));
                }
            }

            BBM_cplex.clear();
            model.remove(obj_fun);
            model.remove(Extra_Constraints);
            Extra_Constraints.clear();

            //        BBM_cplex.end();
            //        BBM_cplex = IloCplex(model);

            /*return z_1 of the bottom vertex in the top area*/
            obj_fun = IloMinimize(env, obj[0]);
            model.add(obj_fun);
            Extra_Constraints.add(obj[1] - New_top_box->vertex_B[1] <= epsilon);
            model.add(Extra_Constraints);

            BBM_cplex.extract(model);

            if (Fesible_Solution_Enabbling == 1) {
                BBM_cplex.addMIPStart(startVar, startVal);
            }

            BBM_cplex.setOut(env.getNullStream());

            if (Set_Para_Threads == 1) {
                BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
            }

            if (Set_Para_Time == 1) {
                BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
            }

            BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);
            BBM_cplex.resetTime();
            if (BBM_cplex.solve() == 0) {
                cout << "Infeasible" << endl;
            }

            Num_of_IPs++;
            IP_Runtime += BBM_cplex.getTime();

            if (Fesible_Solution_Enabbling == 1) {
                startVal.clear();
            }

            IP_type = 0;
            IP_type_set.push_back(IP_type);

            /*generate part informations of the new bottom box*/
            New_top_box->vertex_B[0] = BBM_cplex.getObjValue();

            New_top_box->area = (New_top_box->vertex_B[0] - New_top_box->vertex_T[0])
                    *(New_top_box->vertex_T[1] - New_top_box->vertex_B[1]);

            if (All_Obj_Parameters_Int == 1) {
                New_top_box->length = New_top_box->vertex_B[0] - New_top_box->vertex_T[0];
                New_top_box->width = New_top_box->vertex_T[1] - New_top_box->vertex_B[1];
            }

            if (All_Obj_Parameters_Int == 1 && (New_top_box->length <= 1 + epsilon
                    || New_top_box->width <= 1 + epsilon)) {
                Is_top_one = 1;
                Is_one_num++;
            } else {
                Is_top_one = 0;

                /*recollect the feasible solution*/
                if (Fesible_Solution_Enabbling == 1) {
                    for (int j = 0; j < N_Variables; j++) {
                        New_top_box->solution_vertex_T[j] = Parent->solution_vertex_T[j];
                        New_top_box->solution_vertex_B[j] = BBM_cplex.getValue(dec_var[j]);
                    }
                }
            }
        } else {
            Is_top_empty = 1;

            IP_type = 1;
            IP_type_set.push_back(IP_type);

        }

        BBM_cplex.clear();
        model.remove(obj_fun);
        model.remove(Extra_Constraints);
        Extra_Constraints.clear();

        //    BBM_cplex.end();
        //    BBM_cplex = IloCplex(model);
    }
}

void Add_the_new_nondominated_point_to_the_set(double* New_nondominated_point) {
    bool It_is_Added(0);

    for (int i = 0; i < ND_points_set.size(); i++) {
        if (New_nondominated_point[0] < ND_points_set.at(i)[0]) {
            ND_points_set.insert(ND_points_set.begin() + i, New_nondominated_point);
            It_is_Added = 1;
            break;
        } else {
            if (New_nondominated_point[0] == ND_points_set.at(i)[0]) {
                cout << "Warning: a nondominated point is generated which we did not add to the set" << endl;
                It_is_Added = 1;
                break;
            }
        }
    }

    if (It_is_Added == 0) {

        ND_points_set.push_back(New_nondominated_point);
    }
}

void Epsilon_Constaints_Method(Box * Target_Box) {

    double* Bottom_ND_point;
    double* Top_ND_point;
    double* Previous_Top_ND_point;
    double E_Cons_Delta_Hypervolumn_UB;
    double E_Cons_Delta_Hypervolumn_LB;

    Bottom_ND_point = new double [N_Objectives];
    Top_ND_point = new double [N_Objectives];
    Previous_Top_ND_point = new double [N_Objectives];

    for (int i = 0; i < N_Objectives; i++) {
        Bottom_ND_point[i] = Target_Box->vertex_B[i];
        Top_ND_point[i] = Target_Box->vertex_T[i];
        Previous_Top_ND_point[i] = Target_Box->vertex_T[i];
    }

    int Searching_Done = 0;

    while (Searching_Done == 0) {

        obj_fun = IloMinimize(env, obj[0]);
        model.add(obj_fun);
        if (Box_Constraints_Enabling == 1) {
            Extra_Constraints.add(Top_ND_point[0] - obj[0] <= 0);
            Extra_Constraints.add(obj[0] - Bottom_ND_point[0] <= 0);
            Extra_Constraints.add(Bottom_ND_point[1] - obj[1] <= 0);
        }
        Extra_Constraints.add(obj[1] - Top_ND_point[1] <= -epsilon);

        model.add(Extra_Constraints);

        BBM_cplex.extract(model);

        if (E_Cons_Feasible_Solution_Enabling == 1) {
            for (int j = 0; j < N_Variables; j++) {
                startVal.add(Target_Box->solution_vertex_B[j]);
            }

            BBM_cplex.addMIPStart(startVar, startVal);
        }

        BBM_cplex.setOut(env.getNullStream());

        if (Set_Para_Threads == 1) {
            BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
        }

        if (Set_Para_Time == 1) {
            BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
        }

        BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);
        BBM_cplex.resetTime();
        if (BBM_cplex.solve() == 0) {
            cout << "Infeasible" << endl;
        }

        Num_of_IPs++;
        E_Cons_Num_of_IPs++;

        IP_Runtime += BBM_cplex.getTime();
        E_Cons_IP_Runtime += BBM_cplex.getTime();

        Top_ND_point[0] = BBM_cplex.getObjValue();

        if (E_Cons_Feasible_Solution_Enabling == 1) {
            startVal.clear();
        }

        if (Top_ND_point[0] <= Bottom_ND_point[0] - epsilon) {

            IP_type = 0;
            IP_type_set.push_back(IP_type);

            if (E_Cons_Feasible_Solution_Enabling == 1) {
                for (int j = 0; j < N_Variables; j++) {
                    startVal.add(BBM_cplex.getValue(dec_var[j]));
                }
            }

            BBM_cplex.clear();
            model.remove(obj_fun);
            model.remove(Extra_Constraints);
            Extra_Constraints.clear();

            //            BBM_cplex.end();
            //            BBM_cplex = IloCplex(model);


            obj_fun = IloMinimize(env, obj[1]);
            model.add(obj_fun);
            Extra_Constraints.add(obj[0] - Top_ND_point[0] <= epsilon);
            model.add(Extra_Constraints);

            BBM_cplex.extract(model);

            if (E_Cons_Feasible_Solution_Enabling == 1) {
                BBM_cplex.addMIPStart(startVar, startVal);
            }

            BBM_cplex.setOut(env.getNullStream());

            if (Set_Para_Threads == 1) {
                BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
            }

            if (Set_Para_Time == 1) {
                BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
            }
            BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);

            BBM_cplex.resetTime();
            if (BBM_cplex.solve() == 0) {
                cout << "Infeasible" << endl;
            }

            Num_of_IPs++;
            E_Cons_Num_of_IPs++;

            IP_Runtime += BBM_cplex.getTime();
            E_Cons_IP_Runtime += BBM_cplex.getTime();

            IP_type = 0;
            IP_type_set.push_back(IP_type);

            Top_ND_point[1] = BBM_cplex.getObjValue();

            if (E_Cons_Feasible_Solution_Enabling == 1) {
                startVal.clear();
            }

            double* New_E_cons_point = new double [N_Objectives];
            for (int i = 0; i < N_Objectives; i++) {
                New_E_cons_point[i] = Top_ND_point[i];
            }

            Add_the_new_nondominated_point_to_the_set(New_E_cons_point);

            E_Cons_Delta_Hypervolumn_UB = -(Top_ND_point[0] - Previous_Top_ND_point[0])
                    *(Previous_Top_ND_point[1] - Bottom_ND_point[1]);

            E_Cons_Delta_Hypervolumn_LB = (Bottom_ND_point[0] - Top_ND_point[0])
                    *(Previous_Top_ND_point[1] - Top_ND_point[1]);

            for (int i = 0; i < N_Objectives; i++) {
                Previous_Top_ND_point[i] = Top_ND_point[i];
            }

            if (All_Obj_Parameters_Int == 1) {
                double Now_length;
                double Now_width;
                Now_length = Bottom_ND_point[0] - Top_ND_point[0];
                Now_width = Top_ND_point[1] - Bottom_ND_point[1];

                if (Now_length <= 1 + epsilon || Now_width <= 1 + epsilon) {
                    Searching_Done = 1;
                }
            }
        } else {
            Searching_Done = 1;

            IP_type = 1;
            IP_type_set.push_back(IP_type);
        }

        BBM_cplex.clear();
        model.remove(obj_fun);
        model.remove(Extra_Constraints);
        Extra_Constraints.clear();

        //        BBM_cplex.end();
        //        BBM_cplex = IloCplex(model);
    }

    delete [] Bottom_ND_point;
    delete [] Top_ND_point;
    delete [] Previous_Top_ND_point;
}

void Create_The_Box_ZERO() {

    /* Create a New Node and call it New_Box */
    Box* Initial_Box = new Box;

    /*get the value of Vertex_B[1]*/

    obj_fun = IloMinimize(env, obj[1]);

    /*add objective function into the model*/
    model.add(obj_fun);

    /*generate present cplex model*/
    BBM_cplex.extract(model);

    /*make cplex not to output to terminal*/
    BBM_cplex.setOut(env.getNullStream());

    /*set cplex parameter*/
    if (Set_Para_Threads == 1) {
        BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
    }

    if (Set_Para_Time == 1) {
        BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
    }
    BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);
    /*let cplex solve the model*/
    BBM_cplex.resetTime();
    if (BBM_cplex.solve() == 0) {
        cout << "Infeasible" << endl;
    }

    Num_of_IPs++;
    IP_Runtime += BBM_cplex.getTime();

    Initial_Box->vertex_B[1] = BBM_cplex.getObjValue();

    /*collect the feasible solution*/
    if (Fesible_Solution_Enabbling == 1) {
        for (int j = 0; j < N_Variables; j++) {
            startVal.add(BBM_cplex.getValue(dec_var[j]));
        }
    }

    IP_type = 0;
    IP_type_set.push_back(IP_type);

    BBM_cplex.clear();
    model.remove(obj_fun);

    //    BBM_cplex.end();
    //    BBM_cplex = IloCplex(model);

    /*get the value of vertex_B[0]*/
    obj_fun = IloMinimize(env, obj[0]);
    model.add(obj_fun);
    Extra_Constraints.add(obj[1] - Initial_Box->vertex_B[1] <= epsilon);
    model.add(Extra_Constraints);

    BBM_cplex.extract(model);

    /*use the feasible solution to start the solver*/
    if (Fesible_Solution_Enabbling == 1) {
        BBM_cplex.addMIPStart(startVar, startVal);
    }

    BBM_cplex.setOut(env.getNullStream());

    if (Set_Para_Threads == 1) {
        BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
    }

    if (Set_Para_Time == 1) {
        BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
    }

    BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);
    BBM_cplex.resetTime();
    if (BBM_cplex.solve() == 0) {
        cout << "Infeasible" << endl;
    }

    Num_of_IPs++;
    IP_Runtime += BBM_cplex.getTime();

    Initial_Box->vertex_B[0] = BBM_cplex.getObjValue();

    /*clear the value of start variable*/
    if (Fesible_Solution_Enabbling == 1) {
        startVal.clear();

        /*recollect the feasible solution*/
        for (int j = 0; j < N_Variables; j++) {
            startVal.add(BBM_cplex.getValue(dec_var[j]));
            Initial_Box->solution_vertex_B[j] = BBM_cplex.getValue(dec_var[j]);
        }
    }

    IP_type = 0;
    IP_type_set.push_back(IP_type);

    BBM_cplex.clear();
    model.remove(Extra_Constraints);
    Extra_Constraints.clear();

    //    BBM_cplex.end();
    //    BBM_cplex = IloCplex(model);

    /*get the value of vertex_T[0]*/

    BBM_cplex.extract(model);

    if (Fesible_Solution_Enabbling == 1) {
        BBM_cplex.addMIPStart(startVar, startVal);
    }

    BBM_cplex.setOut(env.getNullStream());

    if (Set_Para_Threads == 1) {
        BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
    }

    if (Set_Para_Time == 1) {
        BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
    }

    BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);
    BBM_cplex.resetTime();
    if (BBM_cplex.solve() == 0) {
        cout << "Infeasible" << endl;
    }

    Num_of_IPs++;
    IP_Runtime += BBM_cplex.getTime();

    Initial_Box->vertex_T[0] = BBM_cplex.getObjValue();

    if (Fesible_Solution_Enabbling == 1) {
        startVal.clear();

        for (int j = 0; j < N_Variables; j++) {
            startVal.add(BBM_cplex.getValue(dec_var[j]));
        }
    }

    BBM_cplex.clear();
    model.remove(obj_fun);

    //    BBM_cplex.end();
    //    BBM_cplex = IloCplex(model);

    if (Initial_Box->vertex_B[0] == Initial_Box->vertex_T[0]) {
        Is_one_ND_point = 1;

        IP_type = 1;
        IP_type_set.push_back(IP_type);
    }

    /*get the value of vertex_T[1]*/
    if (Is_one_ND_point == 0) {

        IP_type = 0;
        IP_type_set.push_back(IP_type);

        obj_fun = IloMinimize(env, obj[1]);

        model.add(obj_fun);
        Extra_Constraints.add(obj[0] - Initial_Box->vertex_T[0] <= epsilon);
        model.add(Extra_Constraints);

        BBM_cplex.extract(model);

        if (Fesible_Solution_Enabbling == 1) {
            BBM_cplex.addMIPStart(startVar, startVal);
        }

        BBM_cplex.setOut(env.getNullStream());

        if (Set_Para_Threads == 1) {
            BBM_cplex.setParam(IloCplex::Threads, Num_of_Threads);
        }

        if (Set_Para_Time == 1) {
            BBM_cplex.setParam(IloCplex::TiLim, Cplex_Time_Limit);
        }

        BBM_cplex.setParam(IloCplex::EpGap, Optimality_Gap);
        BBM_cplex.resetTime();
        if (BBM_cplex.solve() == 0) {
            cout << "Infeasible" << endl;
        }

        Num_of_IPs++;
        IP_Runtime += BBM_cplex.getTime();

        Initial_Box->vertex_T[1] = BBM_cplex.getObjValue();

        if (Fesible_Solution_Enabbling == 1) {
            for (int j = 0; j < N_Variables; j++) {
                Initial_Box->solution_vertex_T[j] = BBM_cplex.getValue(dec_var[j]);
                startVal.clear();
            }
        }

        IP_type = 0;
        IP_type_set.push_back(IP_type);

        BBM_cplex.clear();

        model.remove(obj_fun);
        model.remove(Extra_Constraints);
        Extra_Constraints.clear();

        //        BBM_cplex.end();
        //        BBM_cplex = IloCplex(model);

        Initial_Box->area = (Initial_Box->vertex_B[0] - Initial_Box->vertex_T[0])*(Initial_Box->vertex_T[1]
                - Initial_Box->vertex_B[1]);

        if (All_Obj_Parameters_Int == 1) {

            Initial_Box->length = Initial_Box->vertex_B[0] - Initial_Box->vertex_T[0];
            Initial_Box->width = Initial_Box->vertex_T[1] - Initial_Box->vertex_B[1];
        }
    }

    Tree_of_Boxes.push_back(Initial_Box);

}

void Add_the_new_Box_to_the_tree(Box * New_Generated_Box) {
    bool It_is_Added(0);

    if (Largest_Area_First == 1) {
        for (int i = 1; i < Tree_of_Boxes.size(); i++) {
            if (New_Generated_Box-> area >= Tree_of_Boxes.at(i)-> area) {
                Tree_of_Boxes.insert(Tree_of_Boxes.begin() + i, New_Generated_Box);
                It_is_Added = 1;
                break;
            }
        }
    }


    if (It_is_Added == 0) {

        Tree_of_Boxes.push_back(New_Generated_Box);
    }

}

void Writing_The_Output_File(char* Report_file, char* ND_set_file, char* Case_name) {
    finish_time = clock();
    ofstream OutputFile;
    OutputFile.open(ND_set_file);
    for (int i = 0; i < ND_points_set.size(); i++) {
        OutputFile << ND_points_set.at(i)[0] << "    " << ND_points_set.at(i)[1] << endl;
    }
    OutputFile << endl;
    //    OutputFile << endl;
    //    for (int i = 0; i < ND_points_set.size(); i++) {
    //        OutputFile << E_cons_ND_points_set.at(i)[0] << "    " << E_cons_ND_points_set.at(i)[1] << endl;
    //    }
    //    OutputFile << endl;
    OutputFile.close();

    OutputFile.open(Report_file, ios::app);
    OutputFile << Case_name << " ";

    OutputFile << Total_Runtime << " ";
    OutputFile << Total_Runtime - E_Cons_Runtime << " ";
    OutputFile << E_Cons_Runtime << " ";

    OutputFile << IP_Runtime << " ";
    OutputFile << IP_Runtime - E_Cons_IP_Runtime << " ";
    OutputFile << E_Cons_IP_Runtime << " ";

    OutputFile << Num_of_IPs << " ";
    OutputFile << Num_of_IPs - E_Cons_Num_of_IPs << " ";
    OutputFile << E_Cons_Num_of_IPs << " ";

    Num_of_ND_points = ND_points_set.size();
    OutputFile << Num_of_ND_points << " ";
    OutputFile << Num_of_Rectangles << " ";
    OutputFile << endl;

    OutputFile.close();
}

void Reading_Input_File(char* Name_of_File) {

    BBM_cplex.importModel(model, Name_of_File, object, dec_var, rngs, sos1, sos2);

    obj = IloExprArray(env, N_Objectives);

    for (int i = 0; i < N_Objectives; i++) {
        obj[i] = rngs[i].getExpr();
    }

    for (int i = 0; i < N_Objectives; i++) {
        model.remove(rngs[i]);
    }

    model.remove(object);

    N_Variables = dec_var.getSize();

    /*import the decision variables into the start variable*/
    for (int j = 0; j < N_Variables; j++) {

        startVar.add(dec_var[j]);
    }
}

int main(int argc, char** argv) {
    Reading_Input_File(argv[1]);

    start_time = clock();

    Create_The_Box_ZERO();

    double* Initial1_point = new double [N_Objectives];

    for (int i = 0; i < N_Objectives; i++) {
        Initial1_point[i] = Tree_of_Boxes.front()->vertex_B[i];
    }

    ND_points_set.push_back(Initial1_point);

    if (Is_one_ND_point == 0) {

        double* Initial2_point = new double [N_Objectives];

        for (int i = 0; i < N_Objectives; i++) {
            Initial2_point[i] = Tree_of_Boxes.front()->vertex_T[i];
        }

        ND_points_set.insert(ND_points_set.begin(), Initial2_point);

        int Initial_Box_Check1 = 0;
        int Initial_Box_Check2 = 0;

        if (All_Obj_Parameters_Int == 1) {
            Initial_Box_Check1 = 1;
            if (Tree_of_Boxes.front()->length > 1 && Tree_of_Boxes.front()->width > 1) {
                Initial_Box_Check2 = 1;
            }
        }
        if (Initial_Box_Check1 == 0 || Initial_Box_Check2 == 1) {

            while (Tree_of_Boxes.size() > 0) {

                if (Num_of_Rectangles >= Num_of_Rectangles_Checking) {

                    int IP_type_check(0);

                    for (int i = 0; i < Num_of_Rectangles_Checking; i++) {
                        IP_type_check += IP_type_set.at(IP_type_set.size() - 1 - i);
                    }

                    if (IP_type_check >= IP_type_check_Num) {

                        double E_Cons_start_time;
                        E_Cons_start_time = clock();

                        for (int i = 0; i < Tree_of_Boxes.size(); i++) {
                            Epsilon_Constaints_Method(Tree_of_Boxes.at(i));
                        }

                        E_Cons_Runtime = (clock() - E_Cons_start_time) / CLOCKS_PER_SEC;

                        break;
                    }
                }

                Box* Bottom_temp_box = new Box;
                Box* Top_temp_box = new Box;

                Generate_Two_New_Box(Tree_of_Boxes.front(), Bottom_temp_box, Top_temp_box);

                if (Is_bottom_empty == 0) {

                    double* New1_point = new double [N_Objectives];
                    for (int i = 0; i < N_Objectives; i++) {
                        New1_point[i] = Bottom_temp_box->vertex_T[i];
                        //                        cout << New1_point[i] << "; ";
                    }
                    //                    cout << endl;
                    Add_the_new_nondominated_point_to_the_set(New1_point);

                    if (Is_bottom_one == 0) {
                        Add_the_new_Box_to_the_tree(Bottom_temp_box);
                    } else {
                        Bottom_temp_box->~Box();
                    }

                } else {
                    Bottom_temp_box->~Box();
                }

                if (Is_top_empty == 0) {

                    double* New2_point = new double [N_Objectives];
                    for (int i = 0; i < N_Objectives; i++) {
                        New2_point[i] = Top_temp_box->vertex_B[i];
                        //                        cout << New2_point[i] << "; ";
                    }
                    //                    cout << endl;

                    Add_the_new_nondominated_point_to_the_set(New2_point);

                    if (Is_top_one == 0) {
                        Add_the_new_Box_to_the_tree(Top_temp_box);
                    } else {
                        Top_temp_box->~Box();
                    }

                } else {
                    Top_temp_box->~Box();
                }

                Tree_of_Boxes.front()->~Box();
                Tree_of_Boxes.erase(Tree_of_Boxes.begin());
                Num_of_Rectangles++;

            }
        } else {
            Tree_of_Boxes.front()->~Box();
            Tree_of_Boxes.erase(Tree_of_Boxes.begin());
            Num_of_Rectangles++;
        }
    }

    Total_Runtime = (clock() - start_time) / CLOCKS_PER_SEC;

    //        cout << "One_Box: " << Is_one_num << endl;
    Writing_The_Output_File(argv[2], argv[3], argv[4]);
    return 0;
    cout << endl;
}
