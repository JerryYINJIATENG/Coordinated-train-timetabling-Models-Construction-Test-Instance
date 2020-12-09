// CPLEX code and Decomposition approaches 
// By Jiateng Yin at Beijing Jiaotong

#include "stdafx.h"
#include <stdlib.h>
#include "CSVParser.h"
#include "network.h"
#include <stdio.h>
#include <iostream>
#include <ilcplex/ilocplex.h>
#include <cstdlib>
#include <process.h>
#include <math.h>
#include <cmath>
#include <time.h>
#include <ctime>
#include <numeric>
#include <algorithm>    // std::max


typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<NumVarMatrix>   NumVar3Matrix;
typedef IloArray<NumVar3Matrix>   NumVar4Matrix;

ILOSTLBEGIN

#define _MAX_TIME 300
#define _MAX_PATH 100
#define TRAIN_CAPACITY 3000
#define _MAX_SERVICE 1000
#define number_of_lines 2
NetworkForSP* g_pNetworkForSP = NULL;
std::vector<CNode> g_node_vector;
std::vector<CLink> g_link_vector;
std::vector<CPath> g_path_vector;
std::vector<timetable> planned_timetable;
std::vector<train> Trains;

std::map<int, int> g_internal_node_seq_no_map;
std::map<int, CLink*> g_pLink_map;  // link seq no
std::map<int, vector<float>>  g_path_link_volume_mapping;
std::map<int, vector<float>>  g_path_link_cost_mapping;

///////////////////////////// Parameters
// Construct the space time network by connected graph
// Use CPLEX TO SOLVE THE ORIGINAL MODEL
// Original subproblem: MIP¡¢

float g_demand_array[_MAX_ZONE][_MAX_ZONE][_MAX_TIME] = { 0.0 };
float g_demand_waiting[_MAX_ZONE][2][_MAX_TIME] = { 0.0 }; // From Line a to Line b
float g_demand_array_line_arriving[_MAX_ZONE][2][_MAX_TIME] = { 0.0 };
float g_demand_path_arriving[_MAX_PATH][_MAX_TIME] = { 0.0 };
float g_demand_path_current[_MAX_PATH][_MAX_TIME] = { 0.0 };
float g_boarding[_MAX_ZONE][_MAX_ZONE] = { 0,0 };
float cplex_boarding[_MAX_PATH][_MAX_TIME] = { 0.0 };

// float timetable[_MAX_ZONE][_MAX_ZONE][_MAX_TIME] = { 0.0 };
float r_vehicle_data[_MAX_SERVICE][_MAX_ZONE][_MAX_TIME]; // vehicle_id & destination & time
float r_station_data[_MAX_ZONE][_MAX_ZONE][_MAX_TIME]; // station_id & destination & time

													   // Small-case
int g_number_of_nodes = 0;
int g_number_of_links = 0;  // initialize  the counter to 0
int g_number_of_services = 2;
int number_of_transfer_paths = 2;
int h_min = 5;
int t_cycle = 44; // 4*9+8*1
int rolling_stock[number_of_lines] = { 7,6 };
clock_t startTime, endTime;
// LVNS parameters
// 1 Creat the struct of solutions. 
// (1) Best_solution: the best solution until current iteration
// (2) solution_pool: vecotr of serached solutions
// (3) tem_solution: a temporal solution
// (4) current_solution
typedef struct Solution
{
	int x_lt[number_of_lines][_MAX_TIME];
	float cost;
}SOLUTION;
SOLUTION best_solution;
vector<Solution> solution_pool;
SOLUTION current_solution;
vector<Solution> neighbor_solution;
int destroy_operator = 4;
int samples_generated = 30;
int repair_operator = 2;
int ExchangeOperator = 0;
// int right_move_length[number_of_lines] = { 1,1 };
int right_move_length[number_of_lines] = { 1,1 };

float linear_rate(int t)
{
	float rate;
	if (t<_MAX_TIME / 2)
	{
		rate = 4.0 / (double)_MAX_TIME*t;
	}
	else
	{
		rate = -4.0 / (double)_MAX_TIME*t + 4.0;
	}
	return rate;
}


ILOMIPINFOCALLBACK5(boundlimitCallback,
	IloCplex, cplex,
	IloBool, aborted,
	IloNum, boundBest,
	IloNum, timeLimit,
	IloNum, acceptableGap)
{
	if (!aborted  &&  hasIncumbent()) {
		IloNum boundBest = cplex.getBestObjValue();
		//IloNum timeUsed = cplex.getCplexTime() - timeStart;
		if (boundBest>=best_solution.cost) {
			getEnv().out()<< "Good enough solution"<<endl;
			aborted = IloTrue;
			abort();
		}
	}
}
void g_ReadInputData(void)
{
	int internal_node_seq_no = 0;
	double x, y;
	// step 1: read node file 
	CCSVParser parser;
	if (parser.OpenCSVFile("input_node.csv", true))
	{
		std::map<int, int> node_id_map;
		while (parser.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{

			string name;

			int node_type;
			int node_id;

			if (parser.GetValueByFieldName("node_id", node_id) == false)
				continue;

			if (g_internal_node_seq_no_map.find(node_id) != g_internal_node_seq_no_map.end())
			{
				continue; //has been defined
			}
			g_internal_node_seq_no_map[node_id] = internal_node_seq_no;


			parser.GetValueByFieldName("x", x, false);
			parser.GetValueByFieldName("y", y, false);

			CNode node;  // create a node object 

			node.node_id = node_id;
			node.node_seq_no = internal_node_seq_no;
			parser.GetValueByFieldName("zone_id", node.zone_id);
			//g_zoneid_to_zone_seq_no_mapping[212] = 212;
			node.x = x;
			node.y = y;
			internal_node_seq_no++;
			g_node_vector.push_back(node);  // push it to the global node vector
			g_number_of_nodes++;
			if (g_number_of_nodes % 1000 == 0)
				cout << "reading " << g_number_of_nodes << " nodes.. " << endl;
		}
		cout << "number of nodes = " << g_number_of_nodes << endl;
		parser.CloseCSVFile();
	}

	// step 2: read link file 

	CCSVParser parser_link;

	if (parser_link.OpenCSVFile("input_link.csv", true))
	{
		while (parser_link.ReadRecord())  // if this line contains [] mark, then we will also read field headers.
		{
			int from_node_id = 0;
			int to_node_id = 0;
			if (parser_link.GetValueByFieldName("from_node_id", from_node_id) == false)
				continue;
			if (parser_link.GetValueByFieldName("to_node_id", to_node_id) == false)
				continue;

			// add the to node id into the outbound (adjacent) node list
			// Test Jiateng cout << "Test here " << from_node_id<< endl;
			int internal_from_node_seq_no = g_internal_node_seq_no_map[from_node_id];  // map external node number to internal node seq no. 
			int internal_to_node_seq_no = g_internal_node_seq_no_map[to_node_id];
			// Test Jiateng cout << "Test here: after transform " << internal_from_node_seq_no << endl;
			CLink link;  // create a link object 

			link.from_node_seq_no = internal_from_node_seq_no;
			link.to_node_seq_no = internal_to_node_seq_no;
			link.link_seq_no = g_number_of_links;
			link.to_node_seq_no = internal_to_node_seq_no;

			parser_link.GetValueByFieldName("link_type", link.type);

			float length = 1.0; // km or mile
			float speed_limit = 1.0;
			parser_link.GetValueByFieldName("length", length);
			parser_link.GetValueByFieldName("speed_limit", speed_limit);

			parser_link.GetValueByFieldName("BPR_alpha_term", link.BRP_alpha);
			parser_link.GetValueByFieldName("BPR_beta_term", link.BRP_beta);
			int number_of_lanes = 1;
			float lane_cap = 1000;
			parser_link.GetValueByFieldName("number_of_lanes", number_of_lanes);
			parser_link.GetValueByFieldName("lane_cap", lane_cap);

			//link.m_OutflowNumLanes = number_of_lanes;//visum lane_cap is actually link_cap

			//link.link_capacity = lane_cap* number_of_lanes; //not for visum
			link.link_capacity = lane_cap;

			link.free_flow_travel_time_in_min = length / speed_limit * 60.0;

			link.length = length;
			link.cost = length / speed_limit * 60;
			// min // calculate link cost based length and speed limit // later we should also read link_capacity, calculate delay 


			g_node_vector[internal_from_node_seq_no].m_outgoing_link_vector.push_back(link);  // add this link to the corresponding node as part of outgoing node/link
			g_node_vector[internal_to_node_seq_no].m_incoming_link_seq_no_vector.push_back(link.link_seq_no);  // add this link to the corresponding node as part of outgoing node/link


			g_link_vector.push_back(link);
			g_pLink_map[link.link_seq_no] = &(g_link_vector[g_link_vector.size() - 1]);  // record the pointer to this link according to its link_seq_no

			g_number_of_links++;

			if (g_number_of_links % 10000 == 0)
				cout << "reading " << g_number_of_links << " links.. " << endl;
		}
	}
	// we now know the number of links
	cout << "number of links = " << g_number_of_links << endl;
	parser_link.CloseCSVFile();

	// step 3: read demand file 
	CCSVParser parser_demand;
	if (parser_demand.OpenCSVFile("input_demand.csv", true))
	{
		while (parser_demand.ReadRecord())
		{
			int from_zone_id = 0;
			int to_zone_id = 0;
			int time = 0;
			if (parser_demand.GetValueByFieldName("from_zone_id", from_zone_id) == false)
				continue;
			if (parser_demand.GetValueByFieldName("to_zone_id", to_zone_id) == false)
				continue;
			if (parser_demand.GetValueByFieldName("time", time) == false)
				continue;
			float number_of_passengers = 0.0;
			int origin_demand = g_internal_node_seq_no_map[from_zone_id];
			int destin_demand = g_internal_node_seq_no_map[to_zone_id];
			parser_demand.GetValueByFieldName("number_of_passengers", number_of_passengers);
			g_demand_array[origin_demand][destin_demand][time] = number_of_passengers;
		}
	}
	// None transfer paths
	for (int l = 0; l<2; l++)
	{
		for (int i = 0; i<_MAX_ZONE; i++)
		{
			for (int j = 0; j<_MAX_ZONE; j++)
			{
				if ((i<_MAX_ZONE / 2 && j<_MAX_ZONE / 2 && i<j) || (i >= _MAX_ZONE / 2 && j >= _MAX_ZONE / 2 && i<j))
				{
					CPath path;
					path.column_node_vector.push_back(i);
					path.column_node_line_vector.push_back(l);
					path.column_node_vector.push_back(j);
					path.column_node_line_vector.push_back(l);
					g_path_vector.push_back(path);
				}
			}
		}
	}
	// Path 1: 1(0) --> 1(1)
	// Path 2: 0(0) --> 3(1)
	for (int i = 0; i<2; i++)
	{
		if (i == 0)
		{
			CPath path;
			// One transfer has eight feasible paths
			path.column_node_vector.push_back(1);
			path.column_node_line_vector.push_back(0);
			path.column_node_vector.push_back(2);
			path.column_node_line_vector.push_back(0);
			path.column_node_vector.push_back(1);
			path.column_node_line_vector.push_back(1);
			path.column_node_vector.push_back(3);
			path.column_node_line_vector.push_back(1);
			g_path_vector.push_back(path);
		}
		else
		{
			CPath path;
			path.column_node_vector.push_back(0);
			path.column_node_line_vector.push_back(0);
			path.column_node_vector.push_back(2);
			path.column_node_line_vector.push_back(0);
			path.column_node_vector.push_back(1);
			path.column_node_line_vector.push_back(1);
			path.column_node_vector.push_back(3);
			path.column_node_line_vector.push_back(1);
			g_path_vector.push_back(path);
		}
	}
}

void passenger_demand_initilization(void)
{
	// define g_demand_path_arriving
	for (int t = 0; t<_MAX_TIME - 20; t++)
	{
		for (int p = 0; p<g_path_vector.size(); p++)
		{
			g_demand_path_arriving[p][t] = (int)20 * linear_rate(t);
			if (g_path_vector[p].column_node_vector[0]>_MAX_ZONE / 2 - 1)
			{
				if (t<20)
				{
					g_demand_path_arriving[p][t] = g_demand_path_arriving[p][t] = (int)g_demand_path_arriving[p][t] / 6;
				}
				else
				{
					g_demand_path_arriving[p][t] = (int)g_demand_path_arriving[p][t] / 2;
				}
			}
			if (g_path_vector[p].column_node_line_vector[0] == 0)
			{
				g_demand_path_arriving[p][t] = (int)g_demand_path_arriving[p][t]*4/5;
			}
			if (p == 40)
			{
				g_demand_path_arriving[p][t] = (int)g_demand_path_arriving[p][t] * 2;
			}
			if (p == 41)
			{
				g_demand_path_arriving[p][t] = (int)g_demand_path_arriving[p][t] * 2;
			}
			//g_demand_path_arriving[g_path_vector.size()-1][t]=50;
			g_demand_array_line_arriving[g_path_vector[p].column_node_vector[0]][g_path_vector[p].column_node_line_vector[0]][t] += g_demand_path_arriving[p][t];
		}
	}
	float total_demand = 0;
	for (int p = 0; p<g_path_vector.size(); p++)
	{
		for (int t = 0; t<_MAX_TIME; t++)
		{
			total_demand += g_demand_path_arriving[p][t];
			g_demand_path_current[p][t] = 0;
			for (int tau = 0; tau <= t; tau++)
			{
				g_demand_path_current[p][t] += g_demand_path_arriving[p][tau];
			}
		}
	}
	cout << "Test the total demand = " << total_demand << endl;
}

void new_cplex_model(void)
{
	int t_dwell = 1;
	int t_running = 4;
	int t_transfer = 2;
	int t_r[2] = {0};
	for (int i = 0; i<number_of_transfer_paths; i++)
	{
		int p_order = g_path_vector.size() - number_of_transfer_paths + i;
		if (g_path_vector[p_order].column_node_vector.size() <= 2)
		{
			cout << "Error happens" << endl;
			abort();
		}
		else
		{
			int segment_traveled = 0;
			int station_dwelled = 0;
			segment_traveled = g_path_vector[p_order].column_node_vector[1] - g_path_vector[p_order].column_node_vector[0];
			station_dwelled = segment_traveled - 1;
			t_r[i] = t_transfer + segment_traveled*t_running + t_dwell*station_dwelled;
		}

	}
//	for (int i = 0; i<number_of_transfer_paths; i++)
//	{
//		cout << "Travelling time of path " << i + g_path_vector.size() - number_of_transfer_paths << " = " << t_r[i] << endl;
//	}
	// Define variables
	IloEnv env;
	IloModel model(env);
	NumVarMatrix x_lt(env, 2);
	for (int i = 0; i<2; i++)
	{
		x_lt[i] = IloNumVarArray(env, _MAX_TIME);
		for (int t = 0; t<_MAX_TIME; t++)
		{
			x_lt[i][t] = IloNumVar(env, 0.0, 1.0, ILOINT);
		}
	}
	NumVar3Matrix n_lit(env, 2);
	for (int i = 0; i<2; i++)
	{
		n_lit[i] = NumVarMatrix(env, _MAX_ZONE);
		for (int j = 0; j<_MAX_ZONE; j++)
		{
			n_lit[i][j] = IloNumVarArray(env, _MAX_TIME);
			for (int t = 0; t<_MAX_TIME; t++)
			{
				n_lit[i][j][t] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
			}
		}
	}
	NumVarMatrix p_pt(env, g_path_vector.size());
	for (int i = 0; i<g_path_vector.size(); i++)
	{
		p_pt[i] = IloNumVarArray(env, _MAX_TIME);
		for (int t = 0; t<_MAX_TIME; t++)
		{
			p_pt[i][t] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
		}
	}
	NumVarMatrix transfer_boarding(env, 2);
	for (int i = 0; i<number_of_transfer_paths; i++)
	{
		transfer_boarding[i] = IloNumVarArray(env, _MAX_TIME);
		for (int t = 0; t<_MAX_TIME; t++)
		{
			transfer_boarding[i][t] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
		}
	}
	IloNumVar max_waiting(env, 0, IloInfinity, ILOFLOAT);
	IloRangeArray constraints(env); 
	// Add constriants
	if (ExchangeOperator == 1)
	{
		for (int i = 0; i<1; i++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				constraints.add(x_lt[i][t] - best_solution.x_lt[i][t] == 0);
			}
		}
	}
	else if (ExchangeOperator == 2)
	{
		for (int i = 1; i<2; i++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				constraints.add(x_lt[i][t] - best_solution.x_lt[i][t] == 0);
			}
		}
	}
	 //C1
	for (int i = 0; i<2; i++)
	{
		for (int j = 0; j<_MAX_ZONE; j++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				IloExpr station_waiting(env);
				for (int tau = 0; tau<t; tau++)
				{
					// Arriving passengers and Boarding passengers
					station_waiting += g_demand_array_line_arriving[j][i][tau];
					if (i == 1 && j == 1)
					{
						for (int qq = 0; qq<number_of_transfer_paths; qq++)
						{
							station_waiting -= transfer_boarding[qq][tau];
						}
					}
					int path_id = 0;
					while (path_id<g_path_vector.size())
					{
						if (g_path_vector[path_id].column_node_vector[0] == j && g_path_vector[path_id].column_node_line_vector[0] == i)
						{
							station_waiting -= p_pt[path_id][tau];
						}
						else if (g_path_vector[path_id].column_node_vector.size()>2) // this path has transfers
						{
							for (int q = 1; q<g_path_vector[path_id].column_node_vector.size() / 2; q++)// Arrive at the current line
							{
								if (g_path_vector[path_id].column_node_vector[2 * q] == j && g_path_vector[path_id].column_node_line_vector[2 * q] == i)
								{
									if (tau - t_r[path_id - 40] >= 0)
									{
										station_waiting += p_pt[path_id][tau - t_r[path_id - 40]];
									}
								}
							}
						}
						path_id++;
					}

				}
				constraints.add(max_waiting - station_waiting >= 0);
				constraints.add(n_lit[i][j][t] - station_waiting == 0);
			}
		}
	}

	// C2
	for (int l = 0; l<2; l++)
	{
		for (int i = 0; i<_MAX_ZONE; i++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				IloExpr in_vehicle_passengers(env); // Number of passengers that arrive at time t and station i
				IloExpr train_arrive_indicator(env);
				IloExpr passenger_board_indicator(env);
				int indicator = 0;
				if (t - i*(t_dwell + t_running) >= 0)
				{
					for (int p = 0; p<g_path_vector.size(); p++)
					{
						if (g_path_vector[p].column_node_line_vector[0] == l&&
							g_path_vector[p].column_node_seq[0]<i&&
							g_path_vector[p].column_node_seq[1] >= i)
						{
							in_vehicle_passengers += p_pt[p][t - (t_dwell + t_running)*(i - g_path_vector[p].column_node_seq[0])];
						}

					}
					for (int p = g_path_vector.size() - number_of_transfer_paths; p<g_path_vector.size(); p++)
					{
						//if(l==1 && g_path_vector[p].column_node_vector[g_path_vector[p].column_node_vector.size()-1]>=i
						//	//&&t-(i-g_path_vector[p].column_node_seq[g_path_vector[p].column_node_seq.size()-2])*((t_dwell+t_running))>=0) //
						//		&&t-(i-1)*((t_dwell+t_running))>=0) //
						if (l == 1 && i >= 1 && t - (i - 1)*((t_dwell + t_running)) >= 0) //
						{
							in_vehicle_passengers += transfer_boarding[p - g_path_vector.size() + number_of_transfer_paths][t - (i - 1)*((t_dwell + t_running))];
						}
					}
				}
				while (indicator<g_path_vector.size())
				{
					if (g_path_vector[indicator].column_node_line_vector[0] == l&&
						g_path_vector[indicator].column_node_vector[0] == i)
					{
						passenger_board_indicator += p_pt[indicator][t];
					}
					indicator++;
				}
				// Transfer passengers
				if (l == 1 && i == 1)
				{
					for (int qq = 0; qq<number_of_transfer_paths; qq++)
					{
						passenger_board_indicator += transfer_boarding[qq][t];
					}
				}
				//indicator=0;
				//while(indicator<t_dwell&&t-i*(t_dwell+t_running)-indicator>=0)
				//{
				//	train_arrive_indicator += x_lt[l][t-i*(t_dwell+t_running)-indicator];
				//	indicator++;
				//}
				if (t - i*(t_dwell + t_running) >= 0)
				{
					constraints.add(passenger_board_indicator - TRAIN_CAPACITY*x_lt[l][t - i*(t_dwell + t_running)] <= 0);
					constraints.add(passenger_board_indicator + in_vehicle_passengers <= TRAIN_CAPACITY);
				}
				else
				{
					constraints.add(passenger_board_indicator == 0);
				}
				// constraints.add(passenger_board_indicator==0);
			}
		}
	}
	// C3
	for (int l = 0; l<2; l++)
	{
		for (int t = 0; t<_MAX_TIME; t++)
		{
			IloExpr trains_num(env);
			int tau = t;
			while (tau<_MAX_TIME && tau<t + h_min)
			{
				trains_num += x_lt[l][tau];
				tau++;
			}
			constraints.add(trains_num <= 1);
		}
		for (int t = 0; t<_MAX_TIME; t++)
		{
			if (t + t_cycle<_MAX_TIME)
			{
				IloExpr DepartTrains(env);
				for (int tau = t; tau<t + t_cycle; tau++)
				{
					DepartTrains += x_lt[l][tau];
				}
				constraints.add(DepartTrains <= rolling_stock[l]);
			}
		}
	}
	// C4
	for (int p = 0; p<g_path_vector.size(); p++)
	{
		for (int t = 0; t<_MAX_TIME; t++)
		{
			IloExpr b_board_now(env);
			int tau = 0;
			while (tau <= t)
			{
				b_board_now += p_pt[p][tau];
				tau++;
			}
			constraints.add(b_board_now <= g_demand_path_current[p][t]);
			//constraints.add(b_board_now<=20000);
		}
	}
	for (int t = 0; t<_MAX_TIME; t++)
	{
		IloExpr b_board_now(env);
		IloExpr b_arrive_now(env);
		int tau = 0;
		while (tau <= t)
		{
			for (int qq = 0; qq<number_of_transfer_paths; qq++) // A set of constraints for each path
			{

				if (tau - t_r[qq] >= 0)
				{
					b_arrive_now += p_pt[g_path_vector.size() - number_of_transfer_paths + qq][tau - t_r[qq]];
				}
				b_board_now += transfer_boarding[qq][tau];
				tau++;
			}
			constraints.add(b_board_now - b_arrive_now <= 0);
		}
	}
	// Train capacity constraints

	model.add(constraints);
	//IloExpr expr1(env);
	//for(int t=0;t<_MAX_TIME;t++)
	//{
	//	expr1 += transfer_boarding[t];
	//}
	//IloExpr expr(env);
	//for(int l=0;l<2;l++)
	//	for(int t=0;t<_MAX_TIME;t++)
	//		expr += (t+1)*x_lt[l][t];
	//for(int l=0;l<2;l++)
	//	for(int i=0;i<_MAX_ZONE;i++)
	//		for(int t=0;t<_MAX_TIME;t++)
	//			expr += (_MAX_TIME-t)*n_lit[l][i][t];
	//IloObjective model_obj = IloMaximize (env, expr1);
	IloObjective model_obj = IloMinimize(env, max_waiting);
	model.add(model_obj);
	IloCplex newcc(model);
	//	newcc.setOut(env.getNullStream());
	//newcc.setAnyProperty(IloCplex::Param::Benders::Strategy);
	//newcc.setParam(IloCplex::Param::TimeLimit, 3600);
	newcc.use(boundlimitCallback(env, newcc, IloFalse, newcc.getBestObjValue(), 1.0, 10000.0));
	newcc.solve();
	cout << "Solution status = " << newcc.getCplexStatus() << endl;
	//if (newcc.getCplexStatus() == CPX_STAT_OPTIMAL || newcc.getCplexStatus() == CPXMIP_OPTIMAL_TOL)
	{
		//cout << "Objective value =" << newcc.getObjValue() << endl;
		int out = 0;
		for (int l = 0; l<2; l++)
		{
			cout << "Line " << l << " : ";
			for (int t = 0; t<_MAX_TIME; t++)
			{
				best_solution.x_lt[l][t] = newcc.getIntValue(x_lt[l][t]);
				//best_solution.x_lt[l][t] = newcc.getValue(x_lt[l][t]);
				//cout << newcc.getValue(x_lt[l][t]) << ", ";
				if (newcc.getValue(x_lt[l][t]) == 1)
				{
					cout << t << ", ";
				}
			}
			cout << endl;
		}
		cout << "12345" << endl;
		cout << "%%%%%%%%%%%" << endl;
		float total_passenger = 0;
		for (int i = 0; i<g_path_vector.size(); i++)
		{
			//cout<<"Path "<<i<<":";
			for (int t = 0; t<_MAX_TIME; t++)
			{
				total_passenger += newcc.getValue(p_pt[i][t]);
				cplex_boarding[i][t] = newcc.getValue(p_pt[i][t]);
			}
		}
		cout << "The total demand is " << total_passenger << endl;
		for (int t = 0; t<_MAX_TIME; t++)
		{
			for (int l = 0; l<2; l++)
				for (int i = 0; i<_MAX_ZONE; i++)
					g_demand_waiting[i][l][t] = newcc.getValue(n_lit[l][i][t]);
		}
		cout << "Transfer passengers" << endl;
		for (int t = 0; t<_MAX_TIME; t++)
		{
			cout << newcc.getValue(transfer_boarding[0][t]) << ", ";
		}
	}
}

float cost_calculation(SOLUTION & solution)
{
	float cost = 0;
	//SOLUTION solution_now = solution;
	int t_dwell = 1;
	int t_running = 4;
	int t_transfer = 2;
	int t_r[2] = {0};
	for (int i = 0; i<number_of_transfer_paths; i++)
	{
		int p_order = g_path_vector.size() - number_of_transfer_paths + i;
		if (g_path_vector[p_order].column_node_vector.size() <= 2)
		{
			cout << "Error happens" << endl;
			//abort();
		}
		else
		{
			int segment_traveled = 0;
			int station_dwelled = 0;
			segment_traveled = g_path_vector[p_order].column_node_vector[1] - g_path_vector[p_order].column_node_vector[0];
			station_dwelled = segment_traveled - 1;
			t_r[i] = t_transfer + segment_traveled*t_running + t_dwell*station_dwelled;
		}

	}
	//for (int i = 0; i<number_of_transfer_paths; i++)
	//{
	//	cout << "Travelling time of path " << i + g_path_vector.size() - number_of_transfer_paths << " = " << t_r[i] << endl;
	//}
	// Define variables
	IloEnv env;
	IloModel model(env);
	NumVar3Matrix n_lit(env, 2);
	for (int i = 0; i<2; i++)
	{
		n_lit[i] = NumVarMatrix(env, _MAX_ZONE);
		for (int j = 0; j<_MAX_ZONE; j++)
		{
			n_lit[i][j] = IloNumVarArray(env, _MAX_TIME);
			for (int t = 0; t<_MAX_TIME; t++)
			{
				n_lit[i][j][t] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
			}
		}
	}
	NumVarMatrix p_pt(env, g_path_vector.size());
	for (int i = 0; i<g_path_vector.size(); i++)
	{
		p_pt[i] = IloNumVarArray(env, _MAX_TIME);
		for (int t = 0; t<_MAX_TIME; t++)
		{
			p_pt[i][t] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
		}
	}
	NumVarMatrix transfer_boarding(env, 2);
	for (int i = 0; i<number_of_transfer_paths; i++)
	{
		transfer_boarding[i] = IloNumVarArray(env, _MAX_TIME);
		for (int t = 0; t<_MAX_TIME; t++)
		{
			transfer_boarding[i][t] = IloNumVar(env, 0, IloInfinity, ILOFLOAT);
		}
	}
	IloNumVar max_waiting(env, 0, IloInfinity, ILOFLOAT);
	IloRangeArray constraints(env);

	//C1
	for (int i = 0; i<2; i++)
	{
		for (int j = 0; j<_MAX_ZONE; j++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				IloExpr station_waiting(env);
				for (int tau = 0; tau<t; tau++)
				{
					// Arriving passengers and Boarding passengers
					station_waiting += g_demand_array_line_arriving[j][i][tau];
					if (i == 1 && j == 1)
					{
						for (int qq = 0; qq<number_of_transfer_paths; qq++)
						{
							station_waiting -= transfer_boarding[qq][tau];
						}
					}
					int path_id = 0;
					while (path_id<g_path_vector.size())
					{
						if (g_path_vector[path_id].column_node_vector[0] == j && g_path_vector[path_id].column_node_line_vector[0] == i)
						{
							station_waiting -= p_pt[path_id][tau];
						}
						else if (g_path_vector[path_id].column_node_vector.size()>2) // this path has transfers
						{
							for (int q = 1; q<g_path_vector[path_id].column_node_vector.size() / 2; q++)// Arrive at the current line
							{
								if (g_path_vector[path_id].column_node_vector[2 * q] == j && g_path_vector[path_id].column_node_line_vector[2 * q] == i)
								{
									if (tau - t_r[path_id-40] >= 0)
									{
										//cout << path_id <<" "<<","<<t<<";"<< tau - t_r[path_id - 40]<< endl;
										//station_waiting += 100000000* p_pt[path_id][tau - t_r[path_id - 40]];
										station_waiting += p_pt[path_id][tau - t_r[path_id - 40]];
									}
								}
							}
						}
						path_id++;
					}

				}
				constraints.add(max_waiting - station_waiting >= 0);
				constraints.add(n_lit[i][j][t] - station_waiting == 0);
			}
		}
	}

	// C2
	for (int l = 0; l<2; l++)
	{
		for (int i = 0; i<_MAX_ZONE; i++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				IloExpr in_vehicle_passengers(env); // Number of passengers that arrive at time t and station i
				IloExpr train_arrive_indicator(env);
				IloExpr passenger_board_indicator(env);
				int indicator = 0;
				if (t - i*(t_dwell + t_running) >= 0)
				{
					for (int p = 0; p<g_path_vector.size(); p++)
					{
						if (g_path_vector[p].column_node_line_vector[0] == l&&
							g_path_vector[p].column_node_seq[0]<i&&
							g_path_vector[p].column_node_seq[1] >= i)
						{
							in_vehicle_passengers += p_pt[p][t - (t_dwell + t_running)*(i - g_path_vector[p].column_node_seq[0])];
						}

					}
					for (int p = g_path_vector.size() - number_of_transfer_paths; p<g_path_vector.size(); p++)
					{
						//if(l==1 && g_path_vector[p].column_node_vector[g_path_vector[p].column_node_vector.size()-1]>=i
						//	//&&t-(i-g_path_vector[p].column_node_seq[g_path_vector[p].column_node_seq.size()-2])*((t_dwell+t_running))>=0) //
						//		&&t-(i-1)*((t_dwell+t_running))>=0) //
						if (l == 1 && i >= 1 && t - (i - 1)*((t_dwell + t_running)) >= 0) //
						{
							in_vehicle_passengers += transfer_boarding[p - g_path_vector.size() + number_of_transfer_paths][t - (i - 1)*((t_dwell + t_running))];
						}
					}
				}
				while (indicator<g_path_vector.size())
				{
					if (g_path_vector[indicator].column_node_line_vector[0] == l&&
						g_path_vector[indicator].column_node_vector[0] == i)
					{
						passenger_board_indicator += p_pt[indicator][t];
					}
					indicator++;
				}
				// Transfer passengers
				if (l == 1 && i == 1)
				{
					for (int qq = 0; qq<number_of_transfer_paths; qq++)
					{
						passenger_board_indicator += transfer_boarding[qq][t];
					}
				}
				//indicator=0;
				//while(indicator<t_dwell&&t-i*(t_dwell+t_running)-indicator>=0)
				//{
				//	train_arrive_indicator += x_lt[l][t-i*(t_dwell+t_running)-indicator];
				//	indicator++;
				//}
				if (t - i*(t_dwell + t_running) >= 0)
				{
					constraints.add(passenger_board_indicator - TRAIN_CAPACITY*solution.x_lt[l][t - i*(t_dwell + t_running)] <= 0);
					constraints.add(passenger_board_indicator + in_vehicle_passengers <= TRAIN_CAPACITY);
				}
				else
				{
					constraints.add(passenger_board_indicator == 0);
				}
				// constraints.add(passenger_board_indicator==0);
			}
		}
	}
	// C4
	for (int p = 0; p<g_path_vector.size(); p++)
	{
		for (int t = 0; t<_MAX_TIME; t++)
		{
			IloExpr b_board_now(env);
			int tau = 0;
			while (tau <= t)
			{
				b_board_now += p_pt[p][tau];
				tau++;
			}
			constraints.add(b_board_now <= g_demand_path_current[p][t]);
			//constraints.add(b_board_now<=20000);
		}
	}
	for (int t = 0; t<_MAX_TIME; t++)
	{
		IloExpr b_board_now(env);
		IloExpr b_arrive_now(env);
		int tau = 0;
		while (tau <= t)
		{
			for (int qq = 0; qq<number_of_transfer_paths; qq++) // A set of constraints for each path
			{

				if (tau - t_r[qq] >= 0)
				{
					b_arrive_now += p_pt[g_path_vector.size() - number_of_transfer_paths + qq][tau - t_r[qq]];
				}
				b_board_now += transfer_boarding[qq][tau];
				tau++;
			}
			constraints.add(b_board_now - b_arrive_now <= 0);
		}
	}
	// Train capacity constraints

	model.add(constraints);
	//IloExpr expr1(env);
	//for(int t=0;t<_MAX_TIME;t++)
	//{
	//	expr1 += transfer_boarding[t];
	//}
	//IloExpr expr(env);
	//for(int l=0;l<2;l++)
	//	for(int t=0;t<_MAX_TIME;t++)
	//		expr += (t+1)*x_lt[l][t];
	//for(int l=0;l<2;l++)
	//	for(int i=0;i<_MAX_ZONE;i++)
	//		for(int t=0;t<_MAX_TIME;t++)
	//			expr += (_MAX_TIME-t)*n_lit[l][i][t];
	//IloObjective model_obj = IloMaximize (env, expr1);
	IloObjective model_obj = IloMinimize(env, max_waiting);
	model.add(model_obj);
	IloCplex newcc(model);
	newcc.setOut(env.getNullStream());
	newcc.use(boundlimitCallback(env, newcc, IloFalse, newcc.getBestObjValue(), 1.0, 10.0));
	//newcc.setAnyProperty(IloCplex::Param::Benders::Strategy);
	//newcc.setParam(IloCplex::Param::TimeLimit, 3000);
	newcc.solve();
	//newcc.solve();
	cout<<"Solution status = "<<newcc.getCplexStatus()<<endl;
	if (newcc.getCplexStatus() == CPX_STAT_OPTIMAL || newcc.getCplexStatus() == CPXMIP_OPTIMAL_TOL)
	{
		cost = newcc.getObjValue();
		cout<<"Objective value ="<<newcc.getObjValue()<<endl;
	}
	else
	{
		cout << "ERROR OCCURED!" << endl;
	}
	//delete[]t_r;
	float total_passenger = 0;
	for (int i = 0; i<g_path_vector.size(); i++)
	{
		for (int t = 0; t<_MAX_TIME; t++)
		{
			total_passenger += newcc.getValue(p_pt[i][t]);
			cplex_boarding[i][t] = newcc.getValue(p_pt[i][t]);
		}
	}
	for (int t = 0; t<_MAX_TIME; t++)
	{
		for (int l = 0; l<2; l++)
			for (int i = 0; i<_MAX_ZONE; i++)
				g_demand_waiting[i][l][t] = newcc.getValue(n_lit[l][i][t]);
	}
	env.end();
	//newcc.clear();
	return cost;
}

bool feasibility_check(SOLUTION & solution)
{
	bool feasible = true;
	for (int i = 0; i<number_of_lines; i++)
	{
		for (int t = 0; t + h_min<_MAX_TIME; t++)
		{
			int check_headway = 0;
			for (int tau = t; tau<t + h_min; tau++)
			{
				check_headway += solution.x_lt[i][tau];
			}
			if (check_headway>1)
			{
				return false;
			}
		}
		for (int t = 0; t<_MAX_TIME; t++)
		{
			int check_rolling_stock = 0;
			if (t + t_cycle<_MAX_TIME)
			{
				for (int tau = t; tau<t + t_cycle; tau++)
				{

					check_rolling_stock += solution.x_lt[i][tau];
				}
			}

			if (check_rolling_stock > rolling_stock[i])
			{
				return false;
			}
		}
	}
	return feasible;
}

void initialization(void)
{
	timetable *tb = new timetable();
	int service_num = g_number_of_services;
	int origin_time = 0;
	int headway = 20;
	for (int i = 0; i<service_num; i++)
	{
		for (int j = 0; j<g_number_of_nodes; j++)
		{
			if (j == 0)
			{
				tb->Arrive[j] = origin_time;
			}
			else
			{
				tb->Arrive[j] = tb->Depart[j - 1] + 10;
			}
			tb->Depart[j] = tb->Arrive[j] + 3;
		}
		origin_time += headway;
		planned_timetable.push_back(*tb);
	}
	for (int i = 0; i<g_node_vector.size(); i++)
	{
		for (int j = 0; j<g_number_of_nodes; j++)
		{
			if (j>i)
				g_node_vector[i].passenger_waiting.push_back(g_demand_array[i][j][0]);
			else
				g_node_vector[i].passenger_waiting.push_back(0);
		}
	}
	for (int i = 0; i<g_number_of_services; i++)
		for (int j = 0; j<g_number_of_nodes; j++)
			for (int t = 0; t<_MAX_TIME; t++)
				r_vehicle_data[i][j][t] = 0;
	for (int i = 0; i<g_number_of_nodes; i++)
		for (int j = 0; j<g_number_of_nodes; j++)
			for (int t = 0; t<_MAX_TIME; t++)
			{
				r_station_data[i][j][t] = 0; r_station_data[i][j][0] = g_node_vector[i].passenger_waiting[j];
			}
}

void train_simulater(int k, int t)
{
	if (t<planned_timetable[k].Arrive[0] || t>planned_timetable[k].Depart[g_number_of_nodes - 1])
	{
		Trains[k].depot = true;
	}
	else
	{
		Trains[k].depot = false;
		for (int i = 0; i<g_number_of_nodes; i++)
		{
			Trains[k].dwell = false;
			Trains[k].running = false;
			if (t>planned_timetable[k].Arrive[i] && t <= planned_timetable[k].Depart[i])
			{
				Trains[k].dwell = true;
				Trains[k].dwell_station = i;
				break;
				// cout<<"Dwell "<<i<<endl;
			}
			else if (i != g_number_of_nodes - 1)
			{
				if (t <= planned_timetable[k].Arrive[i + 1] && t>planned_timetable[k].Depart[i])
				{
					Trains[k].running = true;
					Trains[k].running_link = i;
					break;
				}
			}
		}

	}
}

void simulation_process()
{
	//Step 2 Update the running states of trains
	for (int i = 0; i<planned_timetable.size(); i++)
	{
		train *tb = new train();
		Trains.push_back(*tb);
	}
	cout << "Number of passengers in vehicle " << 0 << " =: ";
	int cc = 0;
	for (int t = 1; t<_MAX_TIME; t++)
	{
		for (int i = 0; i<g_number_of_nodes; i++)
		{
			for (int j = 0; j<g_number_of_nodes; j++)
			{
				g_boarding[i][j] = 0.0;
			}
		}
		for (int k = 0; k<planned_timetable.size(); k++)
		{
			//cout<<accumulate(Trains[k].passenger_num.begin(), Trains[k].passenger_num.end(),static_cast<float>(0.0))<<", ";
			// Update the train state
			train_simulater(k, t);
			// Update the passenger state
			if (Trains[k].depot == false)
			{
				if (Trains[k].dwell == true)
				{
					// Passenger alighting
					Trains[k].passenger_num[Trains[k].dwell_station] = 0;
					// Passenger boarding
					float remaining_capacity = TRAIN_CAPACITY - accumulate(Trains[k].passenger_num.begin(), Trains[k].passenger_num.end(), static_cast<float>(0.0));
					//					cout<<"Remaining_capacity = "<<(int)remaining_capacity<<endl;
					float station_waiting = accumulate(g_node_vector[Trains[k].dwell_station].passenger_waiting.begin(),
						g_node_vector[Trains[k].dwell_station].passenger_waiting.end(), static_cast<float>(0.0));
					if (station_waiting <= remaining_capacity)
					{
						for (int j = Trains[k].dwell_station; j<g_number_of_nodes; j++)
						{
							g_boarding[Trains[k].dwell_station][j] = g_node_vector[Trains[k].dwell_station].passenger_waiting[j];
							Trains[k].passenger_num[j] += g_boarding[Trains[k].dwell_station][j];
						}
					}
					else
					{
						for (int j = Trains[k].dwell_station; j<g_number_of_nodes; j++)
						{
							g_boarding[Trains[k].dwell_station][j] = remaining_capacity*g_node_vector[Trains[k].dwell_station].passenger_waiting[j] / station_waiting;
							Trains[k].passenger_num[j] += g_boarding[Trains[k].dwell_station][j];
						}
					}
					// Update the passenger flow
				}
			}
			// Save data
			for (int i = 0; i<g_number_of_nodes; i++)
			{
				r_vehicle_data[k][i][t] = Trains[k].passenger_num[i];
			}
		}
		// Update the passengers at stations
		// cout<<"QQ "<<r_vehicle_data[0][1][2]<<endl;
		for (int i = 0; i<g_number_of_nodes; i++)
		{
			for (int j = 0; j<g_number_of_nodes; j++)
			{
				g_node_vector[i].passenger_waiting[j] = g_node_vector[i].passenger_waiting[j] - g_boarding[i][j] + g_demand_array[i][j][t];
				r_station_data[i][j][t] = g_node_vector[i].passenger_waiting[j]; //Save data
			}
		}
		// Update the passenger flow of station
	}
	cout << "Tese here = " << cc << "; Test another " << r_vehicle_data[0][1][2] << endl;
}

void g_SaveOutputData(void)
{
	FILE* g_pFileTrainPAX = NULL;
	g_pFileTrainPAX = fopen("output_TrainPAX.csv", "w");
	if (g_pFileTrainPAX == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(g_pFileTrainPAX, "Time And Train ID\n");
		for (int t = 0; t<_MAX_TIME; t++)
		{
			for (int i = 0; i<g_number_of_services; i++)
			{
				float in_vehicle_passenger = 0;
				for (int j = 0; j<g_number_of_nodes; j++)
				{
					in_vehicle_passenger += r_vehicle_data[i][j][t];
					if (i == 0)
					{
						cout << ": " << r_vehicle_data[i][j][t] << " ";
					}
				}
				fprintf(g_pFileTrainPAX, "%.3f,", in_vehicle_passenger);
			}
			cout << endl;
			fprintf(g_pFileTrainPAX, "\n");
		}
	}
	fclose(g_pFileTrainPAX);

	FILE* g_pFileStationPAX = NULL;
	g_pFileStationPAX = fopen("output_StationPAX.csv", "w");
	if (g_pFileStationPAX == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(g_pFileStationPAX, "Time And Station ID\n");
		for (int t = 0; t<_MAX_TIME; t++)
		{
			for (int i = 0; i<g_number_of_nodes; i++)
			{
				float station_passenger = 0;
				for (int j = 0; j<g_number_of_nodes; j++)
				{
					station_passenger += r_station_data[i][j][t];
				}
				fprintf(g_pFileStationPAX, "%.3f,", station_passenger);
			}
			fprintf(g_pFileStationPAX, "\n");
		}
	}
	fclose(g_pFileStationPAX);
}

void g_SaveOutputData_CPLEX(void)
{
	// Generated data
	FILE* g_pFilePAX_PATH = NULL;
	g_pFilePAX_PATH = fopen("output_pax_path.csv", "w");
	if (g_pFilePAX_PATH == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(g_pFilePAX_PATH, "Time And Path ID\n");
		for (int t = 0; t<_MAX_TIME; t++)
		{
			for (int i = 0; i<g_path_vector.size(); i++)
			{
				fprintf(g_pFilePAX_PATH, "%.3f,", g_demand_path_current[i][t]);
			}
			fprintf(g_pFilePAX_PATH, "\n");
		}
	}
	fclose(g_pFilePAX_PATH);
	// Boarding data
	FILE* g_pFileBoard_PATH = NULL;
	g_pFileBoard_PATH = fopen("output_board_path.csv", "w");
	if (g_pFileBoard_PATH == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(g_pFileBoard_PATH, "Time And Path ID\n");
		for (int t = 0; t<_MAX_TIME; t++)
		{
			for (int i = 0; i<g_path_vector.size(); i++)
			{
				fprintf(g_pFileBoard_PATH, "%.3f,", cplex_boarding[i][t]);
			}
			fprintf(g_pFileBoard_PATH, "\n");
		}
	}
	fclose(g_pFileBoard_PATH);
	FILE* g_pFilePAX_STATION = NULL;
	g_pFilePAX_STATION = fopen("output_StationPAX.csv", "w");
	if (g_pFilePAX_STATION == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(g_pFilePAX_STATION, "Time And Station ID\n");
		fprintf(g_pFilePAX_STATION, "line,");
		for (int i = 0; i<_MAX_ZONE; i++)
		{
			fprintf(g_pFilePAX_STATION, "%d,", i);
		}
		fprintf(g_pFilePAX_STATION, "\n");
		for (int l = 0; l<2; l++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				fprintf(g_pFilePAX_STATION, "%d,", l);
				for (int i = 0; i<_MAX_ZONE; i++)
				{
					fprintf(g_pFilePAX_STATION, "%.3f,", g_demand_array_line_arriving[i][l][t]);
				}
				fprintf(g_pFilePAX_STATION, "\n");
			}
		}
	}
	fclose(g_pFilePAX_STATION);
	// Waiting passengers at stations
	FILE* g_pFileWAIT_STATION = NULL;
	g_pFileWAIT_STATION = fopen("output_StationWAIT.csv", "w");
	if (g_pFileWAIT_STATION == NULL)
	{
		cout << "File output_LinkMOE.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(g_pFileWAIT_STATION, "Time And Station ID\n");
		fprintf(g_pFileWAIT_STATION, "line,");
		for (int i = 0; i<_MAX_ZONE; i++)
		{
			fprintf(g_pFileWAIT_STATION, "%d,", i);
		}
		fprintf(g_pFileWAIT_STATION, "\n");
		for (int l = 0; l<2; l++)
		{
			for (int t = 0; t<_MAX_TIME; t++)
			{
				fprintf(g_pFileWAIT_STATION, "%d,", l);
				for (int i = 0; i<_MAX_ZONE; i++)
				{
					fprintf(g_pFileWAIT_STATION, "%.3f,", g_demand_waiting[i][l][t]);
				}
				fprintf(g_pFileWAIT_STATION, "\n");
			}
		}
	}
	fclose(g_pFileWAIT_STATION);
	// Timetable
	FILE* timetable = NULL;
	timetable = fopen("timetable.csv", "w");
	if (timetable == NULL)
	{
		cout << "File timetable.csv cannot be opened." << endl;
	}
	else
	{
		fprintf(timetable, "Time And Line ID\n");
		for (int t = 0; t<_MAX_TIME; t++)
		{
			for (int i = 0; i<number_of_lines; i++)
			{
				fprintf(timetable, "%d,", best_solution.x_lt[i][t]);
			}
			fprintf(timetable, "\n");
		}
	}
	fclose(timetable);
}

int _tmain()
{
	srand((unsigned)time(NULL));
	//planned_timetable.
	
	startTime = clock();
	g_ReadInputData();  // step 1: read input data of network and demand table/agents 
						// to do: read demand tables
	initialization();
	cout << "Test " << g_path_vector.size() << endl;
	passenger_demand_initilization();
	// VLNS
	for (int i = 0; i<number_of_lines; i++)
	{
		for (int t = 0; t<_MAX_TIME; t++)
		{
			int cc = 0;
			if (i == 0)
			{
				cc = 7;
			}
			else
			{
				cc = 8;
			}
			if ((t) % cc == 0)
			{
				best_solution.x_lt[i][t] = 1;
			}
			else
			{
				best_solution.x_lt[i][t] = 0;
			}
		}
	}
	
	if (feasibility_check(best_solution) == true)
	{
	// vnls();
	}
	else
	{
		cout << "\t\tThe initial solution is infeasible! Termininated" << endl;
	}
	new_cplex_model();
	if (feasibility_check(best_solution) == true)
	{
		cout << "Feasible!" << endl;
	}
	cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
	cout << "The optimal value is: " << cost_calculation(best_solution) << endl;
	int service_num[2] = { 0 };
	for (int i = 0; i<2; i++)
	{
		cout << "Line " << i << " :";
		for (int t = 0; t<_MAX_TIME; t++)
		{
			if (best_solution.x_lt[i][t] == 1)
			{
				service_num[i] += 1;
				cout << t << ", ";
			}
		}
		cout << endl;
	}
	endTime = clock();
	cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
	cout << "Services of Line 1 = " << service_num[0] << endl;
	cout << "Services of Line 2 = " << service_num[1] << endl;
	//for(int i=0;i<number_of_lines;i++)
	//{
	//	for(int t=0;t<_MAX_TIME;t++)
	//	{
	//		if(best_solution.x_lt[i][t]!=test_solution.x_lt[i][t])
	//		{
	//			cout<<"error"<<endl;
	//		}
	//	}
	//}
	//cout<<"Feasible3 "<<feasibility_check(test_solution)<<endl;
	g_SaveOutputData_CPLEX();
	// delete[] g_pNetworkForSP;
	// best_solution.cost = cost_calculation();
	exit(0);
}

