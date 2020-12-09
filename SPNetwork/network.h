#pragma once

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <list> 
#include <algorithm>
#include <time.h>
#include <functional>
#include<stdio.h>   
#include<math.h>
#include <numeric>

#define _MAX_ZONE 10
#define _MAX_TIME 99999
#define _MAX_LABEL_COST 999999


extern int g_number_of_links;
extern int g_number_of_nodes;
extern int g_number_of_services;

class CLink;
class CNode
{
public:
	vector<float> passenger_waiting;
	float passenger_waiting_num;
	CNode()
	{
		passenger_waiting = vector<float>();
		passenger_waiting_num = accumulate(passenger_waiting.begin(), passenger_waiting.end(), 0);
		zone_id = -1;
		accessible_node_count = 0;
	}

	int accessible_node_count;

	int node_seq_no;  // sequence number 
	int node_id;      //external node number 
	int zone_id;
	double x;
	double y;
	std::vector<CLink> m_outgoing_link_vector;
	std::vector<int> m_incoming_link_seq_no_vector;

};
class CLink
{
public:
	CLink()  // construction 
	{
		cost = 0;
		BRP_alpha = 0.15f;
		BRP_beta = 4.0f;
		link_capacity = 1000;
		free_flow_travel_time_in_min = 1;
		flow_volume = 0;
		link_spatial_capacity = 100;
		/*
		CFlowArrivalCount = 0;
		CFlowDepartureCount = 0;

		LinkOutCapacity = 0;
		LinkLeftOutCapacity = 0;
		LinkInCapacity = 0;
		m_LeftTurn_NumberOfLanes = 0;
		m_LeftTurn_link_seq_no = -1;
		m_OutflowNumLanes = 1;*/
	}

	~CLink()
	{
		//if (flow_volume_for_each_o != NULL)
		//	delete flow_volume_for_each_o;
	}

	void free_memory()
	{
	}

	void AddAgentsToLinkVolume()
	{


	}
	void tally_flow_volume_across_all_processors();

	float get_time_dependent_link_travel_time(int node_time_stamp_in_min)
	{
		int time_interval_no = node_time_stamp_in_min / 15;

		if (time_interval_no < m_avg_travel_time.size())
			return m_avg_travel_time[time_interval_no];
		else
			return cost;

	}

	std::vector<int> m_total_volume_vector;
	std::vector<float> m_avg_travel_time;

	// 1. based on BPR. 


	int m_LeftTurn_link_seq_no;

	int m_RandomSeed;
	int link_seq_no;
	int from_node_seq_no;
	int to_node_seq_no;
	float cost;

	float fftt;
	float free_flow_travel_time_in_min;
	int type;
	float link_capacity;
	float link_spatial_capacity;

	float flow_volume;


	float travel_time;
	float BRP_alpha;
	float BRP_beta;
	float length;
	void statistics_collection(int time_stamp_in_min);
	/// end of simulation data


	// signal data

	bool m_bSignalizedArterialType;

	int m_current_plan_no;
	int m_number_of_plans;

	int m_plan_startime_in_sec[20];
	int m_plan_endtime_in_sec[20];
	int m_to_node_cycle_in_sec[20];
	int m_to_node_offset_in_sec[20];

	int number_of_timing_plans;
	float m_SaturationFlowRate_In_vhc_per_hour_per_lane[20];
	int m_GreenStartTime_In_Second[20];
	int m_GreenEndTime_In_Second[20];
	float m_LeftTurn_SaturationFlowRate_In_vhc_per_hour_per_lane[20];
	int m_LeftTurnGreenStartTime_In_Second[20];
	int m_LeftTurnGreenEndTime_In_Second[20];
	void UpdateCellVehiclePosition(int current_time_interval, int cell_no, int agent_id);
	/*
	void SetupCell(int number_of_cells)
	{
	for (int i = 0; i < number_of_cells; i++)
	{
	m_cell_current_t.push_back(-1);
	m_cell_prev_t_1.push_back(-1);

	}
	}
	void CopyCell()
	{
	// copy cell states
	for (int c = 0; c < m_cell_current_t.size(); c++)
	{
	m_cell_prev_t_1[c] = m_cell_current_t[c];
	m_cell_current_t[c] = -1;

	}

	}*/

	float GetTimeDependentCapacityAtSignalizedIntersection(
		double current_timeInMin,
		int &current_plan_no,
		int plan_startime_in_sec[20],
		int plan_endtime_in_sec[20],
		int to_node_cycle_in_sec[20],
		int to_node_offset_in_sec[20],
		float SaturationFlowRate[20],
		int GreenStartTime_in_second[20],
		int GreenEndTime_in_second[20],
		float simulation_time_interval_in_second);
	//

	float get_VOC_ratio()
	{
		return flow_volume / max((float)0.00001, link_capacity);

	}

	float get_speed()
	{
		return length / max(travel_time, (float)0.0001) * 60;  // per hour
	}

	/*float GetRandomRatio()
	{
	m_RandomSeed = (LCG_a * m_RandomSeed + LCG_c) % LCG_M;  //m_RandomSeed is automatically updated.

	return float(m_RandomSeed) / LCG_M;
	}
	std::vector<int> m_cell_current_t;
	std::vector<int> m_cell_prev_t_1;*/

};
class CPath
{
public:
	int path_id;
	int column_cost;
	float column_flow;
	string served_pax_group;
	std::vector<int> served_pax_group_vector;
	string column_node_seq;
	string column_node_time_seq;
	std::vector<int> column_node_vector;
	std::vector<int> column_node_line_vector;
	//std::vector<int> column_node_arc_vector;
	//std::vector<string> column_node_time_arc_vector;

	CPath()
	{
		path_id = 0;
		column_cost = 0.0;
		column_flow = 0.0;
	}
};
extern std::vector <CPath> g_path_vector;

class timetable
{
public:
	int Trainindex;
	vector<float> Arrive;
	vector<float> Depart;
	vector<string> PassStations;
	timetable()
	{
		Arrive = vector<float>(g_number_of_nodes);
		Depart = vector<float>(g_number_of_nodes);
	}
	//哪辆列车在哪个车站的到达和出发时间
	// FLOAT TIMETABLE.TRAIN.STATION.DEPARTURE;
	// FLPAT TIMETABLE.TRAIN.STATION.DEPARTURE;
};
class train
{
public:
	bool depot;
	bool dwell;
	int dwell_station;
	bool running;
	int running_link;
	//float sum_passenger;
	vector<float> passenger_num;
	train()
	{
		depot = false;
		dwell = false;
		running = false;
		passenger_num = vector<float>(g_number_of_nodes);
		//sum_passenger = accumulate(passenger_num.begin(), passenger_num.end(), 0);
	}
	// 
};
class simulation
{
public:
	vector<int> station_flow;
	vector<int> service_flow;

};


extern std::vector<CNode> g_node_vector;
extern std::vector<CLink> g_link_vector;
extern std::vector<timetable> planned_timetable;
extern std::vector<train> Trains;
// extern vector<CAgent> g_agent_vector;
extern std::map<int, CLink*> g_pLink_map;  // link seq no


class NetworkForSP  // mainly for shortest path calculation
{
public:
	NetworkForSP()
	{
		pFileAgentPathLog = NULL;
	}

	int m_threadNo;  // internal thread number 

	int m_ListFront;
	int m_ListTail;

	int* m_SENodeList;

	//	std::list<int>  m_SENodeList;  //scan eligible list as part of label correcting algorithm 

	float* m_node_label_cost;  // label cost // for shortest path calcuating 
	float* m_node_label_generalized_time;  // label time // for shortest path calcuating 

	int* m_node_predecessor;  // predecessor for nodes
	int* m_node_status_array; // update status 

	int* m_link_predecessor;  // predecessor for this node points to the previous link that updates its label cost (as part of optimality condition) (for easy referencing)

	FILE* pFileAgentPathLog;  // file output

	float* m_link_volume_array; // link volume for all agents assigned in this network (thread)
	float* m_link_cost_array; // link cost 


	std::vector<int>  m_node_vector; // assigned nodes for computing 
									 /*std::vector<CNode2NodeAccessibility>  m_node2node_accessibility_vector;*/

	void AllocateMemory(int number_of_nodes, int number_of_links)
	{

		m_SENodeList = new int[number_of_nodes];

		m_node_predecessor = new int[number_of_nodes];
		m_node_status_array = new int[number_of_nodes];
		m_node_label_cost = new float[number_of_nodes];

		m_node_label_generalized_time = new float[number_of_nodes];
		m_link_predecessor = new int[number_of_nodes];   // note that, the size is still the number of nodes, as each node has only one link predecessor

														 //char buffer[256];
														 //sprintf_s(buffer, "%s_%d.csv", "agent_path", m_threadNo);

														 //pFileAgentPathLog = fopen(buffer, "w");

		m_link_volume_array = new float[number_of_links];

		m_link_cost_array = new float[number_of_links];

		for (int l = 0; l < number_of_links; l++)
		{
			m_link_volume_array[l] = 0.0;
			m_link_cost_array[l] = 1.0; //default value
		}
	}

	~NetworkForSP()
	{
		if (m_SENodeList != NULL)
			delete m_SENodeList;
		if (m_node_label_cost != NULL)
			delete m_node_label_cost;

		if (m_node_predecessor != NULL)
			delete m_node_predecessor;

		if (m_node_status_array != NULL)
			delete m_node_status_array;

		if (m_link_predecessor != NULL)
			delete m_link_predecessor;

		if (m_link_volume_array != NULL)
			delete m_link_volume_array;

		if (m_link_cost_array != NULL)
			delete m_link_cost_array;

		if (pFileAgentPathLog != NULL)
			fclose(pFileAgentPathLog);
	}


	// SEList: scan eligible List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_ListFront = -1;
		m_ListTail = -1;
	}

	void SEList_push_front(int node)
	{
		if (m_ListFront == -1)  // start from empty
		{
			m_SENodeList[node] = -1;
			m_ListFront = node;
			m_ListTail = node;
		}
		else
		{
			m_SENodeList[node] = m_ListFront;
			m_ListFront = node;
		}
	}
	void SEList_push_back(int node)
	{
		if (m_ListFront == -1)  // start from empty
		{
			m_ListFront = node;
			m_ListTail = node;
			m_SENodeList[node] = -1;
		}
		else
		{
			m_SENodeList[m_ListTail] = node;
			m_SENodeList[node] = -1;
			m_ListTail = node;
		}
	}

	bool SEList_empty()
	{
		return(m_ListFront == -1);
	}

	int SEList_front()
	{
		return m_ListFront;
	}

	void SEList_pop_front()
	{
		int tempFront = m_ListFront;
		m_ListFront = m_SENodeList[m_ListFront];
		m_SENodeList[tempFront] = -1;
	}

	int optimal_label_correcting(int origin_node, int destination_node, int departure_time, int shortest_path_debugging_flag, FILE* pFileDebugLog)
		// time-dependent label correcting algorithm with double queue implementation
	{
		// computing_times ++;
		int internal_debug_flag = 0;

		if (g_node_vector[origin_node].m_outgoing_link_vector.size() == 0)
		{
			return 0;
		}

		for (int i = 0; i < g_number_of_nodes; i++) //Initialization for all nodes
		{
			m_node_status_array[i] = 0;  // not scanned
			m_node_label_cost[i] = _MAX_LABEL_COST;
			m_node_predecessor[i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
			m_link_predecessor[i] = -1;  // pointer to previous NODE INDEX from the current label at current node and time
		}

		//Initialization for origin node at the preferred departure time, at departure time, cost = 0, otherwise, the delay at origin node

		m_node_label_cost[origin_node] = 0;  //absolute time

		m_node_label_generalized_time[origin_node] = 0;  //relative travel  time

		SEList_clear();
		SEList_push_back(origin_node);

		while (!SEList_empty())
		{
			int from_node = SEList_front();//pop a node FromID for scanning
			SEList_pop_front();  // remove current node FromID from the SE list
			m_node_status_array[from_node] = 2;

			if (shortest_path_debugging_flag)
				fprintf(pFileDebugLog, "SP: SE node: %d\n", g_node_vector[from_node].node_id);

			//scan all outbound nodes of the current node
			for (int i = 0; i < g_node_vector[from_node].m_outgoing_link_vector.size(); i++)  // for each link (i,j) belong A(i)
			{

				int to_node = g_node_vector[from_node].m_outgoing_link_vector[i].to_node_seq_no;

				// ASSERT(to_node <= g_number_of_nodes);
				bool  b_node_updated = false;

				float new_to_node_cost = m_node_label_cost[from_node] + m_link_cost_array[g_node_vector[from_node].m_outgoing_link_vector[i].link_seq_no];

				if (shortest_path_debugging_flag)
				{
					fprintf(pFileDebugLog, "SP: checking from node %d, to node %d  cost = %d\n",
						g_node_vector[from_node].node_id,
						g_node_vector[to_node].node_id,
						new_to_node_cost, g_node_vector[from_node].m_outgoing_link_vector[i].cost);
				}



				if (new_to_node_cost < m_node_label_cost[to_node]) // we only compare cost at the downstream node ToID at the new arrival time t
				{

					if (shortest_path_debugging_flag)
					{
						fprintf(pFileDebugLog, "SP: updating node: %d current cost: %.2f, new cost %.2f\n",
							g_node_vector[to_node].node_id,
							m_node_label_cost[to_node], new_to_node_cost);
					}

					// update cost label and node/time predecessor
					float aaa = 0;
					aaa = new_to_node_cost;
					m_node_label_cost[to_node] = aaa;
					int link_seq_no = g_node_vector[from_node].m_outgoing_link_vector[i].link_seq_no;

					//float new_to_node_time = m_node_label_generalized_time[from_node] + g_link_vector[link_seq_no].free_flow_travel_time_in_min;

					//m_node_label_generalized_time[to_node] = new_to_node_time;
					m_node_predecessor[to_node] = from_node;  // pointer to previous physical NODE INDEX from the current label at current node and time
					m_link_predecessor[to_node] = g_node_vector[from_node].m_outgoing_link_vector[i].link_seq_no;  // pointer to previous physical NODE INDEX from the current label at current node and time


					b_node_updated = true;

					if (shortest_path_debugging_flag)
						fprintf(pFileDebugLog, "SP: add node %d into SE List\n",
							g_node_vector[to_node].node_id);
					if (g_node_vector[to_node].node_seq_no != -1 && to_node != origin_node)
					{
						if (m_node_status_array[to_node] == 0)
						{
							SEList_push_back(to_node);
							m_node_status_array[to_node] = 1;
						}

					}
				}

			}
		}

		if (destination_node >= 0 && m_node_label_cost[destination_node] < _MAX_LABEL_COST)
			return 1;
		else if (destination_node == -1)
			return 1;  // one to all shortest pat
		else
			return -1;


	}

	void optimal_label_correcting_for_all_nodes_assigned_to_processor()
	{
		cout << "Test " << m_node_vector.size() << endl;

		for (int i = 0; i < m_node_vector.size(); i++) //Initialization for all nodes
		{
			int origin = m_node_vector[i];
			int return_value = optimal_label_correcting(origin, -1, 0, 0, NULL);  // one to all shortest path
		}
	}
	// step 2: 	scan the shortest path to compute the link volume, 

};