#ifndef __BFWS__
#define __BFWS__

/**
 * LAPKT includes
 */
#include <py_strips_interface.hxx>

#include <iostream>
#include <fstream>

#include <fluent.hxx>
#include <action.hxx>
#include <cond_eff.hxx>
#include <strips_state.hxx>
#include <fwd_search_prob.hxx>
#include <landmark_graph.hxx>
#include <landmark_graph_generator.hxx>
#include <landmark_graph_manager.hxx>
#include "landmark_count.hxx"
#include <h_2.hxx>
#include <h_1.hxx>
#include <layered_h_max.hxx>

#include <open_list.hxx>
#include <string_conversions.hxx>

#include <boost/program_options.hpp>

/**
 * Local includes
 */

// NIR: Files adapted from LAPKT
#include "novelty_partition_1.hxx"
#include "novelty_partition_2.hxx"
#include "ff_rp_heuristic.hxx"
#include "rp_heuristic.hxx"

#include "bfws_4h.hxx"
#include "bfws_2h.hxx"
#include "bfws_2h_M.hxx"
#include "bfws_2h_consistency.hxx"
#include "bfws_2h_consistency_M.hxx"

#include "ipc2014_rwa.hxx"

#include "new_node_comparer.hxx"

//Custom
#include <gs_bfws_3h.hxx>
#include <gs_bfws_3h_rpop_init.hxx>
#include <gs_bfws_thff.hxx>
#include <gs_bfws_3h_op.hxx>

#include <gs_bfcs_3h.hxx>
#include <count_novelty_heuristic.hxx>
#include <count_novelty_partition.hxx>

#include <rp_graph.hxx>
#include <rp_graph_utp.hxx>
#include <rp_graph_thff.hxx>
#include <rp_graph_op.hxx>


namespace po = boost::program_options;

/**
 * NAMESPACES
 */

// NIR: Model
using aptk::Action;
using aptk::agnostic::Fwd_Search_Problem;
// NIR: Heuristics
using aptk::agnostic::Landmarks_Count_Heuristic;
using aptk::agnostic::Landmarks_Graph;
using aptk::agnostic::Landmarks_Graph_Generator;
using aptk::agnostic::Landmarks_Graph_Manager;

using aptk::agnostic::H1_Cost_Function;
using aptk::agnostic::H1_Heuristic;
using aptk::agnostic::H2_Heuristic;
using aptk::agnostic::Layered_H_Max;
using aptk::agnostic::RP_Cost_Function;

using aptk::agnostic::FF_Relaxed_Plan_Heuristic;
using aptk::agnostic::H_Add_Evaluation_Function;
using aptk::agnostic::H_Max_Evaluation_Function;
using aptk::agnostic::Relaxed_Plan_Heuristic;

// NIR: Novelties
using aptk::agnostic::Novelty_Partition;
using aptk::agnostic::Novelty_Partition_2;

// NIR: Open List and evaluation functions
using aptk::search::Node_Comparer_2H;
using aptk::search::Node_Comparer_2H_gn_unit;
using aptk::search::Node_Comparer_4H;
using aptk::search::Open_List;

// NIR: Search Engines
using aptk::search::bfs_dq_mh::IPC2014_RWA;
using aptk::search::bfws_2h::BFWS_2H;
using aptk::search::bfws_2h::BFWS_2H_Consistency;
using aptk::search::bfws_2h::BFWS_2H_Consistency_M;
using aptk::search::bfws_2h::BFWS_2H_M;
using aptk::search::bfws_4h::BFWS_4H;

//CUSTOM
using aptk::agnostic::Count_Novelty_Heuristic;
using aptk::agnostic::Count_Novelty_Partition;

using aptk::search::gs_bfws_3h::GS_BFWS_3H;
using aptk::search::gs_bfws_3h_rpop_init::GS_BFWS_3H_RPOP_INIT;
using aptk::search::gs_bfws_3h_thff::GS_BFWS_3H_THFF;
using aptk::search::gs_bfws_3h_op::GS_BFWS_3H_OP;

using aptk::search::Node_Comparer_3H_gn_unit;
using aptk::search::gs_bfcs_3h::GS_BFCS_3H;
using aptk::search::Pruned_Open_List;

using aptk::search::Alt_Node_Comparer_3H_gn_unit;
using aptk::search::Alt_Node_Comparer_2H_gn_unit;
using aptk::search::Alt_2_Node_Comparer_3H_gn_unit;

using aptk::search::Inverse_Node_Comparer_3H_gn_unit;
using aptk::search::Pruned_Node_Comparer_3H_gn_unit;

using aptk::search::Custom_Priority_Queue;
using aptk::search::Double_Custom_Priority_Queue;
using aptk::search::Triple_Custom_Priority_Queue;

using aptk::agnostic::RelaxedPlanGraph; //OL
using aptk::agnostic::RelaxedPlanGraphUTP;
using aptk::agnostic::RelaxedPlanGraphTHFF;
using aptk::agnostic::RelaxedPlanGraphOP; //OP


/**
 * DEFINITIONS
 */

// NIR: Heuristics
typedef H2_Heuristic<Fwd_Search_Problem> H2_Fwd;
typedef Landmarks_Graph_Generator<Fwd_Search_Problem> Gen_Lms_Fwd;
typedef Landmarks_Count_Heuristic<Fwd_Search_Problem> H_Lmcount_Fwd;
typedef Landmarks_Graph_Manager<Fwd_Search_Problem> Land_Graph_Man;

// NIR: Node representations for each search algorithm
typedef aptk::search::bfws_4h::Node<Fwd_Search_Problem, aptk::State> Search_Node_4h;
typedef aptk::search::bfws_2h::Node<Fwd_Search_Problem, aptk::State> Search_Node_2h;
typedef aptk::search::ipc2014::Node<aptk::State> AT_Search_Node;


// NIR: Novelty functions for each node type. Novelty partition expects class
// node to define partition() function. '_2' version expects partition2() function.
typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_4h> H_Novel_Fwd_4h;
typedef Novelty_Partition_2<Fwd_Search_Problem, Search_Node_4h> H_Novel_2_Fwd_4h;

typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_2h> H_Novel_Fwd_2h;


// NIR: Then we define the type of the tie-breaking algorithm
// for the open list we are going to use
typedef Node_Comparer_4H<Search_Node_4h> Tie_Breaking_Algorithm_4h;
typedef Node_Comparer_2H_gn_unit<Search_Node_2h> Tie_Breaking_Algorithm_2h_ignore_costs;

// NIR: use this node comparer if you want BFWS to use gn with real costs
////////typedef		Node_Comparer_2H< Search_Node_2h >	        		Tie_Breaking_Algorithm_2h;

// NIR: Now we define the Open List type by combining the types we have defined before
typedef Open_List<Tie_Breaking_Algorithm_4h, Search_Node_4h> BFS_Open_List_4h;
typedef Open_List<Tie_Breaking_Algorithm_2h_ignore_costs, Search_Node_2h> BFS_Open_List_2h;
typedef AT_Search_Node::Open_List AT_BFS_Open_List;

// NIR: Now we define the heuristics
typedef H1_Heuristic<Fwd_Search_Problem, H_Add_Evaluation_Function, H1_Cost_Function::Ignore_Costs> H_Add_Fwd;
typedef Relaxed_Plan_Heuristic<Fwd_Search_Problem, H_Add_Fwd, RP_Cost_Function::Ignore_Costs> H_Add_Rp_Fwd;

typedef H1_Heuristic<Fwd_Search_Problem, H_Add_Evaluation_Function, H1_Cost_Function::Use_Costs> H_Add_Fwd_use_costs;
typedef Relaxed_Plan_Heuristic<Fwd_Search_Problem, H_Add_Fwd, RP_Cost_Function::Use_Costs> H_Add_Rp_Fwd_use_costs;

typedef Layered_H_Max<Fwd_Search_Problem> Alt_H_Max;
typedef FF_Relaxed_Plan_Heuristic<Fwd_Search_Problem, Alt_H_Max, unsigned> Classic_FF_H_Max;

// NIR: Now we're ready to define the BFS algorithm we're going to use, H_Lmcount can be used only with goals,
// or with landmarks computed from s0
typedef BFWS_2H<Fwd_Search_Problem, H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h> k_BFWS;
typedef BFWS_2H_M<Fwd_Search_Problem, H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h> k_BFWS_M;
typedef BFWS_4H<Fwd_Search_Problem, H_Novel_Fwd_4h, H_Lmcount_Fwd, H_Novel_2_Fwd_4h, H_Add_Rp_Fwd, BFS_Open_List_4h> BFWS_w_hlm_hadd;

// NIR: Consistency Search variants
typedef BFWS_2H_Consistency<Fwd_Search_Problem, H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h> k_BFWS_Consistency;
typedef BFWS_2H_Consistency_M<Fwd_Search_Problem, H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd, BFS_Open_List_2h> k_BFWS_Consistency_M;

// NIR: Anytime RWA*
typedef IPC2014_RWA<Fwd_Search_Problem, H_Add_Rp_Fwd_use_costs, H_Lmcount_Fwd, AT_BFS_Open_List> Anytime_RWA;


//CUSTOM
typedef aptk::search::gs_bfcs_3h::Node<Fwd_Search_Problem, aptk::State> Search_Node_3h;
typedef Count_Novelty_Heuristic<Fwd_Search_Problem, Search_Node_3h> H_Novel_Count_Blind;
typedef Count_Novelty_Partition<Fwd_Search_Problem, Search_Node_3h> H_Novel_Count_Partition;
typedef Node_Comparer_3H_gn_unit<Search_Node_3h> Tie_Breaking_Algorithm_3h_ignore_costs;
typedef Alt_Node_Comparer_3H_gn_unit<Search_Node_3h> Alt_Tie_Breaking_Algorithm_3h_ignore_costs;
typedef Node_Comparer_2H_gn_unit<Search_Node_3h> Tie_Breaking_Algorithm_2h_ignore_costs_bfcs;
typedef Alt_Node_Comparer_2H_gn_unit<Search_Node_3h> Alt_Tie_Breaking_Algorithm_2h_ignore_costs_bfcs;
typedef Alt_2_Node_Comparer_3H_gn_unit<Search_Node_3h> Alt_2_Tie_Breaking_Algorithm_2h_ignore_costs_bfcs;


typedef Open_List<Tie_Breaking_Algorithm_3h_ignore_costs, Search_Node_3h> BFS_Open_List_3h;
typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_3h> H_Novel_Fwd_3h;
// typedef GS_BFWS_3H<Fwd_Search_Problem, H_Novel_Fwd_3h, H_Lmcount_Fwd, H_Novel_Count_Blind, H_Add_Rp_Fwd, BFS_Open_List_3h> custom_BFWS;


typedef aptk::agnostic::RelaxedPlanGraph<Fwd_Search_Problem> OL;
typedef aptk::agnostic::RelaxedPlanGraphUTP<Fwd_Search_Problem>	 OL_UTP;
typedef aptk::agnostic::RelaxedPlanGraphOP<Fwd_Search_Problem>	 OP;
typedef aptk::agnostic::RelaxedPlanGraphTHFF<Fwd_Search_Problem> THFF;


typedef aptk::search::gs_bfws_3h::Node<Fwd_Search_Problem, aptk::State> Search_Node_3h_bfws;
typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws> H_Novel_Fwd_3h_bfws;
typedef Count_Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws> H_Novel_Count_Partition_bfws;
typedef Node_Comparer_3H_gn_unit<Search_Node_3h_bfws> Tie_Breaking_Algorithm_3h_ignore_costs_bfws;
typedef Open_List<Tie_Breaking_Algorithm_3h_ignore_costs_bfws, Search_Node_3h_bfws> BFS_Open_List_3h_bfws_normal;
typedef Custom_Priority_Queue<Tie_Breaking_Algorithm_3h_ignore_costs_bfws, Search_Node_3h_bfws> BFS_Open_List_3h_bfws_trimmed;


typedef GS_BFWS_3H<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws, H_Add_Rp_Fwd, OL, BFS_Open_List_3h_bfws_trimmed> BFWS_3H;
typedef GS_BFWS_3H<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws, H_Add_Rp_Fwd, OL_UTP, BFS_Open_List_3h_bfws_trimmed> BFWS_3H_UTP;
typedef GS_BFWS_3H<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws, H_Add_Rp_Fwd, OL, BFS_Open_List_3h_bfws_normal> BFWS_3H_NORMAL_OL;
typedef GS_BFWS_3H<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws, H_Add_Rp_Fwd, OL_UTP, BFS_Open_List_3h_bfws_normal> BFWS_3H_UTP_NORMAL_OL;


typedef aptk::search::gs_bfws_3h_rpop_init::Node<Fwd_Search_Problem, aptk::State> Search_Node_3h_bfws_rpop_init;
typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws_rpop_init> H_Novel_Fwd_3h_bfws_rpop_init;
typedef Count_Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws_rpop_init> H_Novel_Count_Partition_bfws_rpop_init;
typedef Node_Comparer_3H_gn_unit<Search_Node_3h_bfws_rpop_init> Tie_Breaking_Algorithm_3h_ignore_costs_bfws_rpop_init;
typedef Open_List<Tie_Breaking_Algorithm_3h_ignore_costs_bfws_rpop_init, Search_Node_3h_bfws_rpop_init> BFS_Open_List_3h_bfws_normal_rpop_init;
typedef Custom_Priority_Queue<Tie_Breaking_Algorithm_3h_ignore_costs_bfws_rpop_init, Search_Node_3h_bfws_rpop_init> BFS_Open_List_3h_bfws_trimmed_rpop_init;

typedef GS_BFWS_3H_RPOP_INIT<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_rpop_init, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_rpop_init, H_Add_Rp_Fwd, OL, BFS_Open_List_3h_bfws_trimmed_rpop_init> BFWS_3H_OL_INIT;
typedef GS_BFWS_3H_RPOP_INIT<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_rpop_init, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_rpop_init, H_Add_Rp_Fwd, OL_UTP, BFS_Open_List_3h_bfws_trimmed_rpop_init> BFWS_3H_OL_INIT_UTP;
typedef GS_BFWS_3H_RPOP_INIT<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_rpop_init, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_rpop_init, H_Add_Rp_Fwd, OL, BFS_Open_List_3h_bfws_normal_rpop_init> BFWS_3H_OL_INIT_NORMAL_OL;
typedef GS_BFWS_3H_RPOP_INIT<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_rpop_init, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_rpop_init, H_Add_Rp_Fwd, OL_UTP, BFS_Open_List_3h_bfws_normal_rpop_init> BFWS_3H_OL_INIT_UTP_NORMAL_OL;


typedef aptk::search::gs_bfws_3h_thff::Node<Fwd_Search_Problem, aptk::State> Search_Node_3h_bfws_thff;
typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws_thff> H_Novel_Fwd_3h_bfws_thff;
typedef Count_Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws_thff> H_Novel_Count_Partition_bfws_thff;
typedef Node_Comparer_3H_gn_unit<Search_Node_3h_bfws_thff> Tie_Breaking_Algorithm_3h_ignore_costs_bfws_thff;
typedef Open_List<Tie_Breaking_Algorithm_3h_ignore_costs_bfws_thff, Search_Node_3h_bfws_thff> BFS_Open_List_3h_bfws_normal_thff;
typedef Custom_Priority_Queue<Tie_Breaking_Algorithm_3h_ignore_costs_bfws_thff, Search_Node_3h_bfws_thff> BFS_Open_List_3h_bfws_trimmed_thff;

typedef GS_BFWS_3H_THFF<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_thff, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_thff, H_Add_Rp_Fwd, THFF, BFS_Open_List_3h_bfws_trimmed_thff> BFWS_3H_THFF;
typedef GS_BFWS_3H_THFF<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_thff, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_thff, H_Add_Rp_Fwd, THFF, BFS_Open_List_3h_bfws_normal_thff> BFWS_3H_THFF_NORMAL_OL;


typedef aptk::search::gs_bfws_3h_op::Node<Fwd_Search_Problem, aptk::State> Search_Node_3h_bfws_op;
typedef Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws_op> H_Novel_Fwd_3h_bfws_op;
typedef Count_Novelty_Partition<Fwd_Search_Problem, Search_Node_3h_bfws_op> H_Novel_Count_Partition_bfws_op;
typedef Node_Comparer_3H_gn_unit<Search_Node_3h_bfws_op> Tie_Breaking_Algorithm_3h_ignore_costs_bfws_op;
typedef Open_List<Tie_Breaking_Algorithm_3h_ignore_costs_bfws_op, Search_Node_3h_bfws_op> BFS_Open_List_3h_bfws_normal_op;
typedef Custom_Priority_Queue<Tie_Breaking_Algorithm_3h_ignore_costs_bfws_op, Search_Node_3h_bfws_op> BFS_Open_List_3h_bfws_trimmed_op;

typedef GS_BFWS_3H_OP<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_op, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_op, H_Add_Rp_Fwd, OP, BFS_Open_List_3h_bfws_trimmed_op> BFWS_3H_OP;
typedef GS_BFWS_3H_OP<Fwd_Search_Problem, H_Novel_Fwd_3h_bfws_op, H_Lmcount_Fwd, H_Novel_Count_Partition_bfws_op, H_Add_Rp_Fwd, OP, BFS_Open_List_3h_bfws_normal_op> BFWS_3H_OP_NORMAL_OL;




typedef Inverse_Node_Comparer_3H_gn_unit<Search_Node_3h> Inverse_Tie_Breaking_Algorithm_3h_ignore_costs;


typedef Pruned_Open_List<Tie_Breaking_Algorithm_3h_ignore_costs, Inverse_Tie_Breaking_Algorithm_3h_ignore_costs, Search_Node_3h> Pruned_BFS_Open_List_3h;


typedef Custom_Priority_Queue<Tie_Breaking_Algorithm_3h_ignore_costs, Search_Node_3h> Testing_Open_List;
typedef Double_Custom_Priority_Queue<Tie_Breaking_Algorithm_2h_ignore_costs_bfcs, Alt_Tie_Breaking_Algorithm_2h_ignore_costs_bfcs, Search_Node_3h> Double_Testing_Open_List;
typedef Triple_Custom_Priority_Queue<Tie_Breaking_Algorithm_2h_ignore_costs_bfcs, Alt_Tie_Breaking_Algorithm_2h_ignore_costs_bfcs, Alt_2_Tie_Breaking_Algorithm_2h_ignore_costs_bfcs, Search_Node_3h> Triple_Testing_Open_List;
// typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Blind, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, BFS_Open_List_3h> BFCS_1;
// typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Partition, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, BFS_Open_List_3h> BFCS_1_p;
// typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Partition, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, Pruned_BFS_Open_List_3h> BFCS_1_p_pruned;

// typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Partition, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, Testing_Open_List> BFCS_1_p_pruned;
// typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Partition, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, BFS_Open_List_3h> BFCS_1_p_pruned;
typedef Custom_Priority_Queue<Tie_Breaking_Algorithm_2h_ignore_costs, Search_Node_2h> Testing_Open_List_2h;
// typedef BFWS_2H<Fwd_Search_Problem, H_Novel_Fwd_2h, H_Lmcount_Fwd, H_Add_Rp_Fwd, Testing_Open_List_2h> k_BFWS;

typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Partition, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, Double_Testing_Open_List> BFCS_1_p_pruned;
// typedef GS_BFCS_3H<Fwd_Search_Problem, H_Novel_Count_Partition, H_Lmcount_Fwd, H_Novel_Fwd_3h, H_Add_Rp_Fwd, Triple_Testing_Open_List> BFCS_1_p_pruned;




class BFWS : public STRIPS_Interface
{
public:
	BFWS();
	BFWS(std::string, std::string);
	virtual ~BFWS();

	virtual void setup(bool gen_match_tree = true);
	void solve();

	std::string m_log_filename;
	std::string m_plan_filename;
	std::string m_search_alg;
	unsigned m_M;
	unsigned m_max_novelty;
	bool m_anytime;
	bool m_found_plan;
	float m_cost;
	float m_cost_bound;
	bool m_verbose = false;
	int m_memory_limit;
	int m_time_limit;
	bool m_fallback_backend;
	std::string m_backend_type;
	int m_tol_max_depth;
	int m_seed;
	bool m_normal_open_list;
	bool m_goalcount;
	int m_rpop_num_samples;
	int m_rpop_max_ms;

protected:
	template <typename Search_Engine>
	void bfws_options(Fwd_Search_Problem &search_prob, Search_Engine &bfs_engine, unsigned max_novelty, Landmarks_Graph &graph);

	template <typename Search_Engine>
	void bfcs_options(Fwd_Search_Problem &search_prob, Search_Engine &bfs_engine, unsigned max_novelty_width, unsigned max_novelty_count, Landmarks_Graph &graph);

	template <typename Search_Engine>
	float do_search(Search_Engine &engine, aptk::STRIPS_Problem &plan_prob, std::ofstream &plan_stream);

	float do_anytime(Anytime_RWA &engine);
};

#endif
