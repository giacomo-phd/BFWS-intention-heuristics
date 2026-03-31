#ifndef __BFS_W_COUNT_Planner__
#define __BFS_W_COUNT_Planner__

// //---- CONSTANTS
// #define IW_BOUND 2
// #define LOG_FILE "planner.log"
// #define PLAN_FILE "plan.ipc"

#include <iostream>
#include <fstream>

// #include <ff_to_aptk.hxx>
// #include <strips_prob.hxx>
#include <fluent.hxx>
#include <action.hxx>
#include <cond_eff.hxx>
// #include <strips_state.hxx>
// #include <fwd_search_prob.hxx>

// #include <h_1.hxx>
#include <novelty.hxx>
#include <count_novelty_heuristic.hxx>

// #include <aptk/open_list.hxx>
// #include <aptk/string_conversions.hxx>
// #include <aptk/at_bfs.hxx>
#include <open_list.hxx>
#include <string_conversions.hxx>
// #include <at_bfs.hxx>
#include <gs_custom_bfs_2h.hxx>

#include <py_strips_interface.hxx>
#include <strips_prob.hxx>
#include <strips_state.hxx>
#include <fwd_search_prob.hxx>

#include <fstream>

#include <boost/program_options.hpp>

using	aptk::STRIPS_Problem;

using	aptk::agnostic::Fwd_Search_Problem;
using	aptk::Action;

// using 	aptk::agnostic::H1_Heuristic;
// using	aptk::agnostic::H_Add_Evaluation_Function;
// using	aptk::agnostic::H_Max_Evaluation_Function;
using   aptk::agnostic::Count_Novelty_Heuristic;
using   aptk::agnostic::Novelty;

using 	aptk::search::Open_List;
using	aptk::search::gs_custom_bfs_2h::AT_BFS_SQ_2H;
// using	aptk::search::bfs::AT_BFS_SQ_SH;
using	aptk::search::Node_Comparer_DH;

//---- BFS_W_COUNT_Planner Class -----------------------------------------------------//
class BFS_W_COUNT_Planner : public STRIPS_Interface
{
public:
        typedef aptk::search::gs_custom_bfs_2h::Node<aptk::State> Search_Node;
        // typedef aptk::search::bfs::Node<aptk::State> Search_Node;
        typedef	Node_Comparer_DH< Search_Node > Tie_Breaking_Algorithm;
        typedef Open_List< Tie_Breaking_Algorithm, Search_Node > BFS_Open_List;
        //typedef H1_Heuristic<Fwd_Search_Problem, H_Max_Evaluation_Function> H_Max_Fwd;
        // typedef Count_Novelty_Heuristic<Fwd_Search_Problem, Search_Node> H_Count_Novelty;
        typedef Novelty<Fwd_Search_Problem, Search_Node> H_1;
        typedef Count_Novelty_Heuristic<Fwd_Search_Problem, Search_Node> H_2;
        typedef AT_BFS_SQ_2H< Fwd_Search_Problem, H_1, H_2, BFS_Open_List > BFS_W_COUNT;

        BFS_W_COUNT_Planner();
        BFS_W_COUNT_Planner(std::string, std::string);
        BFS_W_COUNT_Planner(std::string, std::string, unsigned,
                   std::string, std::string);
        virtual ~BFS_W_COUNT_Planner();

        virtual void setup(bool gen_match_tree = true);
        void solve();

        unsigned m_iw_bound;
        std::string m_log_filename;
        std::string m_plan_filename;
        bool m_atomic = false;

        bool greedy = true;
        bool delayed = true;

        unsigned m_memory_budget;
        bool m_h2_blind_only;
        

protected:
        template <typename Search_Engine>
        float do_search(Search_Engine &engine,
                        aptk::STRIPS_Problem &plan_prob,
                        std::ofstream &plan_stream);
        template <typename Search_Engine>
        float do_inc_bound_search(Search_Engine &engine,
                                  aptk::STRIPS_Problem &plan_prob,
                                  std::ofstream &plan_stream);
        template <typename Search_Engine>
        float do_search_single_goal(
            Search_Engine &engine, aptk::STRIPS_Problem &plan_prob,
            std::ofstream &plan_stream);
};
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx//
#endif
