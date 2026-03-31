/*
Lightweight Automated Planning Toolkit
Copyright (C) 2012
Miquel Ramirez <miquel.ramirez@rmit.edu.au>
Nir Lipovetzky <nirlipo@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



#include <bfs_w_count_planner.hxx>
#include <fstream>
#include <memory.hxx>


BFS_W_COUNT_Planner::BFS_W_COUNT_Planner()
        : STRIPS_Interface(), m_log_filename("bfs_w_count.log"), m_plan_filename("plan.ipc"), m_memory_budget(3900)
        {
        }

BFS_W_COUNT_Planner::BFS_W_COUNT_Planner(std::string domain_file, std::string instance_file)
        : STRIPS_Interface(domain_file, instance_file), m_iw_bound(1), m_log_filename("bfs_w_count.log"), m_plan_filename("plan.ipc"), m_memory_budget(3900)
        {
        }
BFS_W_COUNT_Planner::BFS_W_COUNT_Planner(std::string domain_file, std::string instance_file,
                       unsigned bound, std::string log_file, std::string plan_file)
        : STRIPS_Interface(domain_file, instance_file), m_iw_bound(1), m_log_filename("bfs_w_count.log"), m_plan_filename("plan.ipc"), m_memory_budget(3900)
        {
        }

BFS_W_COUNT_Planner::~BFS_W_COUNT_Planner() {}

void BFS_W_COUNT_Planner::setup(bool gen_match_tree)
{
    // MRJ: Call superclass method, then do you own thing here
	STRIPS_Interface::setup(gen_match_tree);
	std::cout << "PDDL problem description loaded: " << std::endl;
	std::cout << "\tDomain: " << instance()->domain_name() << std::endl;
	std::cout << "\tProblem: " << instance()->problem_name() << std::endl;
	std::cout << "\t#Actions: " << instance()->num_actions() << std::endl;
	std::cout << "\t#Fluents: " << instance()->num_fluents() << std::endl;
}

template<typename Search_Engine>
float BFS_W_COUNT_Planner::do_search_single_goal(Search_Engine &engine,
                                               aptk::STRIPS_Problem &plan_prob, std::ofstream &plan_stream)
{
    std::ofstream details(m_log_filename);

    // std::vector<aptk::Action_Idx> plan;
    // float cost;

    float ref = aptk::time_used();
    aptk::Fluent_Vec goals = plan_prob.goal();
    aptk::search::gs_custom_bfs_2h::Atomic_Plan_Vec plans(goals.size());

    float total_time = 0;

    float max_bound = m_iw_bound;

    bool solved = true;

    engine.set_greedy( greedy );
	engine.set_delay_eval( false );
    std::cout<<"Setting Novelty arity: "<< m_iw_bound <<std::endl;
    engine.set_arity_h1( m_iw_bound );
    //hardcode to 1 for now
    engine.set_arity_h2( 1 );
    // std::cout <<"DEBUG: blindonly: "<<m_h2_blind_only<<std::endl;
    engine.set_blind_only_h2(m_h2_blind_only);

    /* 
    Set the memory budget for the BFS algorithm, stops search when exceeds this limit. Necessary to keep process running and return 
    data from previously achieved atomic goals when setting a limit with also an external wrapper library, e.g. lab.
     */
    engine.set_memory_budget_MB( m_memory_budget );

    engine.start();

    std::cout << "Starting search for " << goals.size() << " atomic goals."<<std::endl;
        

    // if (engine.find_solution(cost, plan))
    engine.find_atomic_solution(plans);

    details << "Number of achieved atomic goals: " << plans.get_a_goals_achieved() << " of " << goals.size()<<std::endl;
    std::cout << "Number of achieved atomic goals: " << plans.get_a_goals_achieved() << " of " << goals.size()<<std::endl;
    
    unsigned index = 0;
    for (auto atom_tuple_it = plans.begin(); atom_tuple_it != plans.end(); atom_tuple_it++)
    {
        auto &single_atom_goal = std::get<0>(*atom_tuple_it);
        //if it is a found atomic goal
        if (single_atom_goal != -1)
        {
            std::vector<aptk::Action_Idx> &plan = std::get<1>(*atom_tuple_it);
            float &cost = std::get<2>(*atom_tuple_it);
            details << std::endl;
            details << "Plan found for single goal: [" << single_atom_goal << "]" <<std::endl;
            std::cout << std::endl;
            std::cout << "#singlegoalstart" <<std::endl;
            std::cout << "Plan found for single goal: [" << single_atom_goal << "]" <<std::endl;
            
            details << "Plan found with cost: " << cost << std::endl;
            std::cout << "Plan found with cost: " << cost << std::endl;

            for (int k=0; k < plan.size(); k++)
            {
                details << k + 1 << ". ";
                const aptk::Action &a = *(plan_prob.actions()[plan[k]]);
                details << a.signature();
                details << std::endl;
                plan_stream << a.signature() << std::endl;
            }
            float a_time = engine.atomic_search_time(single_atom_goal);
            unsigned a_expanded = engine.atomic_expanded(single_atom_goal);
            unsigned a_generated = engine.atomic_generated(single_atom_goal);

            details << "Time: " << (a_time) << std::endl;
            details << "Generated: " << (a_generated) << std::endl;
            details << "Expanded: " << (a_expanded) << std::endl;

            std::cout << "Time: " << (a_time) << std::endl;
            std::cout << "Generated: " << (a_generated) << std::endl;
            std::cout << "Expanded: " << (a_expanded) << std::endl;
            std::cout << "#singlegoalend" << std::endl;
            
            plans.reset_tuple(single_atom_goal);

            index++;
        }
        else 
        {
            details << std::endl;
            details << ";; NOT I-REACHABLE ;;" << std::endl;
            std::cout << std::endl;
            std::cout << ";; NOT I-REACHABLE ;;" << std::endl;
        }
        solved = true;
    }

    float partial_time = aptk::time_used() - ref;
    total_time = partial_time;
    details << std::endl;
    details << "Total time: " << partial_time << std::endl;
    details << "Nodes generated during search: " << engine.generated() << std::endl;
    details << "Nodes expanded during search: " << engine.expanded() << std::endl;
    details << "Effective Width during search: " << engine.bound() << std::endl;

    // details << "Nodes pruned by bound: " << engine.sum_pruned_by_bound() << std::endl;
    // details << "Average ef. width: " << engine.avg_B() << std::endl;
    // details << "Max ef. width: " << engine.max_B() << std::endl;

    std::cout << std::endl;
    std::cout << "Total time: " << partial_time << std::endl;
    std::cout << "Nodes generated during search: " << engine.generated() << std::endl;
    std::cout << "Nodes expanded during search: " << engine.expanded() << std::endl;
    std::cout << "Max novelty expanded: " << engine.bound() << std::endl;

    details.close();
    return total_time;
}

template <typename Search_Engine>
float BFS_W_COUNT_Planner::do_search(Search_Engine &engine, aptk::STRIPS_Problem &plan_prob,
                            std::ofstream &plan_stream)
{
    // engine.start();

    std::ofstream details(m_log_filename);

    std::vector<aptk::Action_Idx> plan;
	float cost;

	float ref = aptk::time_used();
	float t0 = aptk::time_used();

	engine.set_greedy( greedy );
	engine.set_delay_eval( delayed );
    engine.set_arity_h1( m_iw_bound );
    //hardcode to 1 for now
    engine.set_arity_h2( 1 );
    engine.set_memory_budget_MB( m_memory_budget );

	engine.start();

    unsigned expanded_0 = engine.expanded();
	unsigned generated_0 = engine.generated();

    //std::ofstream plan_stream(m_plan_filename.c_str());

    if (engine.find_solution(cost, plan))
    {
        details << "Plan found with cost: " << cost << std::endl;
        std::cout << "Plan found with cost: " << cost << std::endl;
        for (unsigned k = 0; k < plan.size(); k++)
        {
            details << k + 1 << ". ";
            const aptk::Action &a = *(plan_prob.actions()[plan[k]]);
            details << a.signature();
            details << std::endl;
            plan_stream << a.signature() << std::endl;
        }
        float tf = aptk::time_used();
        unsigned expanded_f = engine.expanded();
        unsigned generated_f = engine.generated();
        details << "Time: " << (tf - t0) << std::endl;
        details << "Generated: " << (generated_f - generated_0) << std::endl;
        details << "Expanded: " << (expanded_f - expanded_0) << std::endl;
        t0 = tf;
        expanded_0 = expanded_f;
        generated_0 = generated_f;
        plan.clear();
    }
    else
    {
        details << ";; NOT I-REACHABLE ;;" << std::endl;
        std::cout << ";; NOT I-REACHABLE ;;" << std::endl;
    }

    float total_time = aptk::time_used() - ref;
    details << "Total time: " << total_time << std::endl;
    details << "Nodes generated during search: " << std::endl;
    details << "Nodes expanded during search: " << std::endl;

    // details <<  "Nodes pruned by bound: "   <<  engine.sum_pruned_by_bound() <<
    //     std::endl;
    // details <<  "Average ef. width: "   <<  engine.avg_B()  <<  std:endl;
    // details <<  "Max ef. width: "   <<  engine.max_B()  <<  std::endl;
    details.close();
    std::cout << "Total time: " << total_time << std::endl;
    std::cout << "Nodes generated during search: " << engine.generated() << std::endl;
    std::cout << "Nodes expanded during search: " << engine.expanded() << std::endl;
    std::cout << "Max novelty expanded: " << engine.bound() << std::endl;


#ifdef __linux__
    aptk::report_memory_usage();
#endif

    return total_time;
}

void BFS_W_COUNT_Planner::solve()
{
    Fwd_Search_Problem search_prob(instance());
    
    std::ofstream plan_stream;
    plan_stream.open(m_plan_filename);

    std::cout << "Starting search with BFS_w_count ..." << std::endl;
    std::cout << "Search algorithm memory budget: " << m_memory_budget << " MB"<<std::endl;

    // BFS_H_Max_Fwd bfs_engine(search_prob);
    BFS_W_COUNT bfs_engine(search_prob);

    /**
     * TODO: find correct place to set m_atomic
    */
    m_atomic = true;

    float bfs_t;
    if (m_atomic){
        std::cout << "---Performing single goal search" << std::endl;
        bfs_t = do_search_single_goal(bfs_engine, search_prob.task(), plan_stream);
    }
    else
        bfs_t = do_search(bfs_engine, search_prob.task(), plan_stream);

    std::cout << "BFS search completed in " << bfs_t << " secs, check '" << m_log_filename << "' for details" << std::endl;

    plan_stream.close();
}
