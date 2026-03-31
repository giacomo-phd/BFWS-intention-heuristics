
/*
Lightweight Automated Planning Toolkit

Copyright 2022
Miquel Ramirez <miquel.ramirez@unimelb.edu.au>Nir Lipovetzky <nirlipo@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files
(the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject
 to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GIACOMOS_CUSTOM_SINGLE_QUEUE_SINGLE_HEURISTIC_BEST_FIRST_SEARCH__
#define __GIACOMOS_CUSTOM_SINGLE_QUEUE_SINGLE_HEURISTIC_BEST_FIRST_SEARCH__

#include <search_prob.hxx>
#include <resources_control.hxx>
#include <closed_list.hxx>
#include <ext_math.hxx> // no_such_index, infty
#include <vector>
#include <algorithm>
#include <iostream>

#include <types.hxx>
#include <memory.hxx>

namespace aptk
{

	namespace search
	{

		namespace custom_bfs
		{

			template <typename State>
			class Node
			{
			public:
				typedef State State_Type;

				Node(State *s, float cost, Action_Idx action, Node<State> *parent)
						: m_state(s), m_parent(parent), m_action(action), m_g(0), m_f(0), m_h(0)
				{
					m_g = (parent ? parent->m_g + cost : 0.0f);
				}

				virtual ~Node()
				{
					if (m_state != NULL)
						delete m_state;
				}

				float &hn() { return m_h; }
				float hn() const { return m_h; }
				float &gn() { return m_g; }
				float gn() const { return m_g; }
				float &fn() { return m_f; }
				float fn() const { return m_f; }
				Node<State> *parent() { return m_parent; }
				Action_Idx action() const { return m_action; }
				bool has_state() const { return m_state != NULL; }
				void set_state(State *s) { m_state = s; }
				State *state() { return m_state; }
				const State &state() const { return *m_state; }
				void print(std::ostream &os) const
				{
					os << "{@ = " << this << ", s = " << m_state << ", parent = " << m_parent << ", g(n) = " << m_g << ", h(n) = " << m_h << ", f(n) = " << m_f << "}";
				}

				bool operator==(const Node<State> &o) const
				{

					if (&(o.state()) != NULL && &(state()) != NULL)
						return (const State &)(o.state()) == (const State &)(state());
					/**
					 * Lazy
					 */
					if (m_parent == NULL)
					{
						if (o.m_parent == NULL)
							return true;
						return false;
					}

					if (o.m_parent == NULL)
						return false;

					return (m_action == o.m_action) && (*(m_parent->m_state) == *(o.m_parent->m_state));
				}

				size_t hash() const { return m_state->hash(); }

				//for compatibility
				Fluent_Vec *&rp_vec() { return m_rp_fl_vec; }
				Fluent_Set *&rp_set() { return m_rp_fl_set; }

			public:
				State *m_state;
				Node<State> *m_parent;
				float m_h;
				Action_Idx m_action;
				float m_g;
				float m_f;
				Fluent_Vec *m_rp_fl_vec;
				Fluent_Set *m_rp_fl_set;
			};

			class Atomic_Plan_Vec
			{
			public:
				Atomic_Plan_Vec(unsigned n) : n_elements(n), a_goals_achieved(0)
				{
					for(int i = 0; i<n_elements; i++)
						plan_vector.push_back(std::make_tuple(-1, std::vector<Action_Idx>(), 0));
				}

				std::vector<std::tuple<int, std::vector<Action_Idx>, float>>::iterator begin() 
				{
					return plan_vector.begin();
				}

				std::vector<std::tuple<int, std::vector<Action_Idx>, float>>::iterator end()
				{
					return plan_vector.end();
				}

				/**
				 * could be more efficient but doesn't really matter as its after the search
				*/
				void reset_tuple(unsigned atom)
				{
					unsigned i = 0;
					for (auto it = plan_vector.begin(); it != plan_vector.end(); it++)
					{
						if (std::get<0>(*it) == atom)
						{
							auto t = plan_vector[i];
							std::get<0>(t) = -1;
							std::get<1>(t).clear();
							std::get<2>(t) = 0;
						}
						i++;
					}
				}

				// void reset_tuple(int i)
				// {
				// 	auto t = plan_vector[i];
				// 	std::get<0>(t) = -1;
				// 	std::get<1>(t).clear();
				// 	std::get<2>(t) = 0;
				// }

				std::tuple<int, std::vector<Action_Idx>, float>& get_tuple(int i) {	return plan_vector[i]; }
				std::vector<std::tuple<int, std::vector<Action_Idx>, float>>& get_vector() { return plan_vector; }
				void increment_a_goals_achieved() {a_goals_achieved++;}
				int get_a_goals_achieved() {return a_goals_achieved;}
				bool no_a_goals_achieved() {return a_goals_achieved == 0;}

			public:
				std::vector<std::tuple<int, std::vector<Action_Idx>, float>> plan_vector;
				int n_elements;
				int a_goals_achieved;
			};

			// Anytime best-first search, with one single open list and one single
			// heuristic estimator, with delayed evaluation of states generated
			template <typename Search_Model, typename Abstract_Heuristic, typename Open_List_Type>
			class AT_BFS_SQ_SH
			{

			public:
				typedef typename Search_Model::State_Type State;
				typedef typename Open_List_Type::Node_Type Search_Node;
				typedef Closed_List<Search_Node> Closed_List_Type;

				AT_BFS_SQ_SH(const Search_Model &search_problem)
						: m_problem(search_problem), m_heuristic_func(NULL),
							m_exp_count(0), m_gen_count(0), m_pruned_B_count(0), m_dead_end_count(0), m_open_repl_count(0),
							m_B(infty), m_time_budget(infty), m_greedy(false), m_delay_eval(true)
				{
					m_heuristic_func = new Abstract_Heuristic(search_problem);
				}

				virtual ~AT_BFS_SQ_SH()
				{
					for (typename Closed_List_Type::iterator i = m_closed.begin();
							 i != m_closed.end(); i++)
					{
						delete i->second;
					}
					while (!m_open.empty())
					{
						Search_Node *n = m_open.pop();
						delete n;
					}
					m_closed.clear();
					m_open_hash.clear();
					delete m_heuristic_func;
				}

				void reset()
				{
					/**
					 * empty open list and closed lists: Open_List_Type m_open; Closed_List_Type m_closed, m_open_hash;
					*/
					for (typename Closed_List_Type::iterator i = m_closed.begin();
							 i != m_closed.end(); i++)
					{
						delete i->second;
					}
					while (!m_open.empty())
					{
						Search_Node *n = m_open.pop();
						delete n;
					}
					m_closed.clear();
					m_open_hash.clear();

					/**
					 * initialize/reset for count_novelty_heuristic
					*/
					m_heuristic_func->init();

					/**
					 * reset counts
					*/
					m_exp_count = 0;
					m_gen_count = 0;
					m_pruned_B_count = 0;
					m_dead_end_count = 0;
					m_open_repl_count = 0;

				}

				void start()
				{					
					m_root = new Search_Node(m_problem.init(), 0.0f, no_op, NULL);
#ifdef DEBUG
					std::cout << "Initial search node: ";
					m_root->print(std::cout);
					std::cout << std::endl;
					m_root->state()->print(std::cout);
					std::cout << std::endl;
#endif
					m_open.insert(m_root);
					m_open_hash.put(m_root);
					if (!m_delay_eval)
						eval(m_root);
					inc_gen();
				}

				bool find_solution(float &cost, std::vector<Action_Idx> &plan)
				{
					m_t0 = time_used();
					Search_Node *end = do_search();
					if (end == NULL)
						return false;
					extract_plan(m_root, end, plan, cost);

					return true;
				}

				bool find_atomic_solution(Atomic_Plan_Vec &plans)
				{
					m_t0 = time_used();
					do_atomic_search();
					extract_atomic_plans(m_root, plans);
					if (plans.no_a_goals_achieved())
						return false;
					return true;
				}

				float bound() const { return m_B; }
				void set_bound(float v) { m_B = v; }
				void set_arity(float v) {
					m_heuristic_func->set_arity(v);
					
				}
				void set_greedy(bool b) { m_greedy = b; }
				void set_delay_eval(bool b) { m_delay_eval = b; }

				void inc_gen() { m_gen_count++; }
				unsigned generated() const { return m_gen_count; }
				void inc_eval() { m_exp_count++; }
				unsigned expanded() const { return m_exp_count; }
				void inc_pruned_bound() { m_pruned_B_count++; }
				unsigned pruned_by_bound() const { return m_pruned_B_count; }
				void inc_dead_end() { m_dead_end_count++; }
				unsigned dead_ends() const { return m_dead_end_count; }
				void inc_replaced_open() { m_open_repl_count++; }
				unsigned open_repl() const { return m_open_repl_count; }

				void set_memory_budget_MB(float m) { m_memory_budget = m; }
				float memory_budget() const {return m_memory_budget; }
				void set_budget(float v) { m_time_budget = v; }
				float time_budget() const { return m_time_budget; }

				float atomic_search_time(unsigned atom) { return m_atomic_time_used_map[atom]; }
				unsigned atomic_expanded(unsigned atom) { return m_atomic_exp_count_map[atom]; }
				unsigned atomic_generated(unsigned atom) { return m_atomic_gen_count_map[atom]; }

				float t0() const { return m_t0; }

				void close(Search_Node *n) { m_closed.put(n); }
				Closed_List_Type &closed() { return m_closed; }
				Closed_List_Type &open_hash() { return m_open_hash; }

				const Search_Model &problem() const { return m_problem; }

				void eval(Search_Node *candidate)
				{
					m_heuristic_func->eval(candidate, candidate->hn());
					// if(candidate->hn() >= 3)
					// std::cout << "DEBUG: "<<candidate->hn()<<" -- "<<expanded()<<" -- "<<candidate->gn()<<" -- "<<candidate->fn()<<std::endl;
				}

				bool is_closed(Search_Node *n)
				{
					Search_Node *n2 = this->closed().retrieve(n);

					// std::cout <<"DEBUG: m_B: "<<m_B<<std::endl;
					if (n2 != NULL)
					{
						/*
						* NOT REOPEN FOR NOW
						*/
						return true;

						// if (n2->gn() <= n->gn())
						// {
						// 	// The node we generated is a worse path than
						// 	// the one we already found
						// 	return true;
						// }

						// // Otherwise, we put it into Open and remove
						// // n2 from closed
						// this->closed().erase(this->closed().retrieve_iterator(n2));

						// // MRJ: This solves the memory leak and updates children nodes
						// // incrementally
						// n2->m_parent = n->m_parent;
						// n2->gn() = n->gn();
						// n2->m_action = n->action();

						// if (!m_greedy)
						// 	n2->fn() = n2->hn() + n2->gn();
						// else
						// 	n2->fn() = n2->hn();
						// open_node(n2);
						// return true;
					}
					return false;
				}

				Search_Node *get_node()
				{
					Search_Node *next = NULL;
					if (!m_open.empty())
					{
						next = m_open.pop();
						m_open_hash.erase(m_open_hash.retrieve_iterator(next));
					}
					return next;
				}

				void open_node(Search_Node *n)
				{
					if (n->hn() == infty)
					{
						close(n);
						inc_dead_end();
					}
					else
					{
						m_open.insert(n);
						m_open_hash.put(n);
					}
					inc_gen();
				}

				/**
				 * Succ Generator Process
				 */
				virtual void process(Search_Node *head)
				{

#ifdef DEBUG
					std::cout << "Expanding:" << std::endl;
					head->print(std::cout);
					std::cout << std::endl;
					head->state()->print(std::cout);
					std::cout << std::endl;
#endif
					std::vector<aptk::Action_Idx> app_set;
					this->problem().applicable_set_v2(*(head->state()), app_set);

					for (unsigned i = 0; i < app_set.size(); ++i)
					{
						int a = app_set[i];

						State *succ = m_problem.next(*(head->state()), a);

						Search_Node *n = new Search_Node(succ, m_problem.cost(*(head->state()), a), a, head);

#ifdef DEBUG
						std::cout << "Successor:" << std::endl;
						n->print(std::cout);
						std::cout << std::endl;
						n->state()->print(std::cout);
						std::cout << std::endl;
#endif

						if (is_closed(n))
						{
#ifdef DEBUG
							std::cout << "Already in CLOSED" << std::endl;
#endif
							delete n;
							continue;
						}
						if (m_delay_eval)
							n->hn() = head->hn();
						else
							eval(n);

						// if (m_greedy)
						// 	n->fn() = n->hn();
						// else
						// 	n->fn() = n->hn() + n->gn();
						n->fn() = n->hn();

						if (previously_hashed(n))
						{
#ifdef DEBUG
							std::cout << "Already in OPEN" << std::endl;
#endif
							delete n;
						}

						else
						{
#ifdef DEBUG
							std::cout << "Inserted into OPEN" << std::endl;
#endif
							open_node(n);
						}
					}
					inc_eval();
				}

				virtual Search_Node *do_search()
				{
					std::cout << "Search starts: " << std::endl;
					std::cout << "\t Bound: " << bound() << std::endl;
					float min_h = infty;

					Search_Node *head = get_node();
					int counter = 0;

					struct rusage usage_report;
					while (head)
					{
						/**
						 * check memory usage if > than threshold, if larger then throw exception or something
						 * TODO: Set memory limit through a passed variable
						*/
						if (counter % 100000 == 0){
							getrusage(RUSAGE_SELF, &usage_report);
							std::cout<<"DEBUG: MEMORY MEASUREMENT: "<< (usage_report.ru_maxrss / 1024) <<std::endl;
							if ((usage_report.ru_maxrss / 1024) > m_memory_budget) {
								// std::cout<<"DEBUG: MEMORY MEASUREMENT EXCEED LIMIT: counterval: "<<counter<<std::endl;
								// std::cout <<(usage_report.ru_maxrss / 1024)<<std::endl;
								std::cout << "Search: Memory limit exceeded." << std::endl;
								return NULL;
							}
						}

						if (head->gn() >= bound())
						{
							inc_pruned_bound();
							close(head);
							head = get_node();
							continue;
						}

						if (m_problem.goal(*(head->state())))
						{
							close(head);
							set_bound(head->gn());
							return head;
						}

						if ((time_used() - m_t0) > m_time_budget)
							return NULL;

						if (m_delay_eval)
							eval(head);

						if (head->hn() < min_h)
						{
							min_h = head->hn();
							std::cout << "\t Best h(n) = " << min_h << ", expanded: " << expanded() << ", generated: " << generated() << std::endl;
						}

						process(head);
						close(head);
						counter++;
						head = get_node();
					}
					return NULL;
				}

				/*
			 	Process with atomic goal checking
				*/
				bool process_atomic(Search_Node *head)
				{

#ifdef DEBUG
					std::cout << "Expanding:" << std::endl;
					head->print(std::cout);
					std::cout << std::endl;
					head->state()->print(std::cout);
					std::cout << std::endl;
#endif
					std::vector<aptk::Action_Idx> app_set;
					this->problem().applicable_set_v2(*(head->state()), app_set);

					for (unsigned i = 0; i < app_set.size(); ++i)
					{
						int a = app_set[i];

						State *succ = m_problem.next(*(head->state()), a);

						Search_Node *n = new Search_Node(succ, m_problem.cost(*(head->state()), a), a, head);

#ifdef DEBUG
						std::cout << "Successor:" << std::endl;
						n->print(std::cout);
						std::cout << std::endl;
						n->state()->print(std::cout);
						std::cout << std::endl;
#endif

						if (is_closed(n))
						{
#ifdef DEBUG
							std::cout << "Already in CLOSED" << std::endl;
#endif
							delete n;
							continue;
						}
						if (m_delay_eval)
							n->hn() = head->hn();
						else
							eval(n);

						// if (m_greedy)
						// 	n->fn() = n->hn();
						// else
						// 	n->fn() = n->hn() + n->gn();
						n->fn() = n->hn();

						if (previously_hashed(n))
						{
#ifdef DEBUG
							std::cout << "Already in OPEN" << std::endl;
#endif
							delete n;
						}

						else
						{
							//check if has atomic goal
							if (has_additional_atomic_goal(n))
							{
								if (achieved_all_atomic_goals())
								{
									close(n);
									set_bound(n->gn());
									return true;
								}
							}
#ifdef DEBUG
							std::cout << "Inserted into OPEN" << std::endl;
#endif
							open_node(n);
						}
					}
					inc_eval();
					return false;
				}

				// Search_Node *do_atomic_search()
				void do_atomic_search()
				{
					std::cout << "Search starts: " << std::endl;
					std::cout << "\t Bound: " << bound() << std::endl;
					float min_h = infty;

					Search_Node *head = get_node();
					int counter = 0;

					struct rusage usage_report;
					int prev_gen_val = 0;

					//Check for atomic goals in root node!!
					if (has_additional_atomic_goal(head))
					{
						if (achieved_all_atomic_goals())
						{
							close(head);
							set_bound(head->gn());
							return;
						}
					}

					while (head)
					{
						/**
						 * check memory usage if > than threshold, if larger then throw exception or something
						 * TODO: Set memory limit through a passed variable
						*/
						// std::cout << "DEBUG: "<<" -- "<<expanded()<<" -- "<<head->hn()<<head->gn()<<" -- "<<head->fn()<<std::endl;

						if (generated() > prev_gen_val + 100000){
							prev_gen_val = generated();
						// if (counter % 1000 == 0){
							getrusage(RUSAGE_SELF, &usage_report);
							// if (counter % 100000 == 0)
							std::cout<<"DEBUG: MEMORY MEASUREMENT: "<< (usage_report.ru_maxrss / 1024) <<std::endl;
							if ((usage_report.ru_maxrss / 1024) > m_memory_budget) {
								// std::cout<<"DEBUG: MEMORY MEASUREMENT EXCEED LIMIT: counterval: "<<counter<<std::endl;
								// std::cout <<(usage_report.ru_maxrss / 1024)<<std::endl;
								std::cout << "Search: Memory limit exceeded." << std::endl;
								return;
							}
						}

						if (head->gn() >= bound())
						{
							inc_pruned_bound();
							close(head);
							head = get_node();
							continue;
						}

						// if (has_additional_atomic_goal(head))
						// {
						// 	if (achieved_all_atomic_goals())
						// 	{
						// 		close(head);
						// 		set_bound(head->gn());
						// 		return head;
						// 	}
						// }

						// if (m_problem.goal(*(head->state())))
						// {
						// 	close(head);
						// 	set_bound(head->gn());
						// 	return head;
						// }
						if ((time_used() - m_t0) > m_time_budget)
							return;

						if (m_delay_eval)
							eval(head);

						if (head->hn() < min_h)
						{
							min_h = head->hn();
							std::cout << "\t Best h(n) = " << min_h << ", expanded: " << expanded() << ", generated: " << generated() << std::endl;
						}

						bool found_all_a_goals = process_atomic(head);
						if (found_all_a_goals)
							return;
						close(head);
						counter++;
						head = get_node();
					}
					return;
				}

				virtual bool previously_hashed(Search_Node *n)
				{
					Search_Node *previous_copy = NULL;

					if ((previous_copy = m_open_hash.retrieve(n)))
					{
						if (n->gn() < previous_copy->gn())
						{
							// MRJ: Updates are only possible if we're using a dynamic heap
							// like boost::fibonacci_heap, otherwise, if we generate a better
							// node we need to suck it up and put it into OPEN

							/*
							previous_copy->m_parent = n->m_parent;
							previous_copy->m_action = n->m_action;
							previous_copy->m_g = n->m_g;
							if(!m_greedy)
								previous_copy->m_f = previous_copy->m_h + previous_copy->m_g;
							inc_replaced_open();
							*/
							return false;
						}
						return true;
					}

					return false;
				}

			protected:
				void extract_plan(Search_Node *s, Search_Node *t, std::vector<Action_Idx> &plan, float &cost)
				{
					Search_Node *tmp = t;
					cost = 0.0f;
					while (tmp != s)
					{
						// cost += m_problem.cost(*(tmp->state()), tmp->action());
						cost += 1;
						plan.push_back(tmp->action());
						tmp = tmp->parent();
					}
					std::reverse(plan.begin(), plan.end());
				}


				// void extract_atomic_plans(Search_Node *root, std::vector<std::tuple<unsigned, std::vector<Action_Idx>, cost>> &atomic_plans, float std::vector<cost>)
				// {
				// 	unsigned index = 0;
				// 	for (std::set<unsigned>::iterator atom_i = m_achieved_atomic_goals_set.begin(); atom_i != m_achieved_atomic_goals_set.end(); it++) {
				// 		Search_Node* this_end_node = m_atomic_goals_state_map[*atom_i];
				// 		std::tuple<unsigned, std::vector<Action_Idx>, cost> &this_atom_tuple = atomic_plans[i];
				// 		std::vector<Action_Idx> &this_atom_plan = std::get<1>(this_atom_tuple);
				// 		cost &this_atom_c = std::get<2>(this_atom_tuple);
				// 		//set plan and cost by reference
				// 		extract_plan(root, this_end_node, this_atom_plan, this_atom_c);
				// 		//set atom value
				// 		std::get<0>(this_atom_tuple) = *atom_i;
				// 	}
				// }
				
				void extract_atomic_plans(Search_Node *root, Atomic_Plan_Vec &plan_list)
				{
					unsigned index = 0;
					for (std::set<unsigned>::iterator atom_i = m_achieved_atomic_goals_set.begin(); atom_i != m_achieved_atomic_goals_set.end(); atom_i++) {
						Search_Node* this_end_node = m_atomic_goals_state_map[*atom_i];
						std::tuple<int, std::vector<Action_Idx>, float> &this_atom_tuple = plan_list.get_tuple(index);
						std::vector<Action_Idx> &this_atom_plan = std::get<1>(this_atom_tuple);
						float &this_atom_c = std::get<2>(this_atom_tuple);
						//set plan and cost by reference
						extract_plan(root, this_end_node, this_atom_plan, this_atom_c);
						//set atom value
						std::get<0>(this_atom_tuple) = *atom_i;
						plan_list.increment_a_goals_achieved();
						index++;
					}
				}


				void extract_path(Search_Node *s, Search_Node *t, std::vector<Search_Node *> &plan)
				{
					Search_Node *tmp = t;
					while (tmp != s)
					{
						plan.push_back(tmp);
						tmp = tmp->parent();
					}

					std::reverse(plan.begin(), plan.end());
				}

				bool achieved_all_atomic_goals()
				{
					return m_achieved_atomic_goals_set.size() == m_problem.task().goal().size();
				}

				void record_atomic_info(unsigned f) 
				{
					m_atomic_exp_count_map[f] = expanded();
					m_atomic_gen_count_map[f] = generated();
					m_atomic_pruned_B_count_map[f] = pruned_by_bound();
					m_atomic_dead_end_count_map[f] = dead_ends();
					m_atomic_open_repl_count_map[f] = open_repl();
					m_atomic_time_used_map[f] = time_used() - m_t0;
				}

				bool has_additional_atomic_goal(Search_Node *s) 
				{
					bool new_atomic_goal = false;
					Fluent_Vec added = get_added_atoms(s);
					// STRIPS_Problem strips_model = m_problem.task();
					std::for_each(added.begin(), added.end(), [&](unsigned f) {
						//if atom is a goal atom
						if (m_problem.task().is_in_goal(f)) {
							//if not found in set of achieved goal atoms
							if (m_achieved_atomic_goals_set.find(f) == m_achieved_atomic_goals_set.end())
							{
								std::cout<<"DEBUG: NEW ATOM GOAL ACHIEVED!: "<<f<<std::endl;
								new_atomic_goal = true;
								m_achieved_atomic_goals_set.insert(f);
								m_atomic_goals_state_map[f] = s;
								record_atomic_info(f);
							}
						}
					});
					return new_atomic_goal;
				}

				const Fluent_Vec get_added_atoms(Search_Node *n) const
				{
					if (n->action() == no_op) 
					{
						return n->m_state->fluent_vec();
					}
					else 
					{
						// STRIPS_Problem strips_model = m_problem.task();

						Fluent_Vec new_atom_vec;
						const Action *a = m_problem.task().actions()[n->action()];
						if (a->has_ceff())
						{
							static Fluent_Set new_atom_set(m_problem.task().num_fluents() + 1);
							new_atom_set.reset();
							new_atom_vec.clear();
							for (Fluent_Vec::const_iterator it = a->add_vec().begin(); it != a->add_vec().end(); it++)
							{
								if (new_atom_set.isset(*it))
									continue;

								new_atom_vec.push_back(*it);
								new_atom_set.set(*it);
							}
							for (unsigned i = 0; i < a->ceff_vec().size(); i++)
							{
								Conditional_Effect *ce = a->ceff_vec()[i];
								if (ce->can_be_applied_on(*(n->parent()->state()))){
									for (Fluent_Vec::iterator it = ce->add_vec().begin(); it != ce->add_vec().end(); it++)
									{
										{
											if (new_atom_set.isset(*it))
												continue;

											new_atom_vec.push_back(*it);
											new_atom_set.set(*it);
										}
									}
								}
							}
						}

						const Fluent_Vec &add = a->has_ceff() ? new_atom_vec : a->add_vec();

						return add;
					}
				}

			protected:
				const Search_Model &m_problem;
				Abstract_Heuristic *m_heuristic_func;
				Open_List_Type m_open;
				Closed_List_Type m_closed, m_open_hash;
				unsigned m_exp_count;
				unsigned m_gen_count;
				unsigned m_pruned_B_count;
				unsigned m_dead_end_count;
				unsigned m_open_repl_count;
				float m_B;
				float m_time_budget;
				float m_memory_budget;
				float m_t0;
				Search_Node *m_root;
				std::vector<Action_Idx> m_app_set;
				bool m_greedy;
				bool m_delay_eval;

				std::set<unsigned> m_achieved_atomic_goals_set; //Use Fluent_Set instead? is bitset it more efficient??
				std::unordered_map<unsigned int, Search_Node*> m_atomic_goals_state_map;
				std::unordered_map<unsigned int, unsigned> m_atomic_exp_count_map;
				std::unordered_map<unsigned int, unsigned> m_atomic_gen_count_map;
				std::unordered_map<unsigned int, unsigned> m_atomic_pruned_B_count_map;
				std::unordered_map<unsigned int, unsigned> m_atomic_dead_end_count_map;
				std::unordered_map<unsigned int, unsigned> m_atomic_open_repl_count_map;
				std::unordered_map<unsigned int, float> m_atomic_time_used_map;

			};

		}

	}

}

#endif // gs_custom_bfs.hxx
