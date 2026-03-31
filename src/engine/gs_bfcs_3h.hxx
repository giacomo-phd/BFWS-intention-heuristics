/*
Lightweight Automated Planning Toolkit

Copyright 2022
Miquel Ramirez <miquel.ramirez@unimelb.edu.au>
Nir Lipovetzky <nirlipo@gmail.com>

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

#ifndef __SINGLE_QUEUE_TRIPLE_HEURISTIC_GREEDY_BEST_FIRST_COUNT_SEARCH_FAST_EXP__
#define __SINGLE_QUEUE_TRIPLE_HEURISTIC_GREEDY_BEST_FIRST_COUNT_SEARCH_FAST_EXP__

#include <search_prob.hxx>
#include <resources_control.hxx>
#include <closed_list.hxx>
#include <landmark_graph_manager.hxx>
#include <vector>
#include <algorithm>
#include <iostream>
#include <hash_table.hxx>
#include <types.hxx>
#include <memory.hxx>
// #include <chrono>
#include <cstdint>
#include <rp_graph.hxx>
#include <chrono>

namespace aptk
{

	namespace search
	{

		namespace gs_bfcs_3h
		{

			// Custom hash function for std::vector<int>
			struct VectorHash {
				size_t operator()(const std::vector<int>& v) const {
					size_t hash = 0;
					for (int i : v) {
						hash ^= std::hash<int>{}(i) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
					}
					return hash;
				}
				size_t operator()(std::vector<int>& v) const {
					size_t hash = 0;
					for (int i : v) {
						hash ^= std::hash<int>{}(i) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
					}
					return hash;
				}
			};


			// // Custom equality operator for std::vector<int>
			// struct VectorEqual {
			// 	bool operator()(const std::vector<int>& lhs, const std::vector<int>& rhs) const {
			// 		// assert(lhs.size() == rhs.size());
			// 		for (int i = 0; i<lhs.size(); i++)
			// 		{
			// 			if (lhs[i] != rhs[i])
			// 				return false;
			// 		}
			// 		return true;
			// 	}
			// };
			// Custom equality operator for std::vector<int>
			// struct VectorEqual {
			// 	bool operator()(const std::vector<int>& lhs, const std::vector<int>& rhs) const {
			// 		return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
			// 	}
			// };

			template <typename Search_Model, typename State>
			class Node
			{
			public:
				typedef aptk::agnostic::Landmarks_Graph_Manager<Search_Model> Landmarks_Graph_Manager;

				typedef State State_Type;
				typedef Node<Search_Model, State> *Node_Ptr;
				typedef typename std::vector<Node<Search_Model, State> *> Node_Vec_Ptr;
				typedef typename std::vector<Node<Search_Model, State> *>::reverse_iterator Node_Vec_Ptr_Rit;
				typedef typename std::vector<Node<Search_Model, State> *>::iterator Node_Vec_Ptr_It;

				Node(State *s, float cost, Action_Idx action, Node<Search_Model, State> *parent, int num_actions)
						: m_state(s), m_parent(parent), m_action(action), m_g(0), m_g_unit(0), 
						m_h1(0), m_alt_h1(0), m_h2(0), m_h3(0.0), m_r(0), m_partition(0), m_M(0), m_GC(0),
						m_land_consumed(NULL), m_land_unconsumed(NULL), m_rp_fl_vec(NULL), m_rp_fl_set(NULL), m_relaxed_deadend(false),
						m_sign_features(NULL), m_open_delete(0), m_already_expanded(false), m_pop_count(0), m_closed(false),
						m_olp_cc(0), m_olp_cn(0), m_olp_ct(0), m_new_subgoal(false), m_obs_prob_sum(0.0f)//, m_alt2_h1(0), m_alt2_h3(0.0f)//, m_alt(false)
				{
					m_g = (parent ? parent->m_g + cost : 0.0f);
					m_g_unit = (parent ? parent->m_g_unit + 1 : 0);
					update_hash();
				}

				virtual ~Node()
				{
					if (m_state != NULL)
						delete m_state;
					if (m_rp_fl_vec != NULL)
						delete m_rp_fl_vec;
					if (m_rp_fl_set != NULL)
						delete m_rp_fl_set;
				}

				unsigned &h1n() { return m_h1; }
				unsigned h1n() const { return m_h1; }
				unsigned &alt_h1n() { return m_alt_h1; }
				unsigned alt_h1n() const { return m_alt_h1; }
				// unsigned &alt2_h1n() { return m_alt2_h1; }
				// unsigned alt2_h1n() const { return m_alt2_h1; }
				// float &alt2_h3n() { return m_alt2_h3; }
				// float alt2_h3n() const { return m_alt2_h3; }

				unsigned &h2n() { return m_h2; }
				unsigned h2n() const { return m_h2; }
				// unsigned &alt_h2n() { return m_h2; }
				// unsigned alt_h2n() const { return m_h2; }
				// unsigned &h3n() { return m_h3; }
				// unsigned h3n() const { return m_h3; } 
				float &h3n() { return m_h3; }
				float h3n() const { return m_h3; } 

				unsigned &r() { return m_r; }
				unsigned r() const { return m_r; }
				unsigned &M() { return m_M; }
				unsigned M() const { return m_M; }
				unsigned &GC() {return m_GC; }
				unsigned GC() const {return m_GC; }
				unsigned &partition() { return m_partition; }
				unsigned partition() const { return m_partition; }

				float &gn() { return m_g; }
				float gn() const { return m_g; }
				unsigned &gn_unit() { return m_g_unit; }
				unsigned gn_unit() const { return m_g_unit; }

				Node_Ptr parent() { return m_parent; }
				const Node_Ptr parent() const { return m_parent; }
				Action_Idx action() const { return m_action; }
				State *state() { return m_state; }
				void set_state(State *s) { m_state = s; }
				bool has_state() const { return m_state != NULL; }
				const State &state() const { return *m_state; }
				Bool_Vec_Ptr *&land_consumed() { return m_land_consumed; }
				Bool_Vec_Ptr *&land_unconsumed() { return m_land_unconsumed; }
				Fluent_Vec *&rp_vec() { return m_rp_fl_vec; }
				Fluent_Set *&rp_set() { return m_rp_fl_set; }
				bool &relaxed_deadend() { return m_relaxed_deadend; }

				void set_expanded() { m_already_expanded=true; }
				bool already_expanded() { return m_already_expanded; }
				// bool is_alt() { return m_alt; }
				// void set_alt() { m_alt = true; }

				void set_olp_cc(unsigned olp_index) { m_olp_cc = olp_index; }
				void set_olp_cn(unsigned olp_index) { m_olp_cn = olp_index; }
				void set_olp_ct(unsigned olp_index) { m_olp_ct = olp_index; }
				unsigned olp_cc() { return m_olp_cc; }
				unsigned olp_cn() { return m_olp_cn; }	
				unsigned olp_ct() { return m_olp_ct; }

				bool &new_subgoal() { return m_new_subgoal; }
				std::vector<bool> &seen_rp_fluents() { return m_seen_rp_fluents; }
				float &obs_prob_sum() { return m_obs_prob_sum; }	

				// Used to update novelty table
				bool is_better(Node *n) const
				{

					return false;

					// One could mark as novel a tuple that has a better reward, like ICAPS17
					// it is orthogonal, solves more problems in some domains, less in others.

					// return this->gn() < n->gn();
				}

				void update_land_graph(Landmarks_Graph_Manager *lgm)
				{
					Node_Vec_Ptr path(gn_unit() + 1);
					Node_Vec_Ptr_Rit rit = path.rbegin();
					Node_Ptr n = this;

					do
					{
						*rit = n;
						rit++;
						n = n->parent();
					} while (n);
					if (rit != path.rend())
						*rit = NULL;

					lgm->reset_graph();
					for (Node_Vec_Ptr_It it = path.begin(); it != path.end(); it++)
					{

						if (*it == NULL)
							break;
						lgm->update_graph((*it)->land_consumed(), (*it)->land_unconsumed());
					}
				}

				void undo_land_graph(Landmarks_Graph_Manager *lgm)
				{
					lgm->undo_graph(land_consumed(), land_unconsumed());
				}

				void print(std::ostream &os) const
				{
					os << "{@ = " << this << ", s = " << m_state << ", parent = "
						 << ", action_id = " << m_action << ", g(n) = ";
					os << m_g << ", h1(n) = " << m_h1 << ", h2(n) = " << m_h2 << ", r(n) = " << m_r << "}";
				}

				bool operator==(const Node<Search_Model, State> &o) const
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

				size_t hash() const { return m_hash; }

				void update_hash()
				{
					if (m_parent != NULL)
					{
						Hash_Key hasher(m_parent->state()->hash());
						hasher.add(m_action);
						m_hash = (size_t)hasher;
					}
					else
					{
						m_hash = m_state->hash();
					}
				}

			public:
				State *m_state;
				Node_Ptr m_parent;
				Action_Idx m_action;
				float m_g;
				unsigned m_g_unit;
				unsigned m_h1;
				unsigned m_alt_h1;
				// unsigned m_alt2_h1;
				// float m_alt2_h3;
				unsigned m_h2;
				// unsigned m_alt_h2;
				// unsigned m_h3;
				float m_h3;
				unsigned m_r;
				unsigned m_partition;
				unsigned m_M;
				unsigned m_GC;

				unsigned m_olp_cc;
				unsigned m_olp_cn;
				unsigned m_olp_ct;

				size_t m_hash;
				Bool_Vec_Ptr *m_land_consumed;
				Bool_Vec_Ptr *m_land_unconsumed;
				Fluent_Vec *m_rp_fl_vec;
				Fluent_Set *m_rp_fl_set;

				Fluent_Vec m_goals_achieved;
				Fluent_Vec m_goal_candidates;

				bool m_relaxed_deadend;

				const std::vector<int>* m_sign_features;
				int m_open_delete;
				int m_pop_count;
				bool m_already_expanded;
				bool m_closed;
				//bool m_alt;

				bool m_new_subgoal;
				std::vector<bool> m_seen_rp_fluents;
				float m_obs_prob_sum;
			};



			/**
			 * @brief
			 *
			 * @tparam Search_Model
			 * @tparam First_Heuristic
			 * @tparam Second_Heuristic
			 * @tparam Relevant_Fluents_Heuristic
			 * @tparam Open_List_Type
			 */
			template <typename Search_Model, typename First_Heuristic, typename Second_Heuristic, typename Third_Heuristic, typename Relevant_Fluents_Heuristic, typename Open_List_Type>
			class GS_BFCS_3H
			{

			public:
				typedef typename Search_Model::State_Type State;
				typedef typename Open_List_Type::Node_Type Search_Node;
				typedef Closed_List<Search_Node> Closed_List_Type;
				typedef aptk::agnostic::Landmarks_Graph_Manager<Search_Model> Landmarks_Graph_Manager;
				typedef aptk::agnostic::RelaxedPlanGraph<Search_Model> RPG;

				GS_BFCS_3H(const Search_Model &search_problem, bool verbose)
						: m_problem(search_problem), m_expanded_count_by_novelty(nullptr), m_generated_count_by_novelty(nullptr), m_novelty_count_plan(nullptr), 
						m_exp_count(0), m_gen_count(0), m_dead_end_count(0), m_open_repl_count(0), m_max_depth(infty), m_max_novelty(1), m_time_budget(infty), m_lgm(NULL), 
						m_max_h2n(no_such_index), m_max_r(no_such_index), m_verbose(verbose), m_use_novelty(false), m_use_novelty_pruning(false), m_use_rp(true), m_use_rp_from_init_only(false), 
						m_use_h2n(false), m_h3_rp_fl_only(false), m_sign_count(0), m_num_lf_p(0), m_memory_budget(0),
						m_memory_stop(false), m_alt(false)//, m_h3_only_max_nov(true)
				{

					// m_memory_budget = 6000;

					m_first_h = new First_Heuristic(search_problem);
					m_second_h = new Second_Heuristic(search_problem);
					m_third_h = new Third_Heuristic(search_problem);
					m_relevant_fluents_h = new Relevant_Fluents_Heuristic(search_problem);

					//max depth determined size of list (2^17 = 262143)					
					// int OPEN_MAX_DEPTH =18;
					// m_open.init(OPEN_MAX_DEPTH);

					std::unordered_set<std::string> unique_signatures;
					m_fluent_to_feature.resize(this->problem().task().num_fluents());
					unsigned i_val = 0;
					for (const Fluent* f: this->m_problem.task().fluents())
					{
						std::string s = signature_to_lifted_fl(f->signature());
						if (unique_signatures.find(s) == unique_signatures.end())
						{
							unique_signatures.insert(s);
							m_sign_to_int[s] = i_val++;
						}
						m_fluent_to_feature[f->index()] = m_sign_to_int[s];
					}
					m_sign_count = i_val;

					// m_goal_partial_lf_feat = std::vector<unsigned>(m_sign_count, 0);
					// for (auto f: this->problem().task().goal())
					// {
					// 	m_goal_partial_lf_feat[m_fluent_to_feature[f]]++;
					// }

					auto start = std::chrono::steady_clock::now();
					RPG rpg(search_problem);
					rpg.compute(*(search_problem.init()));
					// m_fluent_obs_prob = rpg.get_fact_observation_probabilities_by_value(200, 10, search_problem.task().goal());
					auto end = std::chrono::steady_clock::now();
					std::chrono::duration<double, std::milli> duration = end - start;
					std::cout << "Fluent Observation Probability computation time: " << duration.count()  << " ms"<<std::endl;


				}

				virtual ~GS_BFCS_3H()
				{
					delete_lists_nodes();
					// while (!m_open.empty())
					// {
					// 	Search_Node *n = m_open.pop();
					// 	if ( !n->m_closed && (n->m_pop_count == 2 || n->m_open_delete == 1))
					// 		delete n;
					// 	// else
					// 	// {
					// 	// 	//n->m_pop_count++;
					// 	// 	//n->m_open_delete++;
					// 	// }
					// }
					// for (typename Closed_List_Type::iterator i = m_closed.begin();
					// 		 i != m_closed.end(); i++)
					// {
					// 	delete i->second;
					// }

					// m_closed.clear();

					// delete m_first_h;
					// delete m_second_h;

					// delete m_relevant_fluents_h;
					// if (m_expanded_count_by_novelty != nullptr)
					// 	free(m_expanded_count_by_novelty);
					// if (m_generated_count_by_novelty != nullptr)
					// 	free(m_generated_count_by_novelty);
					// if (m_novelty_count_plan != nullptr)
					// 	free(m_novelty_count_plan);
						

					//no need to delete lifted features count table elements->not allocated dynamically
				}

				void delete_heuristics()
				{
					if (m_first_h != nullptr)
						delete m_first_h;
					m_first_h = nullptr;
					if (m_second_h != nullptr)
						delete m_second_h;
					m_second_h = nullptr;
					if (m_relevant_fluents_h != nullptr)
						delete m_relevant_fluents_h;
					m_relevant_fluents_h = nullptr;
					if (m_expanded_count_by_novelty != nullptr)
						free(m_expanded_count_by_novelty);
					if (m_generated_count_by_novelty != nullptr)
						free(m_generated_count_by_novelty);
					if (m_novelty_count_plan != nullptr)
						free(m_novelty_count_plan);
				}

				void delete_lists_nodes()
				{
					while (!m_open.empty())
					{
						Search_Node *n = m_open.pop();
						if ( !n->m_closed && (n->m_pop_count == 3 || n->m_open_delete == 2))
							delete n;
						// else
						// {
						// 	//n->m_pop_count++;
						// 	//n->m_open_delete++;
						// }
					}
					for (typename Closed_List_Type::iterator i = m_closed.begin();
							 i != m_closed.end(); i++)
					{
						delete i->second;
					}

					m_closed.clear();
				}
				


				/**
				 * Set the relevant fluents from node n
				 * computing a relaxed plan, and marking the fluents
				 * added by actions in relaxed plan as relevant
				 */
				void set_relplan(Search_Node *n, State *s)
				{
					std::vector<Action_Idx> po;
					std::vector<Action_Idx> rel_plan;
					unsigned h = 0;

					m_relevant_fluents_h->ignore_rp_h_value(true);
					m_relevant_fluents_h->eval(*s, h, po, rel_plan);

					if (h == std::numeric_limits<unsigned>::max())
					{ // rel_plan infty
						n->relaxed_deadend() = true;
						return;
					}

#ifdef DEBUG
					for (unsigned p = 0; p < this->problem().task().num_fluents(); p++)
					{
						if (!m_rp_h->is_relaxed_plan_relevant(p))
							continue;
						n->rp_vec()->push_back(p);
						n->rp_set()->set(p);
					}

					std::cout << "rel_plan size: " << rel_plan.size() << " " << std::flush;
#endif
					/**
					 * Reserve space
					 */
					if (!n->rp_vec())
					{
						n->rp_vec() = new Fluent_Vec;
						n->rp_set() = new Fluent_Set(this->problem().task().num_fluents());
					}
					else
					{
						n->rp_vec()->clear();
						n->rp_set()->reset();
					}

					for (std::vector<Action_Idx>::iterator it_a = rel_plan.begin();
							 it_a != rel_plan.end(); it_a++)
					{
						const Action *a = this->problem().task().actions()[*it_a];

						// Add Conditional Effects
						if (!a->ceff_vec().empty())
						{
							for (unsigned i = 0; i < a->ceff_vec().size(); i++)
							{
								Conditional_Effect *ce = a->ceff_vec()[i];
								for (auto p : ce->add_vec())
								{
									if (!n->rp_set()->isset(p))
									{
										n->rp_vec()->push_back(p);
										n->rp_set()->set(p);
#ifdef DEBUG
										std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
									}
								}
							}
						}

						const Fluent_Vec &add = a->add_vec();

#ifdef DEBUG
						std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
#endif
						for (unsigned i = 0; i < add.size(); i++)
						{
							if (!n->rp_set()->isset(add[i]))
							{
								n->rp_vec()->push_back(add[i]);
								n->rp_set()->set(add[i]);
#ifdef DEBUG
								std::cout << this->problem().task().fluents()[add[i]]->signature() << std::endl;
#endif
							}
						}
					}
				}

				virtual void start(float B = infty)
				{
					m_max_depth = B;
					m_root = new Search_Node(m_problem.init(), 0.0f, no_op, NULL, m_problem.num_actions());
					// Init Novelty
					m_third_h->init();
					// m_first_h->set_rp_fl_only(m_h3_rp_fl_only);

					if (m_use_rp)
						set_relplan(this->m_root, this->m_root->state());

					if (m_root->relaxed_deadend())
					{ // rel_plan infty
#ifdef DEBUG
						if (m_verbose)
						{
							std::cout << "h_add is infinite" << std::endl;
						}
#endif
						inc_dead_end();
						return;
						;
					}

					m_root->new_subgoal() = true;
					m_root->seen_rp_fluents().resize(this->problem().task().num_fluents(), false);
					for (const auto& f : m_root->state()->fluent_vec())
					{
						m_root->seen_rp_fluents().at(f) = true;
						m_root->obs_prob_sum() -= std::log( m_fluent_obs_prob[f]+0.0001);
					}
					// m_root->alt2_h3n() = m_root->obs_prob_sum();
					m_root->h3n() = m_root->obs_prob_sum();

					// if using the landmark manager to count goals or landmarks
					if (m_lgm)
					{
						m_lgm->apply_state(m_root->state()->fluent_vec(), m_root->land_consumed(), m_root->land_unconsumed());

						eval(m_root);

						if (m_use_rp)
						{
							eval_rp(m_root);
							eval_relevant_fluents(m_root);
						}
						eval_count_based(m_root);
						// eval_lf_counts(m_root);
						if (m_use_novelty)
							eval_novel(m_root);

						m_root->undo_land_graph(m_lgm);

					}
					else

					{
						eval(m_root);

						if (m_use_rp)
						{
							eval_rp(m_root);
							eval_relevant_fluents(m_root);
						}
						eval_count_based(m_root);
						if (m_use_novelty)
							eval_novel(m_root);
						
					}
					// int tv = get_lifted_counts_state(m_root);
					// std::cout << "DEBUG: " << tv <<std::endl;

#ifdef DEBUG
					if (m_verbose)
					{
						std::cout << "Initial search node: ";
						m_root->print(std::cout);
						std::cout << std::endl;
						m_root->state()->print(std::cout);
						std::cout << std::endl;
					}
#endif

					m_root->set_olp_cc(1);
					m_root->set_olp_cn(1);

					m_open.insert(m_root);

					// m_generated_count_by_novelty[m_root->h1n() - 1]++;
					inc_gen();
				}

				virtual void eval(Search_Node *candidate)
				{

					if (m_lgm)
					{
						// Update land/goal counter up to parent node
						if (candidate->parent())
							candidate->parent()->update_land_graph(m_lgm);

						// Update counter with current operator
						if (candidate->action() != no_op)
						{
							const bool has_cond_eff = !(m_problem.task().actions()[candidate->action()]->ceff_vec().empty());

							// If state hasn't been generated yet, update counter progressing the state of the parent
							if (!candidate->has_state() && has_cond_eff)
							{
								//	candidate->parent()->state()->progress_lazy_state(  m_problem.task().actions()[ candidate->action() ] );

								m_lgm->apply_action(candidate->parent()->state(), candidate->action(), candidate->land_consumed(), candidate->land_unconsumed());

								// candidate->parent()->state()->regress_lazy_state( m_problem.task().actions()[ candidate->action() ] );
							}
							else
							{
								// update the counter with current state
								m_lgm->apply_action(candidate->state(), candidate->action(), candidate->land_consumed(), candidate->land_unconsumed());
							}
						}
						else // If it's the root node, just initialize the counter
							m_lgm->apply_state(m_root->state()->fluent_vec(), m_root->land_consumed(), m_root->land_unconsumed());
					}

					// Count land/goal unachieved
					m_second_h->eval(*(candidate->state()), candidate->GC());
					if (m_use_h2n)
						candidate->h2n() = candidate->GC();

					if (candidate->GC() < m_max_h2n)
					{
						m_max_h2n = candidate->GC();
						m_max_r = 0;
						if (m_verbose)
						{
							std::cout << "--[" << m_max_h2n << " / " << m_max_r << "]--" << std::endl;
						}
						//DEBUG
						std::cout << "--[" << m_max_h2n << " / " << m_max_r << "]--" << std::endl;
						std::cout << "Expanded: "<<expanded()<<"\tGenerated: "<<generated()<<std::endl; 
					}
				}

			void eval_rp(Search_Node *candidate)
			{
				// If relevant fluents are in use
				if (m_use_rp && !m_use_rp_from_init_only)
				{
					// if land/goal counter has decreased, then update relevant fluents
					if (candidate->parent() && candidate->GC() < candidate->parent()->GC())
					{
						// If state hasn't been gereated, update the parent state with current op
						if (!candidate->has_state())
						{
							static Fluent_Vec added, deleted;
							added.clear();
							deleted.clear();
							candidate->parent()->state()->progress_lazy_state(this->problem().task().actions()[candidate->action()], &added, &deleted);
							set_relplan(candidate, candidate->parent()->state());
							candidate->parent()->state()->regress_lazy_state(this->problem().task().actions()[candidate->action()], &added, &deleted);
						}
						else
							set_relplan(candidate, candidate->state());
					}
				}
			}

				// unsigned rp_fl_achieved(Search_Node *n)
				// {
				// 	unsigned count = 0;
				// 	static Fluent_Set counted(this->problem().task().num_fluents());
				// 	Search_Node *n_start = n;
				// 	while (!n_start->rp_vec())
				// 	{
				// 		n_start = n_start->parent();
				// 	}

				// 	while (n->action() != no_op && n != n_start)
				// 	{

				// 		const Action *a = this->problem().task().actions()[n->action()];

				// 		// Add Conditional Effects
				// 		if (!a->ceff_vec().empty())
				// 		{
				// 			for (unsigned i = 0; i < a->ceff_vec().size(); i++)
				// 			{
				// 				Conditional_Effect *ce = a->ceff_vec()[i];
				// 				for (auto p : ce->add_vec())
				// 				{
				// 					if (n_start->rp_set()->isset(p) && !counted.isset(p))
				// 					{
				// 						count++;
				// 						counted.set(p);
				// 					}
				// 				}
				// 			}
				// 		}

				// 		const Fluent_Vec &add = a->add_vec();

				// 		// std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
				// 		for (unsigned i = 0; i < add.size(); i++)
				// 		{
				// 			const unsigned p = add[i];
				// 			if (n_start->rp_set()->isset(p) && !counted.isset(p))
				// 			{
				// 				count++;
				// 				counted.set(p);
				// 			}
				// 		}

				// 		n = n->parent();
				// 	}
				// 	counted.reset();
				// 	return count;
				// }

				// void eval_relevant_fluents(Search_Node *candidate)
				// {
				// 	candidate->r() = rp_fl_achieved(candidate);

				// 	if (candidate->r() > m_max_r)
				// 	{
				// 		m_max_r = candidate->r();
				// 		if (m_verbose)
				// 			std::cout << "--[" << m_max_h2n << " / " << m_max_r << "]--" << std::endl;
				// 	}
				// }


				unsigned rp_fl_achieved(Search_Node *n, bool update = false)
				{
					unsigned count = 0;
					static Fluent_Set counted(this->problem().task().num_fluents());
					Search_Node *n_start = n;

			
					Search_Node* candidate = n;
					float obs_prob_sum;
					Fluent_Set added_rp_fluents(this->problem().task().num_fluents());



					while (!n_start->rp_vec())
					{
						n_start = n_start->parent();
					}


					obs_prob_sum = n_start->h3n();
					if (update)
					{
						candidate->seen_rp_fluents().resize(this->problem().task().num_fluents(), false);
						for (int i = 0; i<this->problem().task().num_fluents(); i++)
						{
							if (n_start->seen_rp_fluents().at(i))
							{
								candidate->seen_rp_fluents().at(i) = true;
								added_rp_fluents.set(i);
							}
						}
					}


					while (n->action() != no_op && n != n_start)
					{

						const Action *a = this->problem().task().actions()[n->action()];

						// Add Conditional Effects
						if (!a->ceff_vec().empty())
						{
							for (unsigned i = 0; i < a->ceff_vec().size(); i++)
							{
								Conditional_Effect *ce = a->ceff_vec()[i];
								for (auto p : ce->add_vec())
								{
									if (n_start->rp_set()->isset(p) && !counted.isset(p))
									{
										count++;
										counted.set(p);
									}
									if (!n_start->seen_rp_fluents().at(p) && !added_rp_fluents.isset(p))
									{
										if (update)
											candidate->seen_rp_fluents().at(p) = true;
										added_rp_fluents.set(p);
										obs_prob_sum -= std::log(m_fluent_obs_prob[p]+0.0001);
									}
								}
							}
						}

						const Fluent_Vec &add = a->add_vec();

						// std::cout << this->problem().task().actions()[*it_a]->signature() << std::endl;
						for (unsigned i = 0; i < add.size(); i++)
						{
							const unsigned p = add[i];
							if (n_start->rp_set()->isset(p) && !counted.isset(p))
							{
								count++;
								counted.set(p);
							}
							if (!n_start->seen_rp_fluents().at(p) && !added_rp_fluents.isset(p))
							{
								if (update)
									candidate->seen_rp_fluents().at(p) = true;
								added_rp_fluents.set(p);
								obs_prob_sum -= std::log(m_fluent_obs_prob[p]+0.0001);
							}
						}

						n = n->parent();
					}
					counted.reset();

					// candidate->alt2_h3n() = obs_prob_sum;
					candidate->h3n() = obs_prob_sum;

					return count;
				}

				void eval_relevant_fluents(Search_Node *candidate)
				{
					if (candidate->parent() && candidate->h2n() < candidate->parent()->h2n())
						candidate->r() = rp_fl_achieved(candidate, true);
					else
						candidate->r() = rp_fl_achieved(candidate, false);

					if (candidate->r() > m_max_r)
					{
						m_max_r = candidate->r();
						if (m_verbose)
							std::cout << "--[" << m_max_h2n << " / " << m_max_r << "]--" << std::endl;
					}
				}

				// void eval_novel(Search_Node *candidate)
				// {
				// 	candidate->partition() = (1000 * candidate->h2n()) + candidate->r();
				// 	m_first_h->eval(candidate, candidate->h1n());
				// }

				void eval_novel(Search_Node *candidate)
				{
					if (candidate->parent() != NULL)
						candidate->partition() = (1000 * candidate->GC()) + 2*candidate->r() + candidate->parent()->olp_cn();
					else
						candidate->partition() = (1000 * candidate->GC()) + 2*candidate->r() + 1;
					
					m_third_h->eval(candidate, candidate->alt_h1n());
					// if (candidate->alt_h1n() < 3)
					// 	candidate->alt2_h1n() = 2;
					// else
					// 	candidate->alt2_h1n() = 3;
				}

				void eval_count_based(Search_Node *candidate)
				{
					//adding olp partitions by making r() even for open list 0 and odd for open list 1 of parent
					if (candidate->parent() != NULL)
						candidate->partition() = (1000 * candidate->GC()) + 2*candidate->r() + candidate->parent()->olp_cc();
					else
						candidate->partition() = (1000 * candidate->GC()) + 2*candidate->r() + 1;

					m_first_h->eval(candidate, candidate->h1n());		
				}

				void eval_lf_counts(Search_Node* n)
				{
					//unsigned lf_count = get_lifted_counts_state(n);
					// if (n->parent() != nullptr && !n->parent()->is_alt())
					unsigned lf_count = get_lifted_counts_state_partition(n);
					n->alt_h1n() = -(float)1 / (1+lf_count);

				}


				// void record_count_h(Search_Node* candidate)
				// {
				// 	int key = (int)(-10000*candidate->h1n());
				// 	if (m_h1_record.find(key) != m_h1_record.end()) {
				// 		// Key found in map, increment its value by 1
				// 		m_h1_record[key]++;
				// 	} else {
				// 		// Key not found in map, set its value to 1
				// 		m_h1_record[key] = 1;
				// 	}
				// }

				void printMap(const std::unordered_map<int, unsigned int>& myMap) {
					std::cout << "--- count h1 values:" <<std::endl;
					for (const auto& pair : myMap) {
						std::cout << "ckey: " << pair.first << " - cvalue: " << pair.second << std::endl;
					}
					std::cout << std::endl;
				}


				bool is_closed(Search_Node *n)
				{
					Search_Node *n2 = this->closed().retrieve(n);

					if (n2 != NULL)
					{
						if (n2->gn() <= n->gn())
						{
							// The node we generated is a worse path than
							// the one we already found
							return true;
						}
						// Otherwise, we put it into Open and remove
						// n2 from closed
						this->closed().erase(this->closed().retrieve_iterator(n2));
					}
					return false;
				}

				Search_Node *get_node()
				{
					Search_Node *next = NULL;
					if (!m_open.empty())
					{
						next = m_open.pop();
					}
					// std::cout << next->olp_cc() << " "
					// 		  << next->olp_hc() << " "
					// 		  << next->olp_cn() << " "
					// 		  << next->olp_hn() << std::endl;
					return next;
				}

				void open_node(Search_Node *n)
				{
					m_open.insert(n);
					inc_gen();
					// m_generated_count_by_novelty[n->h1n() - 1]++;
				}

				// std::string get_Fluent_signature(unsigned ai)
				// {
				// 	return this->problem().task().fluents()[ai]->signature();
				// }

				std::string signature_to_lifted_fl(std::string signature)
				{
					std::stringstream ss(signature);

					std::string lfl;
					if (std::getline(ss, lfl, '_'))
						return lfl;
					return "";
				}

				const std::vector<int>* get_key_ptr(const std::unordered_map<std::vector<int>, unsigned, VectorHash>& myMap, const std::vector<int>& keyToFind) {
					auto it = myMap.find(keyToFind);
					if (it != myMap.end()) {
						return &(it->first); // Return pointer to the key vector
					} else {
						return nullptr; // Key not found, return null pointer
					}
				}

				const std::vector<int>* get_key_ptr(const std::unordered_map<std::vector<int>, uint_fast8_t, VectorHash>& myMap, const std::vector<int>& keyToFind) {
					auto it = myMap.find(keyToFind);
					if (it != myMap.end()) {
						return &(it->first); // Return pointer to the key vector
					} else {
						return nullptr; // Key not found, return null pointer
					}
				}

				unsigned get_lifted_counts_state(Search_Node* n)
				{
					if (n->parent() == NULL) //root node
					{
						std::vector<int> sign_features(m_sign_count, 0);
						for (auto f: n->state()->fluent_vec())
							sign_features[m_fluent_to_feature[f]]++;
						m_sign_feat_occurrences[sign_features] = 1;
						const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, sign_features);
						n->m_sign_features = kp;
						return 0;			
					}
					unsigned feat_count_value;
					
					static Fluent_Vec added, deleted, temp_fv;
					added.clear();
					deleted.clear();	
					n->parent()->state()->progress_lazy_state(this->problem().task().actions()[n->action()], &added, &deleted);
					n->parent()->state()->regress_lazy_state(this->problem().task().actions()[n->action()], &added, &deleted);
					
					const std::vector<int>* parent_features = n->parent()->m_sign_features;
					std::vector<int> child_features(*parent_features);
					std::unordered_set<unsigned> counted_a;
					for (auto f: added)
					{
						if (counted_a.find(f) == counted_a.end())
						{
							counted_a.insert(f);
							if (!n->parent()->state()->entails(f))
								child_features[m_fluent_to_feature[f]]++;
						}
					}
					std::unordered_set<unsigned> counted_d;
					for (auto f: deleted)
					{
						if (counted_d.find(f) == counted_d.end())
						{
							// if (child_features[m_fluent_to_feature[f]] > 0)
							counted_d.insert(f);
							if (n->parent()->state()->entails(f))
								child_features[m_fluent_to_feature[f]]--;
						}
					}
					auto it = m_sign_feat_occurrences.find(child_features);
					if (it != m_sign_feat_occurrences.end())
					{
						feat_count_value = m_sign_feat_occurrences[child_features]++;
						const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, child_features);
						n->m_sign_features = kp;
					}
					else
					{
						m_sign_feat_occurrences[child_features] = 1;
						const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, child_features);
						n->m_sign_features = kp;					
					}

					// std::unordered_set<unsigned> counted;
					// std::vector<int> child_features(m_sign_count, 0);
					// // const std::vector<int>* parent_features = n->parent()->m_sign_features;
					// // std::vector<int> child_features(*parent_features);
					// // for (auto f: added)
					// // 	child_features[m_fluent_to_feature[f]]++;
					// // for (auto f: deleted)
					// // {
					// // 	if (child_features[m_fluent_to_feature[f]] > 0)
					// // 		child_features[m_fluent_to_feature[f]]--;
					// // }
					
					// for (auto f: n->parent()->state()->fluent_vec())
					// {
					// 	if (counted.find(f) != counted.end())
					// 	{
					// 		counted.insert(f);
					// 		child_features[m_fluent_to_feature[f]]++;
					// 	}
					// }
					// auto it = m_sign_feat_occurrences.find(child_features);
					// if (it != m_sign_feat_occurrences.end())
					// {
					// 	feat_count_value = m_sign_feat_occurrences[child_features]++;
					// 	const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, child_features);
					// 	n->m_sign_features = kp;
					// }
					// else
					// {
					// 	m_sign_feat_occurrences[child_features] = 1;
					// 	const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, child_features);
					// 	n->m_sign_features = kp;					
					// }
					return feat_count_value;
				}

				unsigned get_lifted_counts_state_partition(Search_Node* n)
				{
					unsigned partition = n->partition();
					// if (m_sign_feat_partitions.find(partition) == m_sign_feat_partitions.end())
					// {
					// 	m_sign_feat_partitions[partition] = std::unordered_map<std::vector<int>, unsigned int, VectorHash>();
					// }
					
					if (m_sign_feat_partitions.size() <= partition)
					{
						m_sign_feat_partitions.resize(partition + 1);
					}

					if (m_sign_feat_partitions[partition].empty())
					{
						m_sign_feat_partitions[partition] = std::unordered_map<std::vector<int>, uint_fast8_t, VectorHash>();
					}
					
					std::unordered_map<std::vector<int>, uint_fast8_t, VectorHash>& sign_feat_occurrences = m_sign_feat_partitions[partition];


					if (n->parent() == NULL) //root node
					{
						std::vector<int> sign_features(m_sign_count, 0);
						for (auto f: n->state()->fluent_vec())
							sign_features[m_fluent_to_feature[f]]++;
						sign_feat_occurrences[sign_features] = 1;
						const std::vector<int>* kp = get_key_ptr(sign_feat_occurrences, sign_features);
						n->m_sign_features = kp;
						return 0;			
					}
					unsigned feat_count_value;
					
					static Fluent_Vec added, deleted, temp_fv;
					added.clear();
					deleted.clear();
					n->parent()->state()->progress_lazy_state(this->problem().task().actions()[n->action()], &added, &deleted);
					n->parent()->state()->regress_lazy_state(this->problem().task().actions()[n->action()], &added, &deleted);
					
					const std::vector<int>* parent_features = n->parent()->m_sign_features;
					std::vector<int> child_features(*parent_features);
					std::unordered_set<unsigned> counted_a;
					for (auto f: added)
					{
						if (counted_a.find(f) == counted_a.end())
						{
							counted_a.insert(f);
							if (!n->parent()->state()->entails(f))
								child_features[m_fluent_to_feature[f]]++;
						}
					}
					std::unordered_set<unsigned> counted_d;
					for (auto f: deleted)
					{
						if (counted_d.find(f) == counted_d.end())
						{
							// if (child_features[m_fluent_to_feature[f]] > 0)
							counted_d.insert(f);
							if (n->parent()->state()->entails(f))
								child_features[m_fluent_to_feature[f]]--;
						}
					}
					auto it = sign_feat_occurrences.find(child_features);
					if (it != sign_feat_occurrences.end())
					{
						if (sign_feat_occurrences[child_features] < UINT8_MAX)
							feat_count_value = sign_feat_occurrences[child_features]++;
						else
							feat_count_value = UINT8_MAX;

						const std::vector<int>* kp = get_key_ptr(sign_feat_occurrences, child_features);
						n->m_sign_features = kp;
					}
					else
					{
						sign_feat_occurrences[child_features] = 1;
						const std::vector<int>* kp = get_key_ptr(sign_feat_occurrences, child_features);
						n->m_sign_features = kp;					
						feat_count_value = 0;
					}

					// unsigned alt_h2n = 0;
					// for (int i = 0; i<m_sign_count; i++)
					// {
					// 	unsigned gfi = m_goal_partial_lf_feat[i];
					// 	unsigned cfi = child_features[i];
					// 	if (gfi > cfi)
					// 		alt_h2n += ( gfi - cfi );
					// }

					return feat_count_value;
				}

				unsigned get_lifted_counts_p_number(Search_Node* n)
				{
					if (n->parent() == NULL) //root node
					{
						std::vector<int> sign_features(m_sign_count, 0);
						for (auto f: n->state()->fluent_vec())
							sign_features[m_fluent_to_feature[f]]++;
						m_sign_feat_occurrences[sign_features] = 1;
						const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, sign_features);
						n->m_sign_features = kp;

						m_sign_feat_to_p[sign_features] = m_num_lf_p++;
						return 0;
					}
					unsigned feat_count_value;
					
					static Fluent_Vec added, deleted, temp_fv;
					added.clear();
					deleted.clear();
					n->parent()->state()->progress_lazy_state(this->problem().task().actions()[n->action()], &added, &deleted);
					n->parent()->state()->regress_lazy_state(this->problem().task().actions()[n->action()], &added, &deleted);
					
					const std::vector<int>* parent_features = n->parent()->m_sign_features;
					std::vector<int> child_features(*parent_features);
					std::unordered_set<unsigned> counted_a;
					for (auto f: added)
					{
						if (counted_a.find(f) == counted_a.end())
						{
							counted_a.insert(f);
							if (!n->parent()->state()->entails(f))
								child_features[m_fluent_to_feature[f]]++;
						}
					}
					std::unordered_set<unsigned> counted_d;
					for (auto f: deleted)
					{
						if (counted_d.find(f) == counted_d.end())
						{
							// if (child_features[m_fluent_to_feature[f]] > 0)
							counted_d.insert(f);
							if (n->parent()->state()->entails(f))
								child_features[m_fluent_to_feature[f]]--;
						}
					}

					if (m_sign_feat_to_p.find(child_features) == m_sign_feat_to_p.end())
						m_sign_feat_to_p[child_features] = m_num_lf_p++;
					

					auto it = m_sign_feat_occurrences.find(child_features);
					if (it != m_sign_feat_occurrences.end())
					{
						feat_count_value = m_sign_feat_occurrences[child_features]++;
						const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, child_features);
						n->m_sign_features = kp;
					}
					else
					{
						m_sign_feat_occurrences[child_features] = 1;
						const std::vector<int>* kp = get_key_ptr(m_sign_feat_occurrences, child_features);
						n->m_sign_features = kp;					
					}

					return m_sign_feat_to_p[child_features];
					// return feat_count_value;
				}


				virtual void process(Search_Node *head)
				{

#ifdef DEBUG
					
					if (m_verbose)
					{
						std::cout << "Expanding:" << std::endl;
						head->print(std::cout);
						std::cout << std::endl;
						head->state()->print(std::cout);
						std::cout << std::endl;
					}
#endif

					if (m_lgm)
						head->update_land_graph(m_lgm);

					std::vector<aptk::Action_Idx> app_set;
					this->problem().applicable_set_v2(*(head->state()), app_set);

					// Eval RP if needed for the expanded node
					eval_rp(head);
					if (head->relaxed_deadend())
					{ // rel_plan infty
#ifdef DEBUG
						if (m_verbose)
						{
							std::cout << "h_add is infinite" << std::endl;
						}
#endif
						inc_dead_end();
						return;
					}

					for (unsigned i = 0; i < app_set.size(); ++i)
					{
						int a = app_set[i];

						float a_cost = m_problem.cost(*(head->state()), a);

						if (head->gn() + a_cost > m_max_depth)
							continue;

						// Lazy state generation
						State *succ = nullptr;

						Search_Node *n = new Search_Node(succ, a_cost, a, head, m_problem.num_actions());

#ifdef DEBUG
						if (m_verbose)
						{
							std::cout << "Successor:" << std::endl;
							n->print(std::cout);
							std::cout << std::endl;
							if (n->has_state())
								n->state()->print(std::cout);
							std::cout << std::endl;
						}
#endif

						eval(n);
						if (n->relaxed_deadend())
						{ // rel_plan infty
#ifdef DEBUG
							if (m_verbose)
							{
								std::cout << "h_add is infinite" << std::endl;
							}
#endif
							inc_dead_end();
							delete n;
							continue;
						}

						if (m_use_rp)
							// if(n->h2n() == head->h2n())
							eval_relevant_fluents(n);

						eval_count_based(n);
						// eval_lf_counts(n);

						// int tv = get_lifted_counts_state(n);
						// std::cout << "DEBUG: " << tv <<std::endl;
						// std::cout << m_sign_feat_occurrences.size() << std::endl;


						// if (n->h1n() > -0.0001) //if its == 0 (assuming count novelty threshold not allow values <= 0.0001)
						// {
						// 	inc_dead_end();
						// 	delete n;
						// 	continue;
						// }

						if (m_use_novelty)
						{
							eval_novel(n);
							if (m_use_novelty_pruning)
								if (n->h1n() > m_max_novelty)
								{
#ifdef DEBUG
									if (m_verbose)
									{
										std::cout << "h_add is infinite" << std::endl;
									}
#endif
									inc_dead_end();
									delete n;
									continue;
								}
						}


						// if (n->parent() != NULL)
						// {
						// 	n->set_olp_hc(n->parent()->olp_hc());
						// 	n->set_olp_hn(n->parent()->olp_hn());
						// }

						//------
						// if (n->parent() == NULL)
						// 	n->set_olp_hc(0);
						// else
						// 	n->set_olp_hc(n->parent()->olp_c());
						//------





						// if (m_use_h3n) 
						// 	eval_count_based(n);

#ifdef DEBUG
						if (m_verbose)
							std::cout << "Inserted into OPEN" << std::endl;
#endif

						static struct rusage usage_report;
						if (memory_budget_mb() > 0 && generated() % 1000 == 0){
							getrusage(RUSAGE_SELF, &usage_report);
							if ((usage_report.ru_maxrss / 1024) > memory_budget_mb()) {

							std::cout<<"DEBUG: MEMORY MEASUREMENT EXCEED LIMIT: "<<(usage_report.ru_maxrss / 1024)<<std::endl;
							std::cout << "Expanded: "<<expanded()<<"\tGenerated: "<<generated()<<std::endl; 
							m_memory_stop = true;
							}
						}
						open_node(n);
					}
					inc_eval();
					// m_expanded_count_by_novelty[head->h1n() - 1]++;

					//DEBUG
					if ( (m_exp_count % 10000) == 0 )
					{
						std::cout << head->h1n()<< " -- "<< head->h2n()<< " -- "<< head->h3n()<< " -- "
							<< head->GC()<<" -- "<<head->gn_unit() << std::endl;// <<" -- " << m_open.size()<<std::endl;
						std::cout << "Expanded: "<<expanded()<<"\tGenerated: "<<generated()<<std::endl; 
					}
							
				}
  
				virtual Search_Node *do_search()
				{
					Search_Node *head = get_node();
					int counter = 0;

					// static struct rusage usage_report;
					while (head)
					{
						// bool timer = false;
						// if (generated() % 100000 < 100){
						// 	auto start = std::chrono::steady_clock::now();
						// 	getrusage(RUSAGE_SELF, &usage_report);
						// 	auto end = std::chrono::steady_clock::now();
						// 	std::cout<<"DEBUG: MEMORY MEASUREMENT: "<< (usage_report.ru_maxrss / 1024) <<std::endl;
						// 	std::chrono::duration<double, std::milli> duration = end - start;
						// 	std::cout << duration.count() <<std::endl;
						// 	timer = true;
						// 	// if ((usage_report.ru_maxrss / 1024) > m_memory_budget) {
						// 	// 	// std::cout<<"DEBUG: MEMORY MEASUREMENT EXCEED LIMIT: counterval: "<<counter<<std::endl;
						// 	// 	// std::cout <<(usage_report.ru_maxrss / 1024)<<std::endl;
						// 	// 	std::cout << "Search: Memory limit exceeded." << std::endl;
						// 	// 	return NULL;
						// 	// }
						// }
						// auto start = std::chrono::steady_clock::now();
						if (head->already_expanded())
						{
							head = get_node();
							continue;
						}

						// record_count_h(head);
						if (head->gn() >= max_depth())
						{
							close(head);
							head = get_node();
							continue;
						}

						// Generate state
						if (!head->has_state())
							head->set_state(m_problem.next(*(head->parent()->state()), head->action()));

						if (m_problem.goal(*(head->state())))
						{
							close(head);
							set_max_depth(head->gn());
							// printMap(m_h1_record);
							return head;
						}
						if ((time_used() - m_t0) > m_time_budget)
							return NULL;

						if (m_memory_stop)
							return NULL;

						head->set_expanded();

						if (is_closed(head))
						{
#ifdef DEBUG
							if (m_verbose)
								std::cout << "Already in CLOSED" << std::endl;
#endif
							if (head->m_pop_count == 3 || head->m_open_delete == 2)
								delete head;
							else 
							{
								head->m_open_delete++;
								delete head->state();
								head->set_state(nullptr);
							}

							head = get_node();
							continue;
						}
						process(head);
						close(head);
						counter++;
						// auto end = std::chrono::steady_clock::now();
						// if (timer)
						// {
						// 	std::chrono::duration<double, std::milli> duration = end - start;
						// 	std::cout << duration.count() <<std::endl;
						// 	timer = false;
						// }
						head = get_node();
					}
					// printMap(m_h1_record);
					return NULL;
				}

				void set_arity(float v, unsigned g = 0) { 
					m_third_h->set_arity(v, g);
					}

				void set_arity_count(float v, unsigned g = 0) {
					m_first_h->set_arity(v, g);
				}
				// void set_arity_h3(float v, unsigned g = 0) { m_third_h->set_arity(v, g); }
				void set_max_novelty(unsigned v)
				{
					m_max_novelty = v;
					if (m_expanded_count_by_novelty != nullptr)
						free(m_expanded_count_by_novelty);
					m_expanded_count_by_novelty = (unsigned *)calloc(v + 2, sizeof(unsigned));
					if (m_generated_count_by_novelty != nullptr)
						free(m_generated_count_by_novelty);
					m_generated_count_by_novelty = (unsigned *)calloc(v + 2, sizeof(unsigned));
					if (m_novelty_count_plan != nullptr)
						free(m_novelty_count_plan);
					m_novelty_count_plan = (unsigned *)calloc(v + 2, sizeof(unsigned));
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

				float max_depth() const { return m_max_depth; }
				void set_max_depth(float v) { m_max_depth = v; }
				void set_use_rp(bool v) { m_use_rp = v; }
				void set_use_rp_from_init_only(bool v) { m_use_rp_from_init_only = v; }
				void set_use_novelty(bool v) { m_use_novelty = v; }
				void set_use_novelty_pruning(bool v) { m_use_novelty_pruning = v; }

				bool is_alt() { return m_alt; }
				void set_alt(bool b) { m_alt = b; }

				void set_use_h2n(bool v) {m_use_h2n = v; }
				// void set_use_h3_only_max_nov(bool v) { m_h3_only_max_nov = v; }

				// current cbn heuristic version has code related to rp_fl_only commented, purpose
				// was to only evaluate counts of fluents that are present in the #r relaxed plan
				void set_use_count_rp_fl_only(bool v) {
					m_h3_rp_fl_only = true;
					m_first_h->set_rp_fl_only(m_h3_rp_fl_only); //currently does nothing
				}

				unsigned get_max_novelty_expanded()
				{
					for (int i = m_max_novelty + 1; i >= 0; i--)
					{
						if (m_expanded_count_by_novelty[i] > 0)
							return i + 1;
					}
					return 0;
				}
				unsigned get_max_novelty_generated()
				{
					for (int i = m_max_novelty + 1; i >= 0; i--)
					{
						if (m_generated_count_by_novelty[i] > 0)
							return i + 1;
					}
					return 0;
				}

				// const unsigned *
				// count_solution_nodes_by_novelty() const { return m_novelty_count_plan; }
				// const unsigned *
				// // generated_by_novelty() const { return m_generated_count_by_novelty; }
				// const unsigned *
				// expanded_by_novelty() const { return m_expanded_count_by_novelty; }

				void inc_gen() { m_gen_count++; }
				unsigned generated() const { return m_gen_count; }
				void inc_eval() { m_exp_count++; }
				unsigned expanded() const { return m_exp_count; }
				void inc_dead_end() { m_dead_end_count++; }
				unsigned dead_ends() const { return m_dead_end_count; }
				void inc_replaced_open() { m_open_repl_count++; }
				unsigned open_repl() const { return m_open_repl_count; }

				void set_budget(float v) { m_time_budget = v; }
				float time_budget() const { return m_time_budget; }

				void set_memory_budget_mb(int v) { m_memory_budget = v; }
				int memory_budget_mb() const { return m_memory_budget; }

				void set_tol_max_depth(int d) { m_open.init(d); }
				void set_tol_seed(int s) { m_open.set_seed(s); }

				float t0() const { return m_t0; }

				void close(Search_Node *n) 
				{ 
					n->m_closed = true;
					m_closed.put(n); 
					}
				Closed_List_Type &closed() { return m_closed; }

				const Search_Model &problem() const { return m_problem; }

				First_Heuristic &h1() { return *m_first_h; }
				Second_Heuristic &h2() { return *m_second_h; }
				Relevant_Fluents_Heuristic &rel_fl_h() { return *m_relevant_fluents_h; }

				void set_verbose(bool v) { m_verbose = v; }

				void use_land_graph_manager(Landmarks_Graph_Manager *lgm)
				{
					m_lgm = lgm;
					m_second_h->set_graph(m_lgm->graph());
				}

			protected:
				virtual void extract_plan(Search_Node *s, Search_Node *t, std::vector<Action_Idx> &plan, float &cost)
				{
					Search_Node *tmp = t;
					cost = 0.0f;
					while (tmp != s)
					{
						// m_novelty_count_plan[tmp->h1n() - 1]++;
						// cost += m_problem.cost(*(tmp->state()), tmp->action());
						cost++;
						plan.push_back(tmp->action());
						tmp = tmp->parent();
					}

					std::reverse(plan.begin(), plan.end());
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

			protected:
				const Search_Model &m_problem;
				First_Heuristic *m_first_h;
				Second_Heuristic *m_second_h;
				Third_Heuristic *m_third_h;
				Relevant_Fluents_Heuristic *m_relevant_fluents_h;

				Open_List_Type m_open;
				Closed_List_Type m_closed;

				unsigned *m_expanded_count_by_novelty;
				unsigned *m_generated_count_by_novelty;
				unsigned *m_novelty_count_plan;

				unsigned m_exp_count;
				unsigned m_gen_count;
				unsigned m_dead_end_count;
				unsigned m_open_repl_count;

				float m_max_depth;
				unsigned m_max_novelty;
				float m_time_budget;
				float m_t0;

				Search_Node *m_root;
				std::vector<Action_Idx> m_app_set;
				Landmarks_Graph_Manager *m_lgm;

				unsigned m_max_h2n;
				unsigned m_max_r;
				bool m_verbose;

				bool m_use_novelty;
				bool m_use_novelty_pruning;
				bool m_use_rp;
				bool m_use_rp_from_init_only;

				bool m_use_h2n;
				// bool m_h3_only_max_nov;
				bool m_h3_rp_fl_only;

				// std::unordered_map<int, unsigned> m_h1_record;
				int m_sign_count;
				std::unordered_map<std::string, unsigned> m_sign_to_int;
				// std::unordered_map<unsigned, unsigned> m_fluent_to_feature;
				std::vector<unsigned> m_fluent_to_feature;
				std::unordered_map<std::vector<int>, unsigned int, VectorHash> m_sign_feat_occurrences;
				// std::unordered_map<unsigned, std::unordered_map<std::vector<int>, unsigned int, VectorHash>> m_sign_feat_partitions;
				std::vector<std::unordered_map<std::vector<int>, uint_fast8_t, VectorHash>> m_sign_feat_partitions;
				std::unordered_map<std::vector<int>, unsigned int, VectorHash> m_sign_feat_to_p;
				unsigned m_num_lf_p;

				int m_memory_budget;
				bool m_memory_stop;
				bool m_alt;

				std::vector<float> m_fluent_obs_prob;

				// std::vector<unsigned> m_goal_partial_lf_feat;
			};

		}

	}

}

#endif // gs_bfcs_3h.hxx
