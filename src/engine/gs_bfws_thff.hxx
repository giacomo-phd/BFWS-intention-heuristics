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

#ifndef __SINGLE_QUEUE_TRIPLE_HEURISTIC_GREEDY_BEST_FIRST_SEARCH_FAST_EXP_THFF__
#define __SINGLE_QUEUE_TRIPLE_HEURISTIC_GREEDY_BEST_FIRST_SEARCH_FAST_EXP_THFF__

#include <search_prob.hxx>
#include <resources_control.hxx>
#include <closed_list.hxx>
#include <landmark_graph_manager.hxx>
#include <vector>
#include <algorithm>
#include <iostream>
#include <hash_table.hxx>
#include <rp_graph.hxx>
#include <chrono>
#include <cmath>

namespace aptk
{

	namespace search
	{

		namespace gs_bfws_3h_thff
		{

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
						: m_state(s), m_parent(parent), m_action(action), m_g(0), m_g_unit(0), m_h1(0), m_h2(0), 
						m_h3(0.0), m_r(0), m_partition(0), m_M(0), m_land_consumed(NULL), m_land_unconsumed(NULL), 
						m_rp_fl_vec(NULL), m_rp_fl_set(NULL), m_relaxed_deadend(false), m_new_subgoal(false)
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
				unsigned &h2n() { return m_h2; }
				unsigned h2n() const { return m_h2; }
				unsigned &hgc() { return m_gc; }
				unsigned hgc() const { return m_gc; }
				float &h3n() { return m_h3; }
				float h3n() const { return m_h3; } 

				unsigned &r() { return m_r; }
				unsigned r() const { return m_r; }
				unsigned &M() { return m_M; }
				unsigned M() const { return m_M; }
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

				bool &new_subgoal() { return m_new_subgoal; }

				Fluent_Set& seen_rp_fluents() { return m_seen_rp_fluents; }


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

                                void fold_state_vec()
                                {
                                        m_state->fluent_vec() = std::vector<unsigned>();
                                }

                                void collect_state_vec(const Search_Model& problem)
                                {
                                        if (m_state->fluent_vec().size() == 0)
                                        {
                                                for (int f = 0; f < problem.task().num_fluents(); f++)
                                                {
                                                        if (m_state->fluent_set().isset(f))
                                                                m_state->fluent_vec().push_back(f);
                                                }
                                        }
                                }

			public:
				State *m_state;
				Node_Ptr m_parent;
				Action_Idx m_action;
				float m_g;
				unsigned m_g_unit;
				unsigned m_h1;
				unsigned m_h2;
				float m_h3;
				unsigned m_r;
				unsigned m_partition;
				unsigned m_M;
				unsigned m_gc;

				size_t m_hash;
				Bool_Vec_Ptr *m_land_consumed;
				Bool_Vec_Ptr *m_land_unconsumed;
				Fluent_Vec *m_rp_fl_vec;
				Fluent_Set *m_rp_fl_set;

				bool m_relaxed_deadend;

				bool m_new_subgoal;

				Fluent_Set m_seen_rp_fluents;

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
			template <typename Search_Model, typename First_Heuristic, typename Second_Heuristic, typename Third_Heuristic, typename Relevant_Fluents_Heuristic, typename RPG, typename Open_List_Type>
			class GS_BFWS_3H_THFF
			{

			public:
				typedef typename Search_Model::State_Type State;
				typedef typename Open_List_Type::Node_Type Search_Node;
				typedef Closed_List<Search_Node> Closed_List_Type;
				typedef aptk::agnostic::Landmarks_Graph_Manager<Search_Model> Landmarks_Graph_Manager;

				GS_BFWS_3H_THFF(const Search_Model &search_problem, bool verbose)
						: m_problem(search_problem), m_expanded_count_by_novelty(nullptr), m_generated_count_by_novelty(nullptr), m_novelty_count_plan(nullptr), m_exp_count(0), m_gen_count(0), m_dead_end_count(0), m_open_repl_count(0), m_max_depth(infty), m_max_novelty(1), m_time_budget(infty), m_lgm(NULL), m_max_h2n(no_such_index), m_max_r(no_such_index), m_verbose(verbose), m_use_novelty(true), m_use_novelty_pruning(false), m_use_rp(true), m_use_rp_from_init_only(false), 
						m_use_h3n(false), m_h3_only_max_nov(true), m_h3_rp_fl_only(false), m_memory_budget(infty)
				{
					m_first_h = new First_Heuristic(search_problem);
					m_second_h = new Second_Heuristic(search_problem);
					m_third_h = new Third_Heuristic(search_problem);
					m_relevant_fluents_h = new Relevant_Fluents_Heuristic(search_problem);
				}

				virtual ~GS_BFWS_3H_THFF()
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

					delete m_first_h;
					delete m_second_h;
					delete m_relevant_fluents_h;
					if (m_expanded_count_by_novelty != nullptr)
						free(m_expanded_count_by_novelty);
					if (m_generated_count_by_novelty != nullptr)
						free(m_generated_count_by_novelty);
					if (m_novelty_count_plan != nullptr)
						free(m_novelty_count_plan);
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

					auto start_i = std::chrono::steady_clock::now();
					m_rpg = new RPG(m_problem);
					auto start = std::chrono::steady_clock::now();
					m_rpg->compute_cost(m_problem.init()->fluent_vec());

					auto end_comp = std::chrono::steady_clock::now();
					// unsigned aa;
					// m_rpg->extract_ff_cost(m_problem.init()->fluent_set(), aa);
					auto end = std::chrono::steady_clock::now();
					std::chrono::duration<double, std::milli> overall_duration = end - start_i;
					std::chrono::duration<double, std::milli> duration_comp = end_comp - start;
					std::chrono::duration<double, std::milli> duration_instantiation = start - start_i;
					std::chrono::duration<double, std::milli> sampling_duration = end - end_comp;
					std::cout<<std::endl;
					std::cout << "RPG instantiation time: " << duration_instantiation.count()  << " ms"<<std::endl;
					std::cout << "RPG computation time: " << duration_comp.count()  << " ms"<<std::endl;
					std::cout << "ff extract time: " << sampling_duration.count()  << " ms"<<std::endl;
					std::cout << "Overall time: " << overall_duration.count()  << " ms"<<std::endl<<std::endl;



					m_max_depth = B;
					m_root = new Search_Node(m_problem.init(), 0.0f, no_op, NULL, m_problem.num_actions());
					// Init Novelty
					m_first_h->init();
					m_third_h->set_rp_fl_only(m_h3_rp_fl_only);

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
					m_root->seen_rp_fluents().resize(this->problem().task().num_fluents());//, false);
					for (const auto& f : m_root->state()->fluent_vec())
					{
						m_root->seen_rp_fluents().set(f);
					}
					unsigned val;
					m_rpg->extract_ff_cost(m_root->state()->fluent_set(), val);

					// if using the landmark manager to count goals or landmarks
					if (m_lgm)
					{
						m_lgm->apply_state(m_root->state()->fluent_vec(), m_root->land_consumed(), m_root->land_unconsumed());

						eval(m_root);
						m_root->h2n() = m_root->hgc();

						if (m_use_rp)
						{
							eval_rp(m_root);
							// eval_relevant_fluents(m_root);
						}
						if (m_use_novelty)
							// eval_count_based(m_root);
							eval_novel(m_root);

						m_root->undo_land_graph(m_lgm);
							
					}
					else
					{
						eval(m_root);
						m_root->h2n() = m_root->hgc();

						if (m_use_rp)
						{
							eval_rp(m_root);
							// eval_relevant_fluents(m_root);
						}

						if (m_use_novelty)
							// eval_count_based(m_root);
							eval_novel(m_root);
					}

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
					// m_second_h->eval(*(candidate->state()), candidate->h2n());
					m_second_h->eval(*(candidate->state()), candidate->hgc());

					if (candidate->hgc() < m_max_h2n)
					{
						m_max_h2n = candidate->hgc();
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
						if (candidate->parent() && candidate->hgc() < candidate->parent()->hgc())
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

				/*
				The same loop is used to extract achieved facts for the #r and for pcp
				heuristics, to avoid repeating loop redundantly.
				Relaxed state, all facts observed in trajectory from the beginning,
				is extracted for pcp, and m_rpg->extract_ff_cost(rel_state, val);
				calculates the heuristric value.
				*/
				unsigned rp_fl_achieved(Search_Node *n, bool update = false)
				{

					// update = false;

					unsigned count = 0;
					static Fluent_Set counted(this->problem().task().num_fluents());
					Search_Node *n_start = n;

			
					Search_Node* candidate = n;
					float obs_prob_sum;
					Fluent_Set added_rp_fluents(this->problem().task().num_fluents());

					Fluent_Set rel_state(this->problem().task().num_fluents());


					while (!n_start->rp_vec())
					{
						n_start = n_start->parent();
					}

					obs_prob_sum = 0.0f;
					
					if (update)
					{
						candidate->seen_rp_fluents().resize(this->problem().task().num_fluents());//, false);

						for (int i = 0; i<this->problem().task().num_fluents(); i++)
						{
							if (n_start->seen_rp_fluents().isset(i))//at(i))
							{
								candidate->seen_rp_fluents().set(i);//.at(i) = true;
								added_rp_fluents.set(i);
								rel_state.set(i);
							}
						}
					}
					else
					{
						rel_state.set_union(n_start->seen_rp_fluents());
					}
					
					std::vector<float>& fluent_obs_prob = *m_fluent_obs_prob;
					while (n->action() != no_op && n != n_start)
					{

						const Action *a = this->problem().task().actions()[n->action()];

						// Add Conditional Effects
						if (!a->ceff_vec().empty())
						{
							for (unsigned i = 0; i < a->ceff_vec().size(); i++)
							{
								Conditional_Effect *ce = a->ceff_vec()[i];

								bool skip_prec = false;
								for (const auto& prec : ce->prec_vec())
								{
									if (!n->parent() || !n->parent()->state()->fluent_set().isset(prec))
									{
										skip_prec = true;
										break;
									}
								}

								for (auto p : ce->add_vec())
								{
									if (n_start->rp_set()->isset(p) && !counted.isset(p))
									{
										count++;
										counted.set(p);
									}
								
									if (!skip_prec)
									{
										rel_state.set(p);
										if (update)
											candidate->seen_rp_fluents().set(p);
									}
								
								}
							}
						}

						const Fluent_Vec &add = a->add_vec();

						for (unsigned i = 0; i < add.size(); i++)
						{
							const unsigned p = add[i];
							if (n_start->rp_set()->isset(p) && !counted.isset(p))
							{
								count++;
								counted.set(p);
							}

							rel_state.set(p);
							if (update)
								candidate->seen_rp_fluents().set(p);
						}

						n = n->parent();
					}
					counted.reset();

					unsigned val;
					m_rpg->extract_ff_cost(rel_state, val);

					candidate->h3n() = candidate->gn_unit() + val;
					return count;
				}

				void eval_relevant_fluents(Search_Node *candidate)
				{
					if (candidate->parent() && candidate->hgc() < candidate->parent()->hgc())
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


				void update_obs_probs(Search_Node* candidate)
				{
					// if land/goal counter has decreased, then update relevant fluents
					if (candidate->parent() && candidate->hgc() < candidate->parent()->hgc())
					{
						// If state hasn't been gereated, update the parent state with current op
						if (!candidate->has_state())
						{
							static Fluent_Vec added, deleted;
							added.clear();
							deleted.clear();
							candidate->parent()->state()->progress_lazy_state(this->problem().task().actions()[candidate->action()], &added, &deleted);
							calc_observation_probs(candidate, candidate->parent()->state());
							candidate->parent()->state()->regress_lazy_state(this->problem().task().actions()[candidate->action()], &added, &deleted);
						}
						else
							calc_observation_probs(candidate, candidate->state());
					
					}
				}

				void calc_observation_probs(Search_Node *n, State *s)
				{


					m_rpg->reset();
					m_rpg->compute(s->fluent_vec());
					n->set_obs_prob_vec(m_rpg->get_fact_observation_probabilities({}, 10, 10, m_problem.task().goal()));
					n->new_subgoal() = true;

					
					n->h3n() = 0.0f;

				}



				float roundToSigFig(float value, int sigFigs) {
					if (value == 0.0f) return 0.0f;  // Avoid log10(0) issue

					int exponent = std::floor(std::log10(std::fabs(value)));  // Find order of magnitude
					float scale = std::pow(10, sigFigs - 1 - exponent);  // Scaling factor

					return std::round(value * scale) / scale;  // Round and rescale
				}


				void eval_novel(Search_Node *candidate)
				{
					candidate->partition() = (1000 * candidate->hgc()) + candidate->r();
					m_first_h->eval(candidate, candidate->h1n());
				}

				void eval_count_based(Search_Node *candidate)
				{
					candidate->partition() = (1000 * candidate->h2n()) + candidate->r();
					m_third_h->eval(candidate, candidate->h1n());
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
					return next;
				}

				void open_node(Search_Node *n)
				{
					m_open.insert(n);
					inc_gen();
					// m_generated_count_by_novelty[n->h1n() - 1]++;
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
// std::cout << m_expanded_count_by_novelty[0] << " -- "<< m_expanded_count_by_novelty[1] << " -- "<< m_expanded_count_by_novelty[2] << " -- "<< head->h1n()<< " -- "<< head->h2n()<< " -- "<< head->h3n()<< " -- "<< head->r() << " -- "<< head->gn() << std::endl;
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
							eval_relevant_fluents(n);

						if (m_use_novelty)
						{
							// eval_count_based(n);
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
						n->h2n() = n->hgc();

#ifdef DEBUG
						if (m_verbose)
							std::cout << "Inserted into OPEN" << std::endl;
#endif

						static struct rusage usage_report;
						if ((m_memory_budget > 0.0f && m_memory_budget < infty) && generated() % 1000 == 0){
							getrusage(RUSAGE_SELF, &usage_report);
							if ((usage_report.ru_maxrss / 1024.0) > m_memory_budget) {

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
						std::cout << m_expanded_count_by_novelty[0] << " -- "<< m_expanded_count_by_novelty[1] << " -- "<< m_expanded_count_by_novelty[2] << " -- "<< head->h1n()<< " -- "<< head->h2n()<< " -- "<< head->h3n()<< " -- "<< head->r() << " -- "<< head->gn_unit() << std::endl;
				}

				virtual Search_Node *do_search()
				{
					Search_Node *head = get_node();

					while (head)
					{
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
							return head;
						}
						if ((time_used() - m_t0) > m_time_budget)
							return NULL;

						if (m_memory_stop)
							return NULL;

						if (is_closed(head))
						{
#ifdef DEBUG
							if (m_verbose)
								std::cout << "Already in CLOSED" << std::endl;
#endif
							delete head;
							head = get_node();
							continue;
						}
						process(head);
						
						close(head);
						head = get_node();
					}
					return NULL;
				}

				void set_arity(float v, unsigned g) { m_first_h->set_arity(v, g); }
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

				void set_use_h3n(bool v) { m_use_h3n = v; }
				void set_use_h3_only_max_nov(bool v) { m_h3_only_max_nov = v; }
				void set_use_h3_rp_fl_only(bool v) {
					m_h3_rp_fl_only = true;
					m_third_h->set_rp_fl_only(m_h3_rp_fl_only);
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

				const unsigned *
				count_solution_nodes_by_novelty() const { return m_novelty_count_plan; }
				const unsigned *
				generated_by_novelty() const { return m_generated_count_by_novelty; }
				const unsigned *
				expanded_by_novelty() const { return m_expanded_count_by_novelty; }

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
				void set_memory_budget(int v) { m_memory_budget = v; }

				void set_tol_max_depth(int d) { m_open.init(d); }
				void set_tol_seed(int s) { m_open.set_seed(s); }
				void set_rpop_seed(int s) { m_rpop_seed = s; }

				float t0() const { return m_t0; }

				void close(Search_Node *n) { m_closed.put(n); }
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
						cost += m_problem.cost(*(tmp->state()), tmp->action());
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
				float m_memory_budget;

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

				bool m_use_h3n;
				bool m_h3_only_max_nov;
				bool m_h3_rp_fl_only;

				std::vector<float>* m_fluent_obs_prob;

				RPG* m_rpg;
				int m_rpop_seed=42;
				bool m_memory_stop = false;
			};

		}

	}

}

#endif // gs_bfws_3h.hxx
