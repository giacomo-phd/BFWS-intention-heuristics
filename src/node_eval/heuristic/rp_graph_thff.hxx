#ifndef __RELAXED_PLAN_GRAPH_THFF__
#define __RELAXED_PLAN_GRAPH_THFF__

#include <search_prob.hxx>
#include <heuristic.hxx>
#include <ext_math.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <deque>
#include <list>
#include <map>
#include <unordered_set>
#include <random>
#include <cassert>
#include <memory.hxx>
#include <chrono>
#include <utility>



namespace aptk
{

	namespace agnostic
	{

        namespace rp_graph_thff
        {
            struct ActionIndexInfo
            {
                bool m_is_ceff;
                unsigned m_action_index;
                unsigned m_ceff_index;
                ActionIndexInfo() : m_action_index(no_such_index), m_ceff_index(no_such_index), m_is_ceff(false){}
                ActionIndexInfo(unsigned action_index ) : m_action_index(action_index), m_ceff_index(no_such_index), m_is_ceff(false) {}
                ActionIndexInfo(unsigned action_index, unsigned ceff_index) : m_action_index(action_index), m_ceff_index(ceff_index), m_is_ceff(true) {}
                ActionIndexInfo(unsigned action_index, unsigned ceff_index, bool is_ceff) : m_action_index(action_index), m_ceff_index(ceff_index), m_is_ceff(is_ceff) {}
            
                bool operator==(const ActionIndexInfo& o) const {
                    return m_is_ceff == o.m_is_ceff && m_action_index == o.m_action_index && m_ceff_index == o.m_ceff_index;
                }
            }; 

            struct AIIHash {
                std::size_t operator()(const ActionIndexInfo& aii) const
                {
                    return std::hash<unsigned>()(aii.m_action_index) ^ (std::hash<unsigned>()(aii.m_ceff_index) << 1);
                }
            };

            struct ActionsLayer 
            {
                std::unordered_set<ActionIndexInfo, AIIHash> m_actions_layer_set;
                ActionsLayer() {}

                ActionIndexInfo add_action(unsigned action_idx)
                {
                    ActionIndexInfo new_aii(action_idx);
                    m_actions_layer_set.insert(new_aii);
                    return new_aii;
                }

                ActionIndexInfo add_ceff(unsigned action_idx, unsigned ceff_idx)
                {
                    ActionIndexInfo new_aii(action_idx, ceff_idx);
                    m_actions_layer_set.insert(new_aii);  //unordered_set does not store duplicates
                    return new_aii;
                }

                void add_AII(ActionIndexInfo& aii)
                {
                    m_actions_layer_set.insert(aii);
                }


                void remove_AII(ActionIndexInfo& aii)
                {
                    m_actions_layer_set.erase(aii);
                }

                std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator remove_AII(std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator& it)
                {
                    return m_actions_layer_set.erase(it);
                }

                std::unordered_set<ActionIndexInfo, AIIHash>& get_layer()
                {
                    return m_actions_layer_set;
                }

                bool in_layer(ActionIndexInfo& aii)
                {
                    return m_actions_layer_set.find(aii) != m_actions_layer_set.end();
                }
            };


			/*
			Object for storing record and infomration on fluents
			*/
            class FluentRecord
            {
            public:
                FluentRecord() : m_max_layer(0) {}
                FluentRecord(unsigned n_fluents, Fluent_Vec goal, Fluent_Vec initial_fluents) : m_max_layer(0)
                {
                    init(n_fluents);
                    set_goal(goal);
                    set_init_fluents(initial_fluents);
                }

                FluentRecord(const FluentRecord& fr) : m_achieved_fluents(fr.m_achieved_fluents), 
                m_achieved_vec(fr.m_achieved_vec), m_first_achieved_layer(fr.m_first_achieved_layer),
                m_fluent_layer_achievers(fr.m_fluent_layer_achievers), m_goal_fluents(fr.m_goal_fluents),
                m_max_layer(fr.m_max_layer), m_actions_layer(fr.m_actions_layer)
                {}


                void init(unsigned n_fluents)
                {
                    m_achieved_fluents.resize(n_fluents, false);
                    m_first_achieved_layer.resize(n_fluents, no_such_index);
                    m_fluent_layer_achievers.resize(n_fluents);

                    m_fluent_min_costs.resize(n_fluents, no_such_index);
                    m_fluent_min_cost_achiever = new std::unordered_map<unsigned, ActionIndexInfo>();
                }

                void set_goal(Fluent_Vec goal_fluents)
                {
                    m_goal_fluents = goal_fluents;
                }

                void set_init_fluents(Fluent_Vec initial_fluents)
                {
                    for (const auto& f : initial_fluents)
                    {
                        m_init_fluents.insert(f);
                    }
                }

                bool is_init_fluent(unsigned fluent_idx)
                {
                    return m_init_fluents.find(fluent_idx) != m_init_fluents.end();
                }

                void add_achiever_action(unsigned fluent_idx, unsigned layer, unsigned action_idx)
                {
                    ActionIndexInfo aii = m_fluent_layer_achievers[fluent_idx][layer].add_action(action_idx);
                    
                    if (m_actions_layer.size() < layer+1)
                        m_actions_layer.resize(layer+1);

                    m_actions_layer[layer].add_AII(aii);
                }

                void add_achiever_action_cost(unsigned fluent_idx, unsigned action_idx, unsigned cost)
                {
                    ActionIndexInfo aii(action_idx);

                    if (is_min_fluent_cost(fluent_idx, cost))
                    {
                        m_fluent_min_costs[fluent_idx] = cost;
                        (*m_fluent_min_cost_achiever)[fluent_idx] = aii;
                        m_action_min_costs[aii] = cost;
                    }
                }

                void add_achiever_ceff(unsigned fluent_idx, unsigned layer, unsigned action_idx, unsigned ceff_idx)
                {
                    ActionIndexInfo aii = m_fluent_layer_achievers[fluent_idx][layer].add_ceff(action_idx, ceff_idx);
                    if (m_actions_layer.size() < layer+1)
                        m_actions_layer.resize(layer+1);

                    m_actions_layer[layer].add_AII(aii);
                }

                void add_achiever_ceff_cost(unsigned fluent_idx, unsigned action_idx, unsigned ceff_idx, unsigned cost)
                {
                    ActionIndexInfo aii(action_idx, ceff_idx);

                    if (is_min_fluent_cost(fluent_idx, cost))
                    {
                        m_fluent_min_costs[fluent_idx] = cost;
                        (*m_fluent_min_cost_achiever)[fluent_idx] = aii;
                        m_action_min_costs[aii] = cost;
                    }
                }

                bool is_min_fluent_cost(unsigned fluent_idx, unsigned cost)
                {
                    return m_fluent_min_costs[fluent_idx] == no_such_index || cost < m_fluent_min_costs[fluent_idx];
                }

                void update_achieved_fluents(unsigned fluent_idx, unsigned layer)
                {
                    if (layer > m_max_layer)
                        m_max_layer = layer;

                    if ( !achieved(fluent_idx) )
                    {
                        m_achieved_fluents[fluent_idx] = true;
                        m_achieved_vec.push_back(fluent_idx);
                        m_first_achieved_layer[fluent_idx] = layer;
                    }
                }

                void set_achieved_fluent(unsigned fluent_idx)
                {
                    m_achieved_fluents[fluent_idx] = true;
                }

                bool has_achievers(unsigned fluent_idx)
                {
                    if (m_first_achieved_layer[fluent_idx]==0 || m_first_achieved_layer[fluent_idx]==no_such_index)
                        return false;
                    return true;
                }


                //assumes check has_achievers beforehand, otherwise idk for init and unachieved fluents
                const std::unordered_set<ActionIndexInfo, AIIHash>& get_best_fl_achievers(unsigned fluent_idx)
                {
                    return get_fl_achievers(fluent_idx, m_first_achieved_layer[fluent_idx]-1);
                }

                // //layer is the action's layer, not the fluent's
                const std::unordered_set<ActionIndexInfo, AIIHash>& get_fl_achievers(unsigned fluent_idx, unsigned layer)
                {
                    return m_fluent_layer_achievers[fluent_idx][layer].get_layer();
                }

                unsigned get_first_achieved_layer(unsigned fluent_idx)
                {
                    return m_first_achieved_layer[fluent_idx];
                }

                void set_first_achieved_layer(unsigned fluent_idx, unsigned new_layer)
                {
                    m_first_achieved_layer[fluent_idx] = new_layer;
                }

                bool goal_achieved()
                {
                    for (const auto& gi : m_goal_fluents) {
                        if (!m_achieved_fluents[gi]) return false;
                    }
                    return true;
                }

                bool achieved(unsigned fluent_idx) { return m_achieved_fluents[fluent_idx]; }

                const Fluent_Vec& achieved_fluents_vec() { return m_achieved_vec; } 

                const unsigned max_layer() { return m_max_layer; }

                std::vector<ActionsLayer>& get_actions_layer_vec() { return m_actions_layer; }


                //version for iterator object
                std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator update_actions_layer_vec(std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator& it, ActionIndexInfo aii, unsigned old_layer, unsigned new_layer)
                {
                    m_actions_layer[new_layer].add_AII(aii);
                    return m_actions_layer[old_layer].remove_AII(it);
                }

                void update_actions_layer_vec(ActionIndexInfo aii, unsigned old_layer, unsigned new_layer)
                {
                    m_actions_layer[old_layer].remove_AII(aii);
                    m_actions_layer[new_layer].add_AII(aii);
                }

                //assumes old layer is best layer, checked in advance
                void update_fluent_layer_achievers(unsigned fluent_idx, ActionIndexInfo aii, unsigned old_fl_layer, unsigned new_layer)
                {
					m_fluent_layer_achievers[fluent_idx][old_fl_layer].remove_AII(aii);
                    m_fluent_layer_achievers[fluent_idx][new_layer].add_AII(aii);
                }

                std::vector<unsigned>& fluent_min_costs() { return m_fluent_min_costs; }
                std::unordered_map<ActionIndexInfo, unsigned, AIIHash>& action_min_costs() { return m_action_min_costs; }

                ActionIndexInfo& get_best_supporter_action(unsigned fluent_idx)
                {
                    return (*m_fluent_min_cost_achiever)[fluent_idx];
                }

                std::unordered_map<unsigned, ActionIndexInfo>* get_fluent_min_cost_ach_ptr()
                {
                    return m_fluent_min_cost_achiever;
                }
                
            protected:
                std::vector<bool> m_achieved_fluents;
                Fluent_Vec m_achieved_vec;
                std::vector<unsigned> m_first_achieved_layer;
                std::vector<std::map<unsigned, ActionsLayer>> m_fluent_layer_achievers;
                Fluent_Vec m_goal_fluents;
                unsigned m_max_layer;

                std::vector<ActionsLayer> m_actions_layer;

                std::vector<unsigned> m_fluent_min_costs;
                std::unordered_map<ActionIndexInfo, unsigned, AIIHash> m_action_min_costs;

                //not destroyed by this object, ownership passed to outer object before deleting this object
                std::unordered_map<unsigned, ActionIndexInfo>* m_fluent_min_cost_achiever;  

                std::unordered_set<unsigned> m_init_fluents;
            };
        }

        template <typename Search_Model>
        class RelaxedPlanGraphTHFF
        {
        public:
            typedef STRIPS_Problem::Best_Supporter Best_Supporter;
            typedef rp_graph_thff::ActionIndexInfo ActionIndexInfo;
            typedef rp_graph_thff::ActionsLayer ActionsLayer;
            typedef rp_graph_thff::AIIHash AIIHash;
            typedef rp_graph_thff::FluentRecord FluentRecord;



            RelaxedPlanGraphTHFF(const Search_Model &prob) : m_strips_model(prob.task()) 
            {
				m_memory_limit_mb = 8000;;

                m_relevant_actions.resize(m_strips_model.num_fluents());
				m_action_already_achieved.resize(m_strips_model.num_actions());
				m_ceff_already_achieved.resize(m_strips_model.num_actions());

				/* Store relevant actions for each fluent*/
				for (unsigned i = 0; i < m_strips_model.num_actions(); i++)
				{
					const Action &a = *(m_strips_model.actions()[i]);
					m_ceff_already_achieved[i].resize(a.ceff_vec().size(), false);

					// Relevant if the fluent is in the precondition
					for (unsigned j = 0; j < a.prec_vec().size(); ++j)
					{
						m_relevant_actions[a.prec_vec()[j]].insert(i);
					}

					// Relevant if the fluent is in the head of a conditional effect
					for (unsigned j = 0; j < a.ceff_vec().size(); ++j)
					{

						const Conditional_Effect &ceff = *(a.ceff_vec()[j]);

						for (unsigned k = 0; k < ceff.prec_vec().size(); ++k)
						{
							m_relevant_actions[ceff.prec_vec()[k]].insert(i);
						}
					}
				}
            }

			RelaxedPlanGraphTHFF(const RelaxedPlanGraphTHFF& rpg) : m_strips_model(rpg.m_strips_model),
			    m_relevant_actions(rpg.m_relevant_actions), m_action_already_achieved(rpg.m_action_already_achieved),
			    m_ceff_already_achieved(rpg.m_ceff_already_achieved),
			    m_goal_unreachable(rpg.m_goal_unreachable), m_memory_limit_mb(rpg.m_memory_limit_mb),
			    m_start_fluents(rpg.m_start_fluents)
			{
				m_fluent_record = new FluentRecord(*rpg.m_fluent_record);
				// ----------
				// need copy constructor, but m_layer_fluents is not actually used, so just
				// slows down execution. Currently update_rpg does not update m_layer_fluents.
				// m_layer_fluents(rpg.m_layer_fluents), 
				// ----------
			}



			//light copy version only for ff extraction, does not copy or build rpg
			RelaxedPlanGraphTHFF(const Search_Model &prob, std::unordered_map<unsigned, ActionIndexInfo>* fluent_min_cost_achiever_map_ptr, bool goal_unreachable) : m_strips_model(prob.task()) 
			{
				m_fluent_min_cost_achiever_outer = fluent_min_cost_achiever_map_ptr; //pass ownership
				m_goal_unreachable = goal_unreachable;

				m_action_already_achieved = {};
				m_relevant_actions = {};
				m_action_already_achieved = {};
				m_ceff_already_achieved = {};
				//not deleting m_start_fluents idk
				m_fluent_record = nullptr;
			}

            ~RelaxedPlanGraphTHFF() 
			{
				// for (auto it = m_layer_fluents.begin(); it != m_layer_fluents.end(); it++)
				// 	if (*it)
				// 		delete (*it);

				if (m_fluent_record)
					delete m_fluent_record;
				
				if (m_fluent_min_cost_achiever_outer)
					delete m_fluent_min_cost_achiever_outer;
			}

			/*updates rpg from old to new relaxed state, currently not used*/
			void update_rpg(Fluent_Vec new_relaxed_state)
			{
				std::unordered_map<unsigned, unsigned> fluents_to_update;
				for (const auto& fl_i : new_relaxed_state)
				{
					fluents_to_update.insert({fl_i, 0});
				}

				while (!fluents_to_update.empty())
				{
					auto n = fluents_to_update.extract(fluents_to_update.begin());
					unsigned fl_index = n.key();
					unsigned new_layer = n.mapped();
					update_fluent_layer(fl_index, new_layer, fluents_to_update);
				}
			}


			void update_fluent_layer(unsigned fluent_idx, unsigned new_fl_layer, std::unordered_map<unsigned, unsigned>& fluents_to_update)
			{
				//get fluent achieved layer, update its layer to new layer
				//get all actions it achieves (but only in same layer (compared to old layer)?? because if not, it wouldnt be bottleneck anyway...? even if updated other first??)
				unsigned old_fl_layer = m_fluent_record->get_first_achieved_layer(fluent_idx);
				if (old_fl_layer > new_fl_layer)
				{
					m_fluent_record->set_first_achieved_layer(fluent_idx, new_fl_layer);
					
					if (old_fl_layer == m_fluent_record->max_layer())
						return;
					//potential actions induced by fl being a precondition (only in next layer could be bottlenecked by fluent)
					const std::unordered_set<ActionIndexInfo, AIIHash>& al = m_fluent_record->get_actions_layer_vec()[old_fl_layer].get_layer();

					std::unordered_map<unsigned, unsigned> new_action_layer_map;


					for (auto it = al.begin(); it != al.end(); )
					{
						const auto& aii = *it;

						bool fl_is_act_prec = false;
						bool update = false;
						unsigned max_prec_layer_count = 0;
						int second_max_prec_layer = -1;
						for (const unsigned& p : get_preconditions(aii))
						{
							if (p == fluent_idx)
							fl_is_act_prec = true;
							if (m_fluent_record->get_first_achieved_layer(p) == old_fl_layer)
								max_prec_layer_count++;
							else if (m_fluent_record->get_first_achieved_layer(p) > second_max_prec_layer)
								second_max_prec_layer = m_fluent_record->get_first_achieved_layer(p);
						}
						if (fl_is_act_prec && max_prec_layer_count == 1 )
						{
							update = true;
							unsigned new_action_layer = std::max((int)new_fl_layer, second_max_prec_layer);
							//these actions use fluents as precondition, this layer is same as old_fl_layer
							// update_action_layer(aii, old_fl_layer, new_action_layer, fluents_to_update); it++;
							it = update_action_layer(it, aii, old_fl_layer, new_action_layer, fluents_to_update);
						}
						else
							++it;

						//check if fluent in old layer is the only precondition in its current layer
						//if it is, action layer has to be updated, if not then not
						//if action layer has to be updated, new max(new_layer, other preconditions layer) has to be found
					}
				}
				//for actions, get fluents they achieve (in next layer only?!)-->doable through effects (and through m_fluents_layers)
			}


			// version for unordered_set iterator
			std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator update_action_layer(std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator& it, ActionIndexInfo aii, unsigned old_action_layer, unsigned new_action_layer, std::unordered_map<unsigned, unsigned>& fluents_to_update)
			{
				std::unordered_set<ActionIndexInfo, AIIHash>::const_iterator new_it = m_fluent_record->update_actions_layer_vec(it, aii, old_action_layer, new_action_layer);
				//find all fluent adds
				Fluent_Vec adds = get_adds(aii);
				//update action layer in all those fluent m_fluent_actions_layer
				for (const auto& a : adds)
				{
					m_fluent_record->update_fluent_layer_achievers(a, aii, old_action_layer, new_action_layer);
					//if new action achieves fluent in new lowest layer
						//for each fluent, find if the action is in the best old achiever layer
						//if it is, add (fluent, new_layer) to fluent update list
					if ( (m_fluent_record->get_first_achieved_layer(a)-1) > new_action_layer )
					{
						//new fluent layer is new best achiever action layer + 1
						if (fluents_to_update.find(a) != fluents_to_update.end())
						{
							if (new_action_layer+1 < fluents_to_update[a])
								fluents_to_update[a] = new_action_layer+1;
						}
						else
							fluents_to_update.insert({a, new_action_layer+1});

					}
				}
				return new_it;
			}


			void update_action_layer(ActionIndexInfo aii, unsigned old_action_layer, unsigned new_action_layer, std::unordered_map<unsigned, unsigned>& fluents_to_update)
			{
				m_fluent_record->update_actions_layer_vec(aii, old_action_layer, new_action_layer);
				//find all fluent adds
				Fluent_Vec adds = get_adds(aii);
				//update action layer in all those fluent m_fluent_actions_layer
				for (const auto& a : adds)
				{
					m_fluent_record->update_fluent_layer_achievers(a, aii, old_action_layer, new_action_layer);
					//if new action achieves fluent in new lowest layer
						//for each fluent, find if the action is in the best old achiever layer
						//if it is, add (fluent, new_layer) to fluent update list
					if ( (m_fluent_record->get_first_achieved_layer(a)-1) > new_action_layer )
					{
						//new fluent layer is new best achiever action layer + 1
						if (fluents_to_update.find(a) != fluents_to_update.end())
						{
							if (new_action_layer+1 < fluents_to_update[a])
								fluents_to_update[a] = new_action_layer+1;
						}
						else
							fluents_to_update.insert({a, new_action_layer+1});

					}
				}
			}


			
			void reset()
			{
				m_goal_unreachable = false;
				// m_action_already_achieved.resize(m_strips_model.num_actions(), false);
				// m_ceff_already_achieved.resize(m_strips_model.num_actions(), false);
				m_action_already_achieved.assign(m_strips_model.num_actions(), false);
				for (auto& v : m_ceff_already_achieved)
					v.assign(v.size(), false);
				delete m_fluent_record;
				m_fluent_record = nullptr;
				// for (auto it = m_layer_fluents.begin(); it != m_layer_fluents.end(); it++)
				// 	delete (*it);
				// m_layer_fluents.clear();
			}



			/*
			Computes initial RPG, stores the ff-graph, and calculates h^ff of the state.
			*/
            unsigned compute_cost(const Fluent_Vec& fv)
            {

				using FluentCost = std::pair<unsigned, unsigned>;

				m_fluent_record = new FluentRecord(m_strips_model.num_fluents(), m_strips_model.goal(), fv);

				std::priority_queue<FluentCost, std::vector<FluentCost>, std::greater<FluentCost>> fluent_queue;

				for (const auto& start_f : fv)
				{
					fluent_queue.push({0, start_f});
				}

				for (unsigned k = 0; k < m_strips_model.empty_prec_actions().size(); k++)
				{
					const Action &a = *(m_strips_model.empty_prec_actions()[k]);
					
					if (!m_action_already_achieved[a.index()])
					{
						m_action_already_achieved[a.index()] = true;

						for (const auto& f : a.add_vec())
						{
							unsigned cost = 1;
							if (!m_fluent_record->achieved(f))
							{
								fluent_queue.push({cost, f});
							}
							m_fluent_record->add_achiever_action_cost(f, a.index(), cost);
						}
					}

					for (unsigned ceff_idx = 0; ceff_idx < a.ceff_vec().size(); ceff_idx++)
					{
						if (!m_ceff_already_achieved[a.index()][ceff_idx])
						{
							m_ceff_already_achieved[a.index()][ceff_idx] = true;

							const Conditional_Effect &ceff = *(a.ceff_vec()[ceff_idx]);
							if (!ceff.prec_vec().empty())
								continue;
							for (const auto& f : ceff.add_vec())
							{
								unsigned cost = 1;
								if (!m_fluent_record->achieved(f))
								{
									fluent_queue.push({cost, f});
								}
								m_fluent_record->add_achiever_ceff_cost(f, a.index(), ceff_idx, cost);

							}
						}
					}
				}

                bool terminate = false;
				while (!terminate)
				{
					if (fluent_queue.empty())
					{
						terminate = true;
						m_goal_unreachable = true;
						break;
					}

					const auto [fi_cost, fi] = fluent_queue.top();
					fluent_queue.pop();

					if (m_fluent_record->achieved(fi) && m_fluent_record->fluent_min_costs()[fi] <= fi_cost)
						continue;
					
					m_fluent_record->set_achieved_fluent(fi);
					m_fluent_record->fluent_min_costs()[fi] = fi_cost;

					for (std::set<unsigned>::iterator action_it = m_relevant_actions[fi].begin(); action_it != m_relevant_actions[fi].end(); ++action_it)
					{
						const Action &a = *(m_strips_model.actions()[*action_it]);

						unsigned action_prec_cost = 0;

						if (!m_action_already_achieved[a.index()] && action_applies_cost(a, action_prec_cost))
						{
							m_action_already_achieved[a.index()] = true;

							unsigned fl_cost = action_prec_cost+1;

							for (const auto& added_fl: a.add_vec())
							{
								if (m_fluent_record->is_min_fluent_cost(added_fl, fl_cost))
								{
									m_fluent_record->add_achiever_action_cost(added_fl, a.index(), fl_cost);
									fluent_queue.push({fl_cost, added_fl});
								}
							}
						}

						if (m_action_already_achieved[a.index()])
						{
							// Conditional effects
							for (unsigned ceff_idx = 0; ceff_idx < a.ceff_vec().size(); ceff_idx++)
							{
								const Conditional_Effect &ceff = *(a.ceff_vec()[ceff_idx]);

								unsigned ceff_prec_cost = 0;

								if (!m_ceff_already_achieved[a.index()][ceff_idx] && ceff_applies_cost(ceff, ceff_prec_cost))
								{
									m_ceff_already_achieved[a.index()][ceff_idx] = true;

									unsigned fl_cost = action_prec_cost+ceff_prec_cost+1;
									
									for (const auto& ceff_add_fl : ceff.add_vec())
									{
										if (m_fluent_record->is_min_fluent_cost(ceff_add_fl, fl_cost))
										{
											m_fluent_record->add_achiever_ceff_cost(ceff_add_fl, a.index(), ceff_idx, fl_cost);
											fluent_queue.push({fl_cost, ceff_add_fl});
										}
									}
								}
							}
						}
					}

					if (m_fluent_record->goal_achieved())
					{
						terminate = true;
						unsigned h_cost = 0;
						for (const auto& gf : m_strips_model.goal())
						{
							h_cost += get_fluent_cost(gf);
						}
						return h_cost;
					}
				}
				return no_such_index;
            }



			bool action_applies(const Action& a)
			{
				for (const auto& p : a.prec_vec())
				{
					if (!m_fluent_record->achieved(p))
						return false;
				}
				return true;
			}

			bool action_applies_cost(const Action& a, unsigned& cost)
			{
				cost = 0;
				for (const auto& p : a.prec_vec())
				{
					if (!m_fluent_record->achieved(p))
						return false;
					cost += m_fluent_record->fluent_min_costs()[p];
				}
				return true;
			}

			/* assumes action applies */
			bool ceff_applies(const Conditional_Effect& ce)
			{
				for (const auto& p : ce.prec_vec())
				{
					if (!m_fluent_record->achieved(p))
						return false;
				}
				return true;
			}

			/* assumes action applies, cost is only that of ceff preconditions, must be added to action prec cost for full cost */
			bool ceff_applies_cost(const Conditional_Effect& ce, unsigned& conditions_cost)
			{
				conditions_cost = 0;
				for (const auto& p : ce.prec_vec())
				{
					if (!m_fluent_record->achieved(p))
						return false;
					conditions_cost += m_fluent_record->fluent_min_costs()[p];
				}
				return true;
			}


			bool goal_unreachable()
			{
				return m_goal_unreachable;
			}


			//assumes goal reachable
			/* 
			computes the cost given achieved facts in a fluent vector, with respect to the currently stored
			RPG (ff-graph). computes the h_t^{ff} trajectory variant of f^ff mentioned in the paper.
			*/
			void extract_ff_cost(const Fluent_Set& rp_fl_set, unsigned& ff_cost)
			{
				if (m_goal_unreachable)
				{
					ff_cost = no_such_index;
					return;
				}
				// get list / set of fluents that are achieved in relaxed plan to goal (supporter fluents)
				ff_cost = 0;
				std::unordered_set<unsigned> C;
				for (const auto& f : m_strips_model.goal())
				{
					if (!rp_fl_set.isset(f) && !m_fluent_record->is_init_fluent(f))
						C.insert(f);
				}

				std::vector<bool> supporters(m_strips_model.num_fluents(), false);
				std::unordered_set<ActionIndexInfo, AIIHash> achieved_sup_actions;

				while (!C.empty())
				{
					auto n = C.extract(C.begin());
					unsigned f = n.value();
					supporters[f] = true;

					ActionIndexInfo& best_aii = get_best_supporter_action(f);
					if (achieved_sup_actions.find(best_aii) == achieved_sup_actions.end())
					{
						ff_cost++; //add 1 for additional action in ff plan
						achieved_sup_actions.insert(best_aii);
						for (const auto& prec : get_preconditions(best_aii))
						{
							if (!m_fluent_record->is_init_fluent(prec) && !rp_fl_set.isset(prec) && !supporters[prec])
							{
								C.insert(prec);
							}
						}
					}
				}
			}

			ActionIndexInfo& get_best_supporter_action(unsigned fluent_idx)
			{
				if (m_fluent_record)
					return m_fluent_record->get_best_supporter_action(fluent_idx);
				else
					return (*m_fluent_min_cost_achiever_outer)[fluent_idx];
			}

			std::unordered_map<unsigned, ActionIndexInfo>* get_best_supporter_action_map_ptr()
			{
				if (m_fluent_record)
				{
					std::unordered_map<unsigned, ActionIndexInfo>* ret_map = m_fluent_record->get_fluent_min_cost_ach_ptr();
					return ret_map;
				}
				else
					return nullptr;
			}

			unsigned get_fluent_cost(unsigned fluent_idx)
			{
				return m_fluent_record->fluent_min_costs().at(fluent_idx);
			}


		
		protected:
			
			void destroy(std::unordered_map<unsigned, std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>*>& S, std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>* SG)
			{
				for (auto it = SG->begin(); it != SG->end(); it++)
					delete (*it);
				delete SG;
				SG = nullptr;
				for (auto it = S.begin(); it != S.end(); it++)
				{
					for (auto it2 = it->second->begin(); it2 != it->second->end(); it2++)
						delete (*it2);
					delete it->second;
				}

			}

			//less efficient because creates copy of prec vectors, but this 
			//way its easy to add action and ceff precs for ceffs, change for performance
			const Fluent_Vec get_preconditions(ActionIndexInfo aii)
			{
				if (aii.m_is_ceff)
				{
					Fluent_Vec r;
					r.insert(r.end(),get_action(aii)->prec_vec().begin(),get_action(aii)->prec_vec().end());
					r.insert(r.end(), get_ceff(aii)->prec_vec().begin(), get_ceff(aii)->prec_vec().end());
					return r;
				}
				return get_action(aii)->prec_vec();
			}

			const Fluent_Vec get_adds(ActionIndexInfo aii)
			{
				if (aii.m_is_ceff)
				{
					Fluent_Vec r;
					r.insert(r.end(),get_action(aii)->add_vec().begin(),get_action(aii)->add_vec().end());
					r.insert(r.end(), get_ceff(aii)->add_vec().begin(), get_ceff(aii)->add_vec().end());
					return r;
				}
				return get_action(aii)->add_vec();
			}


			const Action* get_action(ActionIndexInfo aii)
			{
				return m_strips_model.actions()[aii.m_action_index];
			}

			Conditional_Effect* get_ceff(ActionIndexInfo aii)
			{
				assert(aii.m_is_ceff);
				return m_strips_model.actions()[aii.m_action_index]->ceff_vec()[aii.m_ceff_index];
			}


			const STRIPS_Problem &m_strips_model;

            std::vector<std::set<unsigned>> m_relevant_actions;

			std::vector<bool> m_action_already_achieved;
			std::vector<std::vector<bool>> m_ceff_already_achieved;

			FluentRecord* m_fluent_record;

			bool m_goal_unreachable = false;

			unsigned m_memory_limit_mb;

			const Fluent_Vec* m_start_fluents;

			std::unordered_map<unsigned, ActionIndexInfo>* m_fluent_min_cost_achiever_outer;  

        };
    }
}


#endif