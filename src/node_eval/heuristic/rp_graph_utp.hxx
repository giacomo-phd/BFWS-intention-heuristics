#ifndef __RELAXED_PLAN_GRAPH_UTP__
#define __RELAXED_PLAN_GRAPH_UTP__

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



namespace aptk
{

	namespace agnostic
	{
		namespace rp_graph_utp
		{
			// Forward declaration of the RelaxedPlanGraphUTP class
			struct ActionIndexInfo
			{
				const bool m_is_ceff;
				const unsigned m_action_index;
				const unsigned m_ceff_index;
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
				std::vector<ActionIndexInfo> m_actions_layer;

				ActionsLayer() {}

				void add_action(unsigned action_idx)
				{
					m_actions_layer.push_back(ActionIndexInfo(action_idx));
				}

				void add_ceff(unsigned action_idx, unsigned ceff_idx)
				{
					m_actions_layer.push_back(ActionIndexInfo(action_idx, ceff_idx));
				}

				const std::vector<ActionIndexInfo>& get_layer()
				{
					return m_actions_layer;
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


				void init(unsigned n_fluents)
				{
					m_achieved_fluents.resize(n_fluents, false);
					m_first_achieved_layer.resize(n_fluents, no_such_index);
					m_fluent_layer_achievers.resize(n_fluents);
				}

				void set_goal(Fluent_Vec goal_fluents)
				{
					m_goal_fluents = goal_fluents;
				}

				void set_init_fluents(Fluent_Vec initial_fluents)
				{
					for (const auto& f : initial_fluents)
					{
						m_achieved_fluents[f] = true;
						m_achieved_vec.push_back(f);
						m_first_achieved_layer[f] = 0;
						m_fluent_cost[f] = 0;
						m_min_cost_achievers[f].clear();			
					}
				}

				void add_achiever_action(unsigned fluent_idx, unsigned layer, unsigned action_idx)
				{
					m_fluent_layer_achievers[fluent_idx][layer].add_action(action_idx);
				}

				void add_achiever_action(unsigned fluent_idx, unsigned layer, unsigned action_idx, unsigned cost)
				{
					m_fluent_layer_achievers[fluent_idx][layer].add_action(action_idx);

					if (m_fluent_cost.find(fluent_idx) == m_fluent_cost.end())
					{
						m_fluent_cost[fluent_idx] = cost;
						m_min_cost_achievers[fluent_idx].clear();
						m_min_cost_achievers[fluent_idx].push_back(ActionIndexInfo(action_idx));
					}
					else if (m_fluent_cost[fluent_idx] > cost)
					{
						m_fluent_cost[fluent_idx] = cost;
						m_min_cost_achievers[fluent_idx].clear();
						m_min_cost_achievers[fluent_idx].push_back(ActionIndexInfo(action_idx));
					}
					else if (m_fluent_cost[fluent_idx] == cost)
					{
						m_min_cost_achievers[fluent_idx].push_back(ActionIndexInfo(action_idx));
					}
				}


				void add_achiever_ceff(unsigned fluent_idx, unsigned layer, unsigned action_idx, unsigned ceff_idx)
				{
					m_fluent_layer_achievers[fluent_idx][layer].add_ceff(action_idx, ceff_idx);
				}

				void add_achiever_ceff(unsigned fluent_idx, unsigned layer, unsigned action_idx, unsigned ceff_idx, unsigned cost)
				{
					m_fluent_layer_achievers[fluent_idx][layer].add_ceff(action_idx, ceff_idx);
					if (m_fluent_cost.find(fluent_idx) == m_fluent_cost.end())
					{
						m_fluent_cost[fluent_idx] = cost;
						m_min_cost_achievers[fluent_idx].clear();
						m_min_cost_achievers[fluent_idx].push_back(ActionIndexInfo(action_idx));
					}
					else if (m_fluent_cost[fluent_idx] > cost)
					{
						m_fluent_cost[fluent_idx] = cost;
						m_min_cost_achievers[fluent_idx].clear();
						m_min_cost_achievers[fluent_idx].push_back(ActionIndexInfo(action_idx, ceff_idx));
					}
					else if (m_fluent_cost[fluent_idx] == cost)
					{
						m_min_cost_achievers[fluent_idx].push_back(ActionIndexInfo(action_idx, ceff_idx));
					}
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

				bool has_achievers(unsigned fluent_idx)
				{
					if (m_first_achieved_layer[fluent_idx]==0 || m_first_achieved_layer[fluent_idx]==no_such_index)
						return false;
					return true;
				}

				//assumes check has_achievers beforehand, otherwise idk for init and unachieved fluents
				const std::vector<ActionIndexInfo>& get_best_fl_achievers(unsigned fluent_idx)
				{
					return get_fl_achievers(fluent_idx, m_first_achieved_layer[fluent_idx]-1);
				}

				const std::vector<ActionIndexInfo>& get_fl_achievers(unsigned fluent_idx, unsigned layer)
				{
					return m_fluent_layer_achievers[fluent_idx][layer].get_layer();
				}

				unsigned get_first_achieved_layer(unsigned fluent_idx)
				{
					return m_first_achieved_layer[fluent_idx];
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

				unsigned fluent_cost(unsigned f) { return m_fluent_cost[f]; }

				void set_fluent_cost(unsigned f, unsigned c) { m_fluent_cost[f] = c; }

				std::vector<ActionIndexInfo>& min_cost_achievers(unsigned f) { return m_min_cost_achievers[f]; }

				
			protected:
				std::vector<bool> m_achieved_fluents;
				Fluent_Vec m_achieved_vec;
				std::vector<unsigned> m_first_achieved_layer;
				std::vector<std::map<unsigned, ActionsLayer>> m_fluent_layer_achievers;
				Fluent_Vec m_goal_fluents;
				unsigned m_max_layer;
				std::unordered_map<unsigned, unsigned> m_fluent_cost;
				std::unordered_map<unsigned, std::vector<ActionIndexInfo>> m_min_cost_achievers;
			};
		} // namespace rp_graph_utp

        template <typename Search_Model>
        class RelaxedPlanGraphUTP
        {
        public:
            typedef STRIPS_Problem::Best_Supporter Best_Supporter;
			typedef rp_graph_utp::ActionIndexInfo ActionIndexInfo;
			typedef rp_graph_utp::FluentRecord FluentRecord;
			typedef rp_graph_utp::ActionsLayer ActionsLayer;
			typedef rp_graph_utp::AIIHash AIIHash;


            RelaxedPlanGraphUTP(const Search_Model &prob) : m_strips_model(prob.task()) 
            {
				m_memory_limit_mb = 8000;
				m_gen.seed(42);
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

            ~RelaxedPlanGraphUTP() 
			{
				for (auto it = m_layer_fluents.begin(); it != m_layer_fluents.end(); it++)
					delete (*it);

				if (m_fluent_record)
					delete m_fluent_record;
			}

			void reset()
			{
				m_goal_unreachable = false;
				m_action_already_achieved.assign(m_strips_model.num_actions(), false);
				for (auto& v : m_ceff_already_achieved)
					v.assign(v.size(), false);
				delete m_fluent_record;
				m_fluent_record = nullptr;
				for (auto it = m_layer_fluents.begin(); it != m_layer_fluents.end(); it++)
					delete (*it);
				m_layer_fluents.clear();
			}

			void set_seed(std::mt19937::result_type new_seed)
			{
				m_gen.seed(new_seed);
			}

            // void eval(const State& s)
            // {
            //     // m_already_updated.reset();
            //     // m_updated.clear();
                
            //     // initialize(s);
			// 	// compute();

			/* Compute and store the Relaxed Planning Graph*/
            void compute(const Fluent_Vec& s_fl_vec)
            {

				m_fluent_record = new FluentRecord(m_strips_model.num_fluents(), m_strips_model.goal(), s_fl_vec);
                Fluent_Vec* prev_layer_fl = new Fluent_Vec(s_fl_vec);
				Fluent_Vec* curr_layer_fl = new Fluent_Vec();
				m_start_fluents = &s_fl_vec;


				unsigned prev_layer_index = 0;

				for (unsigned k = 0; k < m_strips_model.empty_prec_actions().size(); k++)
				{
					const Action &a = *(m_strips_model.empty_prec_actions()[k]);
					
					if (!m_action_already_achieved[a.index()])
					{
						m_action_already_achieved[a.index()] = true;

						for (const auto& f : a.add_vec())
						{
							if (!m_fluent_record->achieved(f))
							{
								curr_layer_fl->push_back(f);
							}
							m_fluent_record->add_achiever_action(f, prev_layer_index, a.index());
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
								if (!m_fluent_record->achieved(f))
								{
									curr_layer_fl->push_back(f);
								}
								m_fluent_record->add_achiever_ceff(f, prev_layer_index, a.index(), ceff_idx);
							}
						}
					}
				}

                bool terminate = false;
				while (!terminate)
				{
					m_layer_fluents.push_back(prev_layer_fl);

					std::vector<bool> achieved_this_round(m_strips_model.num_fluents(), false);
					for (const auto& fi : *prev_layer_fl)
					{
						for (std::set<unsigned>::iterator action_it = m_relevant_actions[fi].begin(); action_it != m_relevant_actions[fi].end(); ++action_it)
						{
							const Action &a = *(m_strips_model.actions()[*action_it]);

							if (!m_action_already_achieved[a.index()] && action_applies(a))
							{
								m_action_already_achieved[a.index()] = true;

								for (const auto& added_fl: a.add_vec())
								{
									if (!m_fluent_record->achieved(added_fl) && !achieved_this_round[added_fl])
									{
										curr_layer_fl->push_back(added_fl);
										achieved_this_round[added_fl] = true;
									}
									m_fluent_record->add_achiever_action(added_fl, prev_layer_index, a.index());
								}

								// Conditional effects
								for (unsigned ceff_idx = 0; ceff_idx < a.ceff_vec().size(); ceff_idx++)
								{
									const Conditional_Effect &ceff = *(a.ceff_vec()[ceff_idx]);

									if (!m_ceff_already_achieved[a.index()][ceff_idx] && ceff_applies(ceff))
									{
										m_ceff_already_achieved[a.index()][ceff_idx] = true;
										
										for (const auto& ceff_add_fl : ceff.add_vec())
										{
											if (!m_fluent_record->achieved(ceff_add_fl) && !achieved_this_round[ceff_add_fl])
											{
												curr_layer_fl->push_back(ceff_add_fl);
												achieved_this_round[ceff_add_fl] = true;
											}
											m_fluent_record->add_achiever_ceff(ceff_add_fl, prev_layer_index, a.index(), ceff_idx);
										}
									}
								}

							}

						}
					}

					for (const auto& f : *curr_layer_fl)
					{
						m_fluent_record->update_achieved_fluents(f, prev_layer_index+1);	
					}			

					if (curr_layer_fl->size() == 0)
					{
						terminate = true;
						m_goal_unreachable = true;
						continue;
					}

					/* terminate early when all goal conditions achieved*/
					if (m_fluent_record->goal_achieved())
					{
						terminate = true;
						m_layer_fluents.push_back(curr_layer_fl);
					}

					prev_layer_fl = curr_layer_fl;
					prev_layer_index++;

					if (!terminate)
						curr_layer_fl = new Fluent_Vec();
				}

            }

			/* Compute and store the Relaxed Planning Graph*/
            void compute(const State& s)
            {

				m_fluent_record = new FluentRecord(m_strips_model.num_fluents(), m_strips_model.goal(), s.fluent_vec());
                Fluent_Vec* prev_layer_fl = new Fluent_Vec(s.fluent_vec());
				Fluent_Vec* curr_layer_fl = new Fluent_Vec();
				m_start_fluents = &s.fluent_vec();


				unsigned prev_layer_index = 0;

				for (unsigned k = 0; k < m_strips_model.empty_prec_actions().size(); k++)
				{
					const Action &a = *(m_strips_model.empty_prec_actions()[k]);
					
					if (!m_action_already_achieved[a.index()])
					{
						m_action_already_achieved[a.index()] = true;

						for (const auto& f : a.add_vec())
						{
							if (!m_fluent_record->achieved(f))
							{
								curr_layer_fl->push_back(f);
								m_fluent_record->set_fluent_cost(f, 1);
							}
							m_fluent_record->add_achiever_action(f, prev_layer_index, a.index(), 1);
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
								if (!m_fluent_record->achieved(f))
								{
									curr_layer_fl->push_back(f);
									m_fluent_record->set_fluent_cost(f, 1);
								}
								m_fluent_record->add_achiever_ceff(f, prev_layer_index, a.index(), ceff_idx, 1);
							}
						}
					}
				}

                bool terminate = false;
				while (!terminate)
				{
					m_layer_fluents.push_back(prev_layer_fl);

					std::vector<bool> achieved_this_round(m_strips_model.num_fluents(), false);
					for (const auto& fi : *prev_layer_fl)
					{
						for (std::set<unsigned>::iterator action_it = m_relevant_actions[fi].begin(); action_it != m_relevant_actions[fi].end(); ++action_it)
						{
							const Action &a = *(m_strips_model.actions()[*action_it]);
							unsigned action_cost = 0;
							if (!m_action_already_achieved[a.index()] && action_applies(a, action_cost))
							{

								m_action_already_achieved[a.index()] = true;

								for (const auto& added_fl: a.add_vec())
								{
									if (!m_fluent_record->achieved(added_fl) && !achieved_this_round[added_fl])
									{
										curr_layer_fl->push_back(added_fl);
										achieved_this_round[added_fl] = true;
									}
									m_fluent_record->add_achiever_action(added_fl, prev_layer_index, a.index(), action_cost+1);
								}
							}

							// Conditional effects
							for (unsigned ceff_idx = 0; ceff_idx < a.ceff_vec().size(); ceff_idx++)
							{
								const Conditional_Effect &ceff = *(a.ceff_vec()[ceff_idx]);

								unsigned ceff_cost = 0;

								//ceff can apply only if action already achieved (because then will always apply in relaxed graph)
								//then we can check ceff_applies (only checks additional ceff preconditions, not base action precs)
								if (m_action_already_achieved[a.index()] && !m_ceff_already_achieved[a.index()][ceff_idx] && ceff_applies(ceff, ceff_cost))
								{
									m_ceff_already_achieved[a.index()][ceff_idx] = true;
									
									for (const auto& ceff_add_fl : ceff.add_vec())
									{
										if (!m_fluent_record->achieved(ceff_add_fl) && !achieved_this_round[ceff_add_fl])
										{
											curr_layer_fl->push_back(ceff_add_fl);
											achieved_this_round[ceff_add_fl] = true;
										}
										m_fluent_record->add_achiever_ceff(ceff_add_fl, prev_layer_index, a.index(), ceff_idx, action_cost+ceff_cost+1);
									}
								}
							}

						}
					}

					for (const auto& f : *curr_layer_fl)
					{
						m_fluent_record->update_achieved_fluents(f, prev_layer_index+1);	
					}			

					if (curr_layer_fl->size() == 0)
					{
						terminate = true;
						m_goal_unreachable = true;
						continue;
					}

					if (m_fluent_record->goal_achieved())
					{
						terminate = true;
						m_layer_fluents.push_back(curr_layer_fl);
					}

					prev_layer_fl = curr_layer_fl;
					prev_layer_index++;

					if (!terminate)
						curr_layer_fl = new Fluent_Vec();
				}
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

			bool action_applies(const Action& a, unsigned& cost)
			{
				cost = 0;
				for (const auto& p : a.prec_vec())
				{
					if (!m_fluent_record->achieved(p))
						return false;
					else
						cost += m_fluent_record->fluent_cost(p);
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

			/* assumes action applies */
			bool ceff_applies(const Conditional_Effect& ce, unsigned& cost)
			{
				cost = 0;
				for (const auto& p : ce.prec_vec())
				{
					if (!m_fluent_record->achieved(p))
						return false;
					else
						cost += m_fluent_record->fluent_cost(p);
				}
				return true;
			}


			const std::vector<ActionIndexInfo>& get_best_layer_achievers_vector(unsigned fluent_idx)
			{
				return m_fluent_record->get_best_fl_achievers(fluent_idx);
			}

			/*
				Samples sets of supporter actions for plans achieving the goal from the RPG.
				This is based off Algorithm 2 in (Wilken et Al. 2024), with modifications to 
				suit this case. Supporter actions are extracted to achieve the full goal
				condition in one go, rather than each goal fact separately, and potential
				supporters is only the set of supporters with min h_add heuristic.
			*/
			void sample_subgoal_supporter_actions(const Fluent_Vec& fv, int& N, Fluent_Vec& subgoals, unsigned max_ms_limit, std::vector<std::vector<ActionIndexInfo>*>*& S) 
			{
				auto begin_time = std::chrono::steady_clock::now();

				std::vector<bool> start(m_strips_model.num_fluents(), false);

				for (const auto& fi : fv)
					start[fi] = true;

				std::unordered_map<unsigned, std::unordered_map<ActionIndexInfo, unsigned, AIIHash>> subgoal_counts;
				
				S = new std::vector<std::vector<ActionIndexInfo>*>();

				for (int i=0; i<N; i++)
				{
					{
						std::unordered_map<ActionIndexInfo, unsigned, AIIHash>& counts = subgoal_counts[0];
						std::unordered_set<unsigned> C;
						for (const auto& g_i : subgoals)
							C.insert(g_i);

						std::vector<bool> found(m_strips_model.num_fluents(), false);

						std::vector<ActionIndexInfo>* sups = new std::vector<ActionIndexInfo>();

						for (int l=m_fluent_record->max_layer(); l>=0; l--)
						{
							std::unordered_set<unsigned> new_C;
							while (C.size() > 0)
							{
								auto n = C.extract(C.begin());
								unsigned p = n.value();

								if (m_fluent_record->has_achievers(p))
								{
									const std::vector<ActionIndexInfo>& psups = m_fluent_record->min_cost_achievers(p);
									const ActionIndexInfo& aii = min_count(psups, counts);

									sups->push_back(aii);
									counts[aii]++;

									const Fluent_Vec precs = get_preconditions(aii);

									for (const auto& f : precs)
									{
										if ( !start[f] && !found[f] && (C.find(f)==C.end()) )
											new_C.insert(f);
									}

									//line 32 in alg.2, must also (for robustness, dumb border case of action adding its precondition
									//but still possible i think) check that fluent added is not in preconditions, otherwise it deletes it
									//it from C & new_C and never discover/support it
									for (const auto& f : get_adds(aii)) 
									{
										//ensure not erasing added if for any reason it is also a precondition
										if (std::find(precs.begin(), precs.end(), f) == precs.end())
										{
											if (C.find(f) != C.end())
												C.erase(f);
											if (new_C.find(f) != new_C.end())
												new_C.erase(f);
										}
									}
								}
								found[p] = true;
								if (C.find(p) != C.end())
									C.erase(p);
							}
							C.merge(new_C);
						}

						S->push_back(sups);
					}

					//check time limit
					auto lap = std::chrono::steady_clock::now();
					std::chrono::duration<double, std::milli> duration = lap - begin_time;
					if ((unsigned)duration.count() > max_ms_limit)
					{
						N = i+1;
						break;
					}

					//check memory limit
					static struct rusage usage_report;
					if (m_memory_limit_mb > 0){
						getrusage(RUSAGE_SELF, &usage_report);
						if ((usage_report.ru_maxrss / 1024) > m_memory_limit_mb) {

							// std::cout<<"DEBUG: Obs. Prob. sampling: MEMORY MEASUREMENT EXCEED LIMIT: "<<(usage_report.ru_maxrss / 1024)<<std::endl;
							N = i+1;
							break;
						}
					}
				}
				return;
				// return samples;
			}


			/*creates supporter sets from sets of supporter actions
			since we directly extract supporter actions for full goals this step is a bit redundant right 
			now, but it is still used */
			std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>* generate_supporter_sets(unsigned N, std::unordered_map<unsigned, std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>*>& S)
			{
				Fluent_Vec g = m_strips_model.goal();
				std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>* SG = new std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>();
				std::unordered_set<ActionIndexInfo, AIIHash>* SG_i;
				
				for (int i=0; i<N; i++)
				{
					SG_i = new std::unordered_set<ActionIndexInfo, AIIHash>();
					// for (const auto& g_f : g)
					// {
					// 	//remove a set from S[g_f] and merge it into SG_i
					// 	auto* rs = pick_erase_random_set(S[g_f]);
					// 	SG_i->merge(*rs); //elements *rs are stored inside S[g_f] so no need to delete SG_i
					// 	delete rs;
					// }
						auto* rs = pick_erase_random_set(S[0]);
						SG_i->merge(*rs); //elements *rs are stored inside S[g_f] so no need to delete SG_i
						delete rs;
					SG->push_back(SG_i);
					SG_i = nullptr;
				}
				return SG;
			}

			/*
			calculates the number of new actions that apply to a relaxed state (i.e. whose preconditions are satisfied by relaxed state)
			*/
			unsigned num_new_action_applies_to_rs(Fluent_Set rs, std::vector<unsigned> added_fluents, std::unordered_set<unsigned>& counted_actions)
			{
				unsigned num = 0;
				for (const auto af : added_fluents)
				{
					std::set<unsigned>& a_set = m_relevant_actions[af];
					for (const auto& ai : a_set)
					{
						if (counted_actions.find(ai) == counted_actions.end())
						{
							bool applies = true;
							const Action* a = m_strips_model.actions()[ai];
							for (const auto& prec_i : a->prec_vec())
							{
								if (!rs.isset(prec_i))
								{
									applies = false;
									break;
								}
							}
							if (applies)
							{
								num++;
								counted_actions.insert(ai);
							}
						}
					}
				}
				return num;
			}

			/*
			Function reweights relaxed plans according to Uniform Transition Probabilities, to obtain UTP weights. At each step, 
			probabilities are divided by number of aoplicable actions. Since these are relaxed states, this probability becomes 
			very small very fast, as such we use log probabilities to deal with these values and avoid overflows, and then re-normalize them.
			*/
			void reweigh_rel_plans(std::vector<std::vector<ActionIndexInfo>*>*& SG, std::vector<float>& rp_weights, float& total_weights)
			{

				double total_agg_log_prob = 0;
				std::vector<float> temp_weights;

				for (int rpi=0; rpi<SG->size(); rpi++)
				{
					Fluent_Set rs(m_strips_model.num_fluents());
					Fluent_Vec init_fl;
					std::unordered_set<unsigned> counted_actions;
					unsigned app_action_count = 0;
					double agg_log_prob = 0;
					double count = 1;
	
					for (const auto& f : m_strips_model.init())
					{
						rs.set(f);
						init_fl.push_back(f);
					}
	
					app_action_count = num_new_action_applies_to_rs(rs, init_fl, counted_actions);
					agg_log_prob -= std::log(app_action_count);

					std::vector<ActionIndexInfo>& rp = *((*SG)[rpi]);
					for(int i=rp.size()-1; i>=0; i--)
					{
						ActionIndexInfo& aii = rp[i];
						Fluent_Vec adds = get_adds(aii);
						Fluent_Vec new_adds;
						for (const auto& af : adds)
						{
							if (!rs.isset(af))
							{
								new_adds.push_back(af);
								rs.set(af);
							}
						}
						app_action_count += num_new_action_applies_to_rs(rs, new_adds, counted_actions);
						if(app_action_count > 0)
							agg_log_prob -= std::log(app_action_count);

						//for each aii get adds, check if adds not in rs, if not get additional actions to app action count, 
						//& calc new weights, and aggregate (remove 1 for taken action?).
					}
					temp_weights.push_back(agg_log_prob);
					if (total_agg_log_prob == 0)
					{
						total_agg_log_prob = agg_log_prob;
					}
					else
					{;
						total_agg_log_prob += std::log(1 + std::exp(agg_log_prob - total_agg_log_prob));
					}
				}
				float log_total_agg_count = total_agg_log_prob;
				total_weights = 0;
				double max = 0.0000000000000000000000000000000001;
				unsigned max_index = 0;
				for (int i=0; i<SG->size(); i++)
				{
					std::cout << "temp_weights[" << i << "] = " << temp_weights[i] << std::endl;
					if (std::exp(temp_weights[i] - log_total_agg_count)> max)
					{
						max = std::exp(temp_weights[i] - log_total_agg_count);
						max_index = i;
					}
					rp_weights.push_back(std::exp(temp_weights[i] - log_total_agg_count));
					total_weights += rp_weights[i];
				}

				std::cout << "max_index = " << max_index << std::endl;
				std::cout << "max = " << max << std::endl;
				std::cout << "total_weights = " << total_weights << std::endl;
				std::cout << "log_total_agg_count = " << log_total_agg_count << std::endl;
				std::cout << "total_weights = " << total_weights << std::endl;
			}


			/*
			We estimate the fact observation probabilities as described in (Wilken et Al. 2024), estimating the probability
			that each fact is added by an action in sampled supporter sets.
			Here we re-weight samples by UTP weights, by performing a weighted average.
			*/
			std::vector<float>* estimate_fact_obs_probs(std::vector<std::vector<ActionIndexInfo>*>& SG, std::vector<float>& rp_weights, float total_weights)
			{

				//get action probs
				std::unordered_map<ActionIndexInfo, float, AIIHash> action_probs;
				std::unordered_map<ActionIndexInfo, float, AIIHash> action_weights;
				std::vector<std::unordered_set<ActionIndexInfo, AIIHash>> fluent_adds(m_strips_model.num_fluents());

				for (int i = 0; i < SG.size(); i++)
				{
					std::vector<ActionIndexInfo>* set = SG[i];
					float weight = rp_weights[i];
					for (const auto& aii : *set)
					{
						action_weights[aii] += weight;
						for (const auto& pf : get_adds(aii))
							fluent_adds[pf].insert(aii);
					}
				}
				
				for (const auto& [aii, count] : action_weights)
				{
					action_probs[aii] = action_weights[aii] / total_weights;
				}

				//get fluent probs from action probs
				std::vector<float>* fluent_probs = new std::vector<float>(m_strips_model.num_fluents(), 0.0f);
				for (int f=0; f<m_strips_model.num_fluents(); f++)
				{
					float prob_non_occurrence = 1.0f;
					for (const auto& aii : fluent_adds[f])
						prob_non_occurrence = prob_non_occurrence * (1-action_probs[aii]);
					(*fluent_probs)[f] = 1-prob_non_occurrence;
				}

				for (const auto& f : (*m_start_fluents))
					(*fluent_probs)[f] = 1.0f;

				return fluent_probs;			
			}

			std::vector<float>* get_fact_observation_probabilities(const State& s,  int N, unsigned max_sample_time_ms, Fluent_Vec subgoals, bool verbose = false)
			{
				return get_fact_observation_probabilities( s.fluent_vec(),  N, max_sample_time_ms, subgoals, verbose );
			}

			/*
			Fact observation probability extraction process
			This function is called by the planner, after extracting the RPG.
			*/
			std::vector<float>* get_fact_observation_probabilities(const Fluent_Vec& fv,  int N, unsigned max_sample_time_ms, Fluent_Vec subgoals, bool verbose = false)
			{

				std::vector<std::vector<ActionIndexInfo>*>* S = nullptr;
				std::vector<float>* fluent_probs;

				auto t1 = std::chrono::high_resolution_clock::now();

				sample_subgoal_supporter_actions(fv, N, subgoals, max_sample_time_ms, S);
				
				auto t2 = std::chrono::high_resolution_clock::now();
				
				// Underlying std::unordered_set<ActionIndexInfo, AIIHash>*>* objects 
				// ownership is passed from S to SG, and removed from S
				// SG = generate_supporter_sets(N, S);

				//get UTP weights
				std::vector<float> rp_weights;
				float total_weights;
				reweigh_rel_plans(S, rp_weights, total_weights);

				auto t3 = std::chrono::high_resolution_clock::now();

				fluent_probs = estimate_fact_obs_probs(*S, rp_weights, total_weights);

				auto t4 = std::chrono::high_resolution_clock::now();
				if (verbose)
				{
					std::chrono::duration<double, std::milli> duration1 = t2 - t1;
					std::chrono::duration<double, std::milli> duration2 = t3 - t2;
					std::chrono::duration<double, std::milli> duration3 = t4 - t3;
					std::cout << "Sample Subgoal Supporter Actions computation time: " << duration1.count()  << " ms"<<std::endl;
					std::cout << "Generate Supporter Sets computation time: " << duration2.count()  << " ms"<<std::endl;
					std::cout << "Estimate Fact Observation Probabilities computation time: " << duration3.count()  << " ms"<<std::endl;
					std::cout << N << " plans sampled" << std::endl;
				}

				destroy(S);

				return fluent_probs;
			}

			//for previous versions compatibility
			std::vector<float> get_fact_observation_probabilities_by_value(Fluent_Vec& fv, int N, unsigned max_sample_time_ms, Fluent_Vec subgoals, bool verbose = false)
			{
				std::unordered_map<unsigned, std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>*> S;
				std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>* SG;
				std::vector<float>* fluent_probs;

				auto t1 = std::chrono::high_resolution_clock::now();

				sample_subgoal_supporter_actions(fv, N, subgoals, max_sample_time_ms, S);
				
				auto t2 = std::chrono::high_resolution_clock::now();

				// Underlying std::unordered_set<ActionIndexInfo, AIIHash>*>* objects 
				// ownership is passed from S to SG, and removed from S
				SG = generate_supporter_sets(N, S);
				auto t3 = std::chrono::high_resolution_clock::now();

				fluent_probs = estimate_fact_obs_probs(SG);

				auto t4 = std::chrono::high_resolution_clock::now();
				std::chrono::duration<double, std::milli> duration1 = t2 - t1;
				std::chrono::duration<double, std::milli> duration2 = t3 - t2;
				std::chrono::duration<double, std::milli> duration3 = t4 - t3;
				std::cout << "Sample Subgoal Supporter Actions computation time: " << duration1.count()  << " ms"<<std::endl;
				std::cout << "Generate Supporter Sets computation time: " << duration2.count()  << " ms"<<std::endl;
				std::cout << "Estimate Fact Observation Probabilities computation time: " << duration3.count()  << " ms"<<std::endl;

				destroy(S, SG);

				return *fluent_probs;
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
				if (m_fluent_record)
				{
					delete m_fluent_record;
					m_fluent_record = nullptr;
				}
			}

			void destroy(std::vector<std::vector<ActionIndexInfo>*>* S)
			{
				if(S)
				{
					for (const auto& vptr : *S)
					{
						if (vptr)
							delete vptr;
					}
					delete S;
				}
				if (m_fluent_record)
				{
					delete m_fluent_record;
					m_fluent_record = nullptr;
				}
			}

			//assumes S_gi not empty
			//returns set and removes it from S_gi
			std::unordered_set<ActionIndexInfo, AIIHash>* pick_erase_random_set(std::vector<std::unordered_set<ActionIndexInfo, AIIHash>*>* S_gi)
			{
					assert(!S_gi->empty());

					std::uniform_int_distribution<size_t> dist(0, S_gi->size() - 1);
					size_t r_i = dist(m_gen);

					std::unordered_set<ActionIndexInfo, AIIHash>* s = (*S_gi)[r_i];
					S_gi->erase(S_gi->begin() + r_i);
					return s;
			}

			const ActionIndexInfo& min_count(const std::vector<ActionIndexInfo>& psups, std::unordered_map<ActionIndexInfo, unsigned, AIIHash>& count_map)
			{
				//return min if 1, and random among min if num min >1 
				std::vector<unsigned> mincountvec;
				unsigned min = no_such_index;

				for (int i = 0; i < psups.size(); i++)
				{
					const auto& ps = psups[i];

					if (count_map[ps] < min)
					{
						min = count_map[ps];
						mincountvec.clear();
						mincountvec.push_back(i);
					}
					else if (count_map[ps] == min)
						mincountvec.push_back(i);
				}

				if (mincountvec.size() > 1)
				{
					std::uniform_int_distribution<size_t> dist(0, mincountvec.size() - 1);
					return psups[mincountvec[dist(m_gen)]];
				}
				else
					return psups[mincountvec[0]];

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

			void set_memory_limit_mb(unsigned memory_limit_mb)
			{
				m_memory_limit_mb = memory_limit_mb;
			}


			const STRIPS_Problem &m_strips_model;
            std::vector<std::set<unsigned>> m_relevant_actions;

            std::vector<Fluent_Vec*> m_layer_fluents;
			std::vector<bool> m_action_already_achieved;
			std::vector<std::vector<bool>> m_ceff_already_achieved;

			FluentRecord* m_fluent_record;

			bool m_goal_unreachable = false;
			std::mt19937 m_gen;

			unsigned m_memory_limit_mb;

			const Fluent_Vec* m_start_fluents;
        };
    }
}


#endif