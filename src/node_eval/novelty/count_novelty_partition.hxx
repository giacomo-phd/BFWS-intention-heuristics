
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

#ifndef __COUNT_NOVELTY_PARTITION__
#define __COUNT_NOVELTY_PARTITION__

#include <search_prob.hxx>
#include <heuristic.hxx>
#include <ext_math.hxx>
#include <strips_state.hxx>
#include <strips_prob.hxx>
#include <vector>
#include <deque>
#include <algorithm>
#include <limits>

namespace aptk
{

	namespace agnostic
	{

		template <typename Search_Model, typename Search_Node>
		class Count_Novelty_Partition : public Heuristic<State>
		{
		public:
			Count_Novelty_Partition(const Search_Model &prob, unsigned max_arity = 1, const unsigned max_MB = 2048)
					: Heuristic<State>(prob), m_strips_model(prob.task()), m_max_memory_size_MB(max_MB), m_always_full_state(false), 
					m_partition_size(0), m_partition_size_2(0),
					m_verbose(true), m_rp_fl_only(false),
					m_use_threshold(false), m_count_threshold(3)
			{
				set_arity(max_arity, 1);
			}

			virtual ~Count_Novelty_Partition()
			{
			}

			void init()
			{
				// typedef typename std::vector<std::vector<Search_Node *>>::iterator Node_Vec_Ptr_It;
				// typedef typename std::vector<Search_Node *>::iterator Node_Ptr_It;
				typedef typename std::vector<std::vector<int>>::iterator Int_Vec_Ptr_It;
				// typedef typename std::vector<std::unordered_map<int, int>>::iterator Int_Vec_Ptr_It;

				// for (Node_Vec_Ptr_It it_p = m_nodes_tuples_by_partition.begin(); it_p != m_nodes_tuples_by_partition.end(); it_p++)
				// 	for (Node_Ptr_It it = it_p->begin(); it != it_p->end(); it++)
				// 		*it = NULL;

				// for (Int_Vec_Ptr_It it_p = m_tuple_counts_by_partition_1.begin(); it_p != m_tuple_counts_by_partition_1.end(); it_p++)
				// 	std::fill(it_p->begin(), it_p->end(), 0);

				// for (Int_Vec_Ptr_It it_p = m_tuple_counts_by_partition_1.begin(); it_p != m_tuple_counts_by_partition_1.end(); it_p++)
				// 	*it_p = std::unordered_map<int, int>();
				// for (Int_Vec_Ptr_It it_p = m_tuple_counts_by_partition_2.begin(); it_p != m_tuple_counts_by_partition_2.end(); it_p++)
				// 	*it_p = std::unordered_map<int, int>();
			}

			unsigned arity() const { return m_arity; }
			void set_full_state_computation(bool b) { m_always_full_state = b; }

			void set_verbose(bool v) { m_verbose = v; }

			void set_rp_fl_only(bool v) { m_rp_fl_only = v; }

			unsigned &partition_size() { return m_partition_size; }
			// bool is_partition_empty(unsigned partition) { return m_nodes_tuples_by_partition[partition].empty(); }

			// Search_Node *table(unsigned partition, unsigned idx) { return m_nodes_tuples_by_partition[partition][idx]; }

			void set_arity(unsigned max_arity, unsigned partition_size = 0)
			{
				if (max_arity > 2)
				{
					std::cerr << "Maximum novelty table allowed for tuples of size 2!" << std::endl;
					std::exit(9);
				}
				// assert(max_arity == 1);
				m_partition_size_2 = m_partition_size = partition_size;
				m_arity = max_arity;
				m_num_tuples_2 = m_num_fluents = m_strips_model.num_fluents();
			
				float size_novelty = ((float)pow(m_num_fluents, 1) / 1024000.) * (float)partition_size * sizeof(int);
				if (m_arity == 2)
					size_novelty += ((float)pow(m_num_fluents, 2) / 1024000.) * (float)partition_size * sizeof(int);
				// std::cout << "Try allocate size: "<< size_novelty<<" MB"<<std::endl;
				if (size_novelty > m_max_memory_size_MB)
				{
					m_arity = 1;
					size_novelty = ((float)pow(m_num_fluents, m_arity) / 1024000.) * (float)partition_size * sizeof(int);

					std::cout << "EXCEDED, m_arity downgraded to 1 --> size: " << size_novelty << " MB" << std::endl;
				}


				if (m_arity == 2)
				{
					// for (unsigned k = 0; k < m_arity; k++)
						m_num_tuples_2 *= m_num_fluents;
				}
					// // m_nodes_tuples_by_partition.resize(partition_size + 1);
					// m_tuple_counts_by_partition.resize(partition_size + 1);

					// for (unsigned i = 0; i < partition_size + 1; i++)
					// {
					// 	// m_nodes_tuples_by_partition[i].clear();
					// 	m_tuple_counts_by_partition[i].resize(m_num_tuples, 0);
					// }
					

				//int oldSize = m_tuple_counts_by_partition_2.size();

				// Resize the vector
				// m_tuple_counts_by_partition_2.resize(partition_size + 1);

					// // Add unordered maps at the new indices
					// for (int i = oldSize; i < (partition_size + 1); ++i) {
					// 	m_tuple_counts_by_partition_2[i] = std::unordered_map<int, int>();
					// }
				// }

				//int oldSize = m_tuple_counts_by_partition_1.size();
				m_tuple_counts_by_partition_1.resize(partition_size + 1);

				// for (int i = oldSize; i < (partition_size + 1); ++i) {
				// 	m_tuple_counts_by_partition_1[i] = std::unordered_map<int, int>();
				// }


				
			}

			// virtual void eval(Search_Node *n, unsigned &h_val)
			// {
				
			// }

			virtual void eval(Search_Node *n, unsigned &h_val)
			{
				compute(n, h_val);

				// // if (m_rp_fl_only)
				// // 	compute_count_metric_rp_fl_only(n, h_val);
				// // else
				// // 	compute_count_metric(n, h_val);
				
				// // update_counts(n);
				// float temp = 0;
				// std::vector<unsigned> bot3 = cover_compute_tuples_1_op(n, temp);
				// // if (m_arity == 2)
				// 	cover_compute_tuples_2_op(n, bot3, h_val);

				// if (temp < -0.3)
				// 	h_val += temp;

			}

			void eval_rp_fl(Search_Node *n, float &h_val)
			{
				compute_count_metric_rp_fl_only(n, h_val);
				update_counts(n);
			}

			virtual void eval(const State &s, unsigned &h_val)
			{

				assert(true);
			}

			virtual void eval(const State &s, unsigned &h_val, std::vector<Action_Idx> &pref_ops)
			{
				assert(true);
			}

			void update_counts(Search_Node *n) {
                float redundant_variable = 0;
                compute(n, redundant_variable);
            }

		protected:

			void check_table_size_1(Search_Node *n)
			{

				if (m_partition_size < n->partition())
				{
					// m_nodes_tuples_by_partition.resize(n->partition() + 1);
					// m_tuple_counts_by_partition_1.resize(n->partition() + 1);

					m_tuple_counts_by_partition_1.resize(n->partition() + 1);

					m_partition_size = n->partition();

				}

				// if (m_nodes_tuples_by_partition[n->partition()].empty())
				// 	m_nodes_tuples_by_partition[n->partition()].resize(m_num_tuples, NULL);

				if (m_tuple_counts_by_partition_1[n->partition()].empty())
					m_tuple_counts_by_partition_1[n->partition()].resize(m_num_fluents, 0);

				// if (m_tuple_counts_by_partition_1[n->partition()].empty())
				// 	m_tuple_counts_by_partition_1[n->partition()] = std::unordered_map<int, int>();
			}

			// void check_table_size_2(Search_Node *n)
			// {

			// 	if (m_partition_size_2 < n->partition())
			// 	{
			// 		// m_nodes_tuples_by_partition.resize(n->partition() + 1);
			// 		// m_tuple_counts_by_partition.resize(n->partition() + 1);

			// 		m_tuple_counts_by_partition_2.resize(n->partition() + 1);

			// 		m_partition_size_2 = n->partition();

			// 	}

			// 	// if (m_nodes_tuples_by_partition[n->partition()].empty())
			// 	// 	m_nodes_tuples_by_partition[n->partition()].resize(m_num_tuples, NULL);

			// 	if (m_tuple_counts_by_partition_2[n->partition()].empty())
			// 		m_tuple_counts_by_partition_2[n->partition()].resize(m_num_tuples_2, 0);

			// 	// if (m_tuple_counts_by_partition_2[n->partition()].empty())
			// 	// 	m_tuple_counts_by_partition_2[n->partition()] = std::unordered_map<int, int>();
			// }
			

			void compute(Search_Node *n, unsigned &h_val)
			{

				if (n->partition() == std::numeric_limits<unsigned>::max())
					return;

				// if (n->parent() != nullptr && n->partition() == n->parent()->partition())
				// {
				// 	float temp = 0;
				// 	std::vector<unsigned> bot3 = cover_compute_tuples_1_op(n, h_val);
				// 	// if (m_arity == 2)
				// 	//	cover_compute_tuples_2_op(n, bot3, h_val);

				// 	//if (temp < -0.3)
				// 	//h_val += temp;
				// }
				// else
				//{
				// float temp = 0;
				cover_compute_tuples_1(n, h_val);
					// if (m_arity == 2)
					//cover_compute_tuples_2(n, h_val);

					//if (temp < -0.3)
					//	h_val += temp;
				//}
			}

			/**
			 * If parent node is in the same space partition, check only new atoms,
			 * otherwise check all atoms in state
			 */
// 			void compute(Search_Node *n, float &novelty)
// 			{
// 				assert(m_arity == 1);

// 				novelty = (float)m_arity + 1;

// 				if (n->partition() == std::numeric_limits<unsigned>::max())
// 					return;

// 				check_table_size(n);

// 				for (unsigned i = 1; i <= m_arity; i++)
// 				{
// #ifdef DEBUG
// 					if (m_verbose)
// 						std::cout << "search state node: " << &(n) << std::endl;
// #endif

// 					bool new_covers;

// 					// if (n->parent() == nullptr || m_always_full_state)
// 					// 	new_covers = cover_tuples(n, i);
// 					// else
// 					// 	new_covers = (n->partition() == n->parent()->partition()) ? cover_tuples_op(n, i) : cover_tuples(n, i);

// 					new_covers = cover_tuples(n, i);

// #ifdef DEBUG
// 					if (m_verbose && !new_covers)
// 						std::cout << "\t \t PRUNE! search node: " << &(n) << std::endl;
// #endif
// 					if (new_covers)
// 						if (i < novelty)
// 							novelty = i;
// 				}
// 			}

// 			bool n_has_state(Search_Node *n){
// 				return n->state() != NULL;
// 			}
 

//  			std::vector<unsigned> initialValues = {0,0,0};
// 				std::priority_queue<float, std::vector<float>, std::greater<float>> top_3_heap(initialValues.begin(), initialValues.end());
// 					m = (float)1 / (1 + tuple_count);
// 					if ( m > top_3_heap.top()) {
// 						top_3_heap.pop();
// 						top_3_heap.push(m);
// 					}
//                 

// 				while (!top_3_heap.empty())
// 				{
// 					float m = top_3_heap.top();
// 					top_3_heap.pop();
// 					metric_value -= m;
// 				}
//  TODO: MAKE SEPARATE LIST/FUNCTION FOR COVER COMPUTE WIDTH 1 AND 2, 
			std::vector<unsigned> getBottom3Keys(const std::map<unsigned, unsigned>& bottom3) {
				std::vector<unsigned> keys;
				for (const auto& pair : bottom3) {
					keys.push_back(pair.first);
				}
				return keys;
			}

			void keepBottom3(int key, int value, std::map<unsigned, unsigned>& bottom3) {
				bottom3[key] = value;

				if (bottom3.size() > 3) {
					auto maxElem = std::max_element(
						bottom3.begin(), bottom3.end(),
						[](const auto& a, const auto& b) { return a.second < b.second; }
					);

					bottom3.erase(maxElem);
				}
			}

			// std::vector<unsigned> cover_compute_tuples_1_op(Search_Node *n, float &metric_value)
			// {
			// 	metric_value = 0;
			// 	unsigned arity = 1;
			// 	// assert(arity == 1);
				

			// 	if (n->partition() == std::numeric_limits<unsigned>::max())
			// 		return std::vector<unsigned>();

			// 	check_table_size_1(n);

			// 	const bool has_state = n->has_state();
			// 	Fluent_Vec fl;
			// 	if (n->parent() == NULL)
			// 	{
			// 		fl = n->state()->fluent_vec();
			// 	}
			// 	else
			// 	{
			// 		static Fluent_Vec new_atom_vec;
			// 		const Action *a = m_strips_model.actions()[n->action()];
			// 		if (a->has_ceff())
			// 		{
			// 			static Fluent_Set new_atom_set(m_strips_model.num_fluents() + 1);
			// 			new_atom_set.reset();
			// 			new_atom_vec.clear();
			// 			for (Fluent_Vec::const_iterator it = a->add_vec().begin(); it != a->add_vec().end(); it++)
			// 			{
			// 				if (new_atom_set.isset(*it))
			// 					continue;

			// 				new_atom_vec.push_back(*it);
			// 				new_atom_set.set(*it);
			// 			}
			// 			for (unsigned i = 0; i < a->ceff_vec().size(); i++)
			// 			{
			// 				Conditional_Effect *ce = a->ceff_vec()[i];
			// 				if (ce->can_be_applied_on(*(n->parent()->state())))
			// 					for (Fluent_Vec::iterator it = ce->add_vec().begin(); it != ce->add_vec().end(); it++)
			// 					{
			// 						{
			// 							if (new_atom_set.isset(*it))
			// 								continue;

			// 							new_atom_vec.push_back(*it);
			// 							new_atom_set.set(*it);
			// 						}
			// 					}
			// 			}
			// 		}

			// 	const Fluent_Vec &add = a->has_ceff() ? new_atom_vec : a->add_vec();
			// 	fl = add;
			// 	}

				

			// 	bool new_covers = false;

			// 	std::vector<unsigned> tuple(arity);

			// 	unsigned n_combinations = aptk::unrolled_pow(fl.size(), arity);


			// 	float m = 0;

			// 	std::map<unsigned, unsigned> bot3;

			// 	for (unsigned idx = 0; idx < n_combinations; idx++)
			// 	{
			// 		/**
			// 		 * get tuples from indexes
			// 		 */
			// 		idx2tuple(tuple, fl, idx, arity);

			// 		/**
			// 		 * Check if tuple is covered
			// 		 */

			// 		unsigned tuple_idx;
			// 		int tuple_count;

			// 		if (arity == 1)
			// 		{
			// 			tuple_idx = tuple2idx(tuple, arity);
			// 		}
			// 		else if (arity == 2)
			// 		{
			// 			if (tuple[0] == tuple[1])
			// 				continue; // don't check singleton tuples
			// 			tuple_idx = tuple2idx_size2(tuple, arity);
			// 		}
			// 		else
			// 		{

			// 			// If all elements in the tuple are equal, ignore the tuple
			// 			if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x)
			// 											{ return x != tuple[0]; }))
			// 				continue;
			// 			tuple_idx = tuple2idx(tuple, arity);
			// 		}

			// 		/**
			// 		 * new_tuple if
			// 		 * -> none was registered
			// 		 * OR
			// 		 * -> n better than old_n
			// 		 */

			// 		// auto &n_seen = m_nodes_tuples_by_partition[n->partition()][tuple_idx];

			// 		/*increment tuple counts for partition*/
			// 		// if(m_use_threshold) {
			// 		// 	if (m_tuple_counts_by_partition_1[n->partition()].count(tuple_idx) > 0)
			// 		// 	{
			// 		// 		if( !(m_tuple_counts_by_partition_1[n->partition()][tuple_idx] >= m_count_threshold) )
			// 		// 			tuple_count = m_tuple_counts_by_partition_1[n->partition()][tuple_idx]++;
			// 		// 		else
			// 		// 			tuple_count = -1;
			// 		// 	}
			// 		// 	else {
			// 		// 		m_tuple_counts_by_partition_1[n->partition()][tuple_idx] = 1;
			// 		// 		tuple_count = 0;
			// 		// 	}
			// 		// }
			// 		// else{
			// 		// 	if (m_tuple_counts_by_partition_1[n->partition()].count(tuple_idx) > 0)
			// 		// 		tuple_count = m_tuple_counts_by_partition_1[n->partition()][tuple_idx]++;
			// 		// 		// tuple_count = -1;
			// 		// 	else 
			// 		// 	{
			// 		// 		m_tuple_counts_by_partition_1[n->partition()][tuple_idx] = 1;
			// 		// 		tuple_count = 0;
			// 		// 	}
			// 		// }
			// 		if (m_tuple_counts_by_partition_1[n->partition()][tuple_idx] > 0)
			// 			tuple_count = m_tuple_counts_by_partition_1[n->partition()][tuple_idx]++;
			// 			// tuple_count = -1;
			// 		else 
			// 		{
			// 			m_tuple_counts_by_partition_1[n->partition()][tuple_idx] = 1;
			// 			tuple_count = 0;
			// 		}

			// 		// if (tuple_count == -1)
			// 		// 	m = 0;
			// 		// else
			// 		// {
			// 		m = -(float)1 / (1 + tuple_count);
			// 		if (m < metric_value)
			// 			metric_value = m;
			// 		// }

			// 		// keepBottom3(tuple[0], tuple_count, bot3);
			// 	}
			// 	// if (!has_state)
			// 	// {
			// 	// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
			// 	// 	// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
			// 	// }
			// 	// if (!has_state)
			// 	// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

			// 	return fl;
			// }
									// m = -(float)1 / (1 + tuple_count);
						// if (m < metric_value)
						// 	metric_value = m;
							//  			std::vector<unsigned> initialValues = {0,0,0};
						// 				std::priority_queue<float, std::vector<float>, std::greater<float>> top_3_heap(initialValues.begin(), initialValues.end());
						// 					m = (float)1 / (1 + tuple_count);
						// 					if ( m > top_3_heap.top()) {
						// 						top_3_heap.pop();
						// 						top_3_heap.push(m);
						// 					}
						//                 

						// 				while (!top_3_heap.empty())
						// 				{
						// 					float m = top_3_heap.top();
						// 					top_3_heap.pop();
						// 					metric_value -= m;
						// 				}


			// bool cover_compute_tuples_2_op(Search_Node *n, std::vector<unsigned> &bot_3_fl, float &metric_value)
			// {
			// 	float metric_value_2 = 0;
			// 	unsigned arity = m_arity;
			// 	// assert(arity == 1);

			// 	if (n->partition() == std::numeric_limits<unsigned>::max())
			// 		return false;

			// 	check_table_size_2(n);

			// 	const bool has_state = n->has_state();

			// 	static Fluent_Vec added, deleted, temp_fv;
			// 	if (!has_state)
			// 	{
			// 		added.clear();
			// 		deleted.clear();
			// 		// temp_fv.clear();
			// 		// temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());	
			// 		n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
			// 	}
			// 	// if (!has_state)
			// 	// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);

			// 	Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

			// 	bool new_covers = false;

			// 	std::vector<unsigned> tuple(arity);

			// 	unsigned atoms_arity = arity - 1;
			// 	unsigned n_combinations = aptk::unrolled_pow(fl.size(), atoms_arity);
			// 	float m = 0;
			// 	for (Fluent_Vec::const_iterator it_bot_3_fl = bot_3_fl.begin();
			// 			 it_bot_3_fl != bot_3_fl.end(); it_bot_3_fl++)
			// 	{

			// 		for (unsigned idx = 0; idx < n_combinations; idx++)
			// 		{

			// 			tuple[atoms_arity] = *it_bot_3_fl;
			// 			/**
			// 			 * Check if tuple is covered
			// 			 */
			// 			unsigned tuple_idx;
			// 			int tuple_count;

			// 			if (arity == 1)
			// 			{
			// 				tuple_idx = tuple2idx(tuple, arity);
			// 			}
			// 			else if (arity == 2)
			// 			{
			// 				tuple[0] = fl[idx];
			// 				if (tuple[0] == tuple[1])
			// 					continue; // don't check singleton tuples
			// 				tuple_idx = tuple2idx_size2(tuple, arity);
			// 			}
			// 			else
			// 			{
							
			// 				exit(1); //undefined for width > 2

			// 				// // If all elements in the tuple are equal, ignore the tuple
			// 				// if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x)
			// 				// 								{ return x != tuple[0]; }))
			// 				// 	continue;
			// 				// /**
			// 				//  * get tuples from indexes
			// 				//  */
			// 				// idx2tuple(tuple, fl, idx, atoms_arity);

			// 				// tuple_idx = tuple2idx(tuple, arity);
			// 			}

			// 			/**
			// 			 * new_tuple if
			// 			 * -> none was registered
			// 			 * OR
			// 			 * -> n better than old_n
			// 			 */

			// 			// /*increment tuple counts for partition*/
			// 			// if(m_use_threshold) {
			// 			// 	if (m_tuple_counts_by_partition_2[n->partition()].count(tuple_idx) > 0)
			// 			// 	{
			// 			// 		if( !(m_tuple_counts_by_partition_2[n->partition()][tuple_idx] >= m_count_threshold) )
			// 			// 			tuple_count = m_tuple_counts_by_partition_2[n->partition()][tuple_idx]++;
			// 			// 		else
			// 			// 			tuple_count = -1;
			// 			// 	}
			// 			// 	else {
			// 			// 		m_tuple_counts_by_partition_2[n->partition()][tuple_idx] = 1;
			// 			// 		tuple_count = 0;
			// 			// 	}
			// 			// }
			// 			// else{
			// 			// if (m_tuple_counts_by_partition_2[n->partition()].count(tuple_idx) > 0)
			// 			if (m_tuple_counts_by_partition_2[n->partition()][tuple_idx] > 0)
			// 				tuple_count = m_tuple_counts_by_partition_2[n->partition()][tuple_idx]++;
			// 				// tuple_count = -1;
			// 			else 
			// 			{
			// 				m_tuple_counts_by_partition_2[n->partition()][tuple_idx] = 1;
			// 				tuple_count = 0;
			// 			}
			// 			// }

			// 			// if (tuple_count == -1)
			// 			// 	m = 0;
			// 			// else
			// 			// {
			// 			m = -(float)1 / (1 + tuple_count);
			// 			if (m < metric_value_2)
			// 				metric_value_2 = m;
			// 			// }	
			// 		}
			// 	}
			// 	metric_value += metric_value_2;

			// 	if (!has_state)
			// 	{
			// 		n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
			// 		// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
			// 	}
			// 	// if (!has_state)
			// 	// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

			// 	return new_covers;









			// 	// std::vector<unsigned> tuple(arity);


			// 	// unsigned n_combinations = aptk::unrolled_pow(fl.size(), arity);


			// 	// float m = 0;
			// 	// for (unsigned idx = 0; idx < n_combinations; idx++)
			// 	// {
			// 	// 	/**
			// 	// 	 * get tuples from indexes
			// 	// 	 */
			// 	// 	idx2tuple(tuple, fl, idx, arity);

			// 	// 	/**
			// 	// 	 * Check if tuple is covered
			// 	// 	 */

			// 	// 	unsigned tuple_idx;
			// 	// 	int tuple_count;

			// 	// 	if (arity == 1)
			// 	// 	{
			// 	// 		tuple_idx = tuple2idx(tuple, arity);
			// 	// 	}
			// 	// 	else if (arity == 2)
			// 	// 	{
			// 	// 		if (tuple[0] == tuple[1])
			// 	// 			continue; // don't check singleton tuples
			// 	// 		tuple_idx = tuple2idx_size2(tuple, arity);
			// 	// 	}
			// 	// 	else
			// 	// 	{

			// 	// 		// If all elements in the tuple are equal, ignore the tuple
			// 	// 		if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x)
			// 	// 										{ return x != tuple[0]; }))
			// 	// 			continue;
			// 	// 		tuple_idx = tuple2idx(tuple, arity);
			// 	// 	}

			// 	// 	/**
			// 	// 	 * new_tuple if
			// 	// 	 * -> none was registered
			// 	// 	 * OR
			// 	// 	 * -> n better than old_n
			// 	// 	 */

			// 	// 	// auto &n_seen = m_nodes_tuples_by_partition[n->partition()][tuple_idx];

			// 	// 	/*increment tuple counts for partition*/
			// 	// 	if(m_use_threshold) {
			// 	// 		if (m_tuple_counts_by_partition_2[n->partition()].count(tuple_idx) > 0)
			// 	// 		{
			// 	// 			if( !(m_tuple_counts_by_partition_2[n->partition()][tuple_idx] >= m_count_threshold) )
			// 	// 				tuple_count = m_tuple_counts_by_partition_2[n->partition()][tuple_idx]++;
			// 	// 			else
			// 	// 				tuple_count = -1;
			// 	// 		}
			// 	// 		else {
			// 	// 			m_tuple_counts_by_partition_2[n->partition()][tuple_idx] = 1;
			// 	// 			tuple_count = 0;
			// 	// 		}
			// 	// 	}
			// 	// 	else{
			// 	// 		if (m_tuple_counts_by_partition_2[n->partition()].count(tuple_idx) > 0)
			// 	// 			tuple_count = m_tuple_counts_by_partition_2[n->partition()][tuple_idx]++;
			// 	// 			// tuple_count = -1;
			// 	// 		else 
			// 	// 		{
			// 	// 			m_tuple_counts_by_partition_2[n->partition()][tuple_idx] = 1;
			// 	// 			tuple_count = 0;
			// 	// 		}
			// 	// 	}

			// 	// 	if (tuple_count == -1)
			// 	// 		m = 0;
			// 	// 	else
			// 	// 	{
			// 	// 		m = -(float)1 / (1 + tuple_count);
			// 	// 		if (m < metric_value)
			// 	// 			metric_value = m;
			// 	// 	}
					
			// 	// }
			// 	// if (!has_state)
			// 	// {
			// 	// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
			// 	// 	// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
			// 	// }
			// 	// // if (!has_state)
			// 	// // 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

			// 	// return new_covers;
			// }

			// bool cover_compute_tuples_2(Search_Node *n, float &metric_value)
			// {
			// 	float metric_value_2 = 0;
			// 	unsigned arity = m_arity;
			// 	// assert(arity == 1);

			// 	if (n->partition() == std::numeric_limits<unsigned>::max())
			// 		return false;

			// 	check_table_size_2(n);

			// 	const bool has_state = n->has_state();

			// 	static Fluent_Vec added, deleted, temp_fv;
			// 	if (!has_state)
			// 	{
			// 		added.clear();
			// 		deleted.clear();
			// 		// temp_fv.clear();
			// 		// temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());	
			// 		n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
			// 	}
			// 	// if (!has_state)
			// 	// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);

			// 	Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

			// 	bool new_covers = false;

			// 	std::vector<unsigned> tuple(arity);

			// 	unsigned n_combinations = aptk::unrolled_pow(fl.size(), arity); 

			// 	float m = 0;

			// 	for (unsigned idx = 0; idx < n_combinations; idx++)
			// 	{


			// 		/**
			// 		 * get tuples from indexes
			// 		 */
			// 		idx2tuple(tuple, fl, idx, arity);

			// 		/**
			// 		 * Check if tuple is covered
			// 		 */
			// 		unsigned tuple_idx;
			// 		int tuple_count;

			// 		if (arity == 1)
			// 		{
			// 			tuple_idx = tuple2idx(tuple, arity);
			// 		}
			// 		else if (arity == 2)
			// 		{
			// 			if (tuple[0] == tuple[1])
			// 				continue; // don't check singleton tuples
			// 			tuple_idx = tuple2idx_size2(tuple, arity);
			// 		}
			// 		else
			// 		{

			// 			// If all elements in the tuple are equal, ignore the tuple
			// 			if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x)
			// 											{ return x != tuple[0]; }))
			// 				continue;
			// 		}

			// 		if (m_tuple_counts_by_partition_2[n->partition()][tuple_idx] > 0)
			// 			tuple_count = m_tuple_counts_by_partition_2[n->partition()][tuple_idx]++;
			// 			// tuple_count = -1;
			// 		else 
			// 		{
			// 			m_tuple_counts_by_partition_2[n->partition()][tuple_idx] = 1;
			// 			tuple_count = 0;
			// 		}

			// 		m = -(float)1 / (1 + tuple_count);
			// 		if (m < metric_value_2)
			// 			metric_value_2 = m;
			// 	}

			// 	metric_value += metric_value_2;

			// 	if (!has_state)
			// 	{
			// 		n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
			// 		// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
			// 	}
			// 	// if (!has_state)
			// 	// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

			// 	return new_covers;
			// }

			bool cover_compute_tuples_1(Search_Node *n, unsigned &metric_value)
			{
				// float metric_value_2 = 0;
				unsigned arity = 1;
				// assert(arity == 1);

				if (n->partition() == std::numeric_limits<unsigned>::max())
					return false;

				check_table_size_1(n);

				const bool has_state = n->has_state();

				static Fluent_Vec added, deleted, temp_fv;
				if (!has_state)
				{
					added.clear();
					deleted.clear();
					// temp_fv.clear();
					// temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());	
					n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
				}
				// if (!has_state)
				// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);

				Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

				bool new_covers = false;

				std::vector<unsigned> tuple(arity);

				unsigned n_combinations = aptk::unrolled_pow(fl.size(), arity); 

				// unsigned m = 0;

				std::vector<unsigned>& tuple_counts_partition = m_tuple_counts_by_partition_1[n->partition()];

				unsigned min_count = std::numeric_limits<unsigned>::max();

				for (unsigned idx = 0; idx < n_combinations; idx++)
				{


					/**
					 * get tuples from indexes
					 */
					idx2tuple(tuple, fl, idx, arity);

					/**
					 * Check if tuple is covered
					 */
					unsigned tuple_idx;
					unsigned tuple_count;

					tuple_idx = tuple2idx(tuple, arity);

					// if (tuple_counts_partition[tuple_idx] > 0)
					tuple_count = tuple_counts_partition[tuple_idx]++; //because set as 0 by default
						// tuple_count = -1;
					// else
					// {
					// 	tuple_counts_partition[tuple_idx] = 1;
					// 	tuple_count = 0;
					// }

					// m = -(float)1 / (1 + tuple_count);
					// if (m < metric_value_2)
					// 	metric_value_2 = m;

					//compare counts to avoid redundant division calculation
					if (tuple_count < min_count)
						min_count = tuple_count;
					// {
					// 	min_count = tuple_count;
					// 	m = -(float)1 / (1 + tuple_count);
					// 	metric_value_2 = m;
					// }
				}

				// metric_value += metric_value_2;
				metric_value = min_count;

				if (!has_state)
				{
					n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
					// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
				}
				// if (!has_state)
				// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

				return new_covers;
			}



// 			bool cover_tuples(Search_Node *n, unsigned arity)
// 			{
// 				assert(arity == 1);


// 				const bool has_state = n->has_state();

// 				static Fluent_Vec added, deleted, temp_fv;
// 				if (!has_state)
// 				{
					
// 					added.clear();
// 					deleted.clear();
// 					// temp_fv.clear();
// 					// temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());
// 					n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
// 				}
// 				// if (!has_state)
// 				// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);

// 				Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

// 				bool new_covers = false;

// 				std::vector<unsigned> tuple(arity);

// 				unsigned n_combinations = aptk::unrolled_pow(fl.size(), arity);

// 				for (unsigned idx = 0; idx < n_combinations; idx++)
// 				{
// 					/**
// 					 * get tuples from indexes
// 					 */
// 					idx2tuple(tuple, fl, idx, arity);

// 					/**
// 					 * Check if tuple is covered
// 					 */

// 					unsigned tuple_idx;

// 					if (arity == 1)
// 					{
// 						tuple_idx = tuple2idx(tuple, arity);
// 					}
// 					else if (arity == 2)
// 					{
// 						if (tuple[0] == tuple[1])
// 							continue; // don't check singleton tuples
// 						tuple_idx = tuple2idx_size2(tuple, arity);
// 					}
// 					else
// 					{

// 						// If all elements in the tuple are equal, ignore the tuple
// 						if (std::any_of(tuple.cbegin(), tuple.cend(), [&tuple](unsigned x)
// 														{ return x != tuple[0]; }))
// 							continue;
// 						tuple_idx = tuple2idx(tuple, arity);
// 					}

// 					/**
// 					 * new_tuple if
// 					 * -> none was registered
// 					 * OR
// 					 * -> n better than old_n
// 					 */

// 					// auto &n_seen = m_nodes_tuples_by_partition[n->partition()][tuple_idx];

// 					/*increment tuple counts for partition*/
// 					if(m_use_threshold) {
// 						if (m_tuple_counts_by_partition[n->partition()].count(tuple_idx) > 0)
// 						{
// 							if( !(m_tuple_counts_by_partition[n->partition()][tuple_idx] > m_count_threshold) )
// 								m_tuple_counts_by_partition[n->partition()][tuple_idx]++;
// 						}
// 						else 
// 							m_tuple_counts_by_partition[n->partition()][tuple_idx] = 1;
// 					}
// 					else{
// 						if (m_tuple_counts_by_partition[n->partition()].count(tuple_idx) > 0)
// 							m_tuple_counts_by_partition[n->partition()][tuple_idx]++;
// 						else 
// 							m_tuple_counts_by_partition[n->partition()][tuple_idx] = 1;
// 					}
// // 					if (!n_seen || is_better(n_seen, n))
// // 					{

// // 						n_seen = (Search_Node *)n;
// // 						new_covers = true;

// // #ifdef DEBUG
// // 						if (m_verbose)
// // 						{
// // 							std::cout << "\t NEW!! : ";
// // 							for (unsigned i = 0; i < arity; i++)
// // 							{
// // 								std::cout << m_strips_model.fluents()[tuple[i]]->signature() << "  ";
// // 							}
// // 							std::cout << std::endl;
// // 						}
// // #endif
// // 					}
// 				}
// 				if (!has_state)
// 				{
// 					n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
// 					// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
// 				}
// 				// if (!has_state)
// 				// 	n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

// 				return new_covers;
// 			}

// 			 /* currently designed for width=1, behavior for w>1 undefined*/
//             void compute_count_metric(Search_Node *n, float &metric_value) {

// 				if (n->partition() == std::numeric_limits<unsigned>::max())
// 					return;

// 				check_table_size(n);
                
//                 /*HARD CODED TO 1 FOR THIS METHOD*/
//                 unsigned arity = 1;
		
//                 metric_value = 0;

//                 // const bool has_state = n->has_state();
// 				const bool has_state = n_has_state(n);

// 				static Fluent_Vec added, deleted, temp_fv;
// 				if (!has_state)
// 				{
					
// 					added.clear();
// 					deleted.clear();
// 					// temp_fv.clear();
// 					// temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());
// 					n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
// 				}
//                 // if (!has_state)
// 				// 	n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()]);

//                 Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

//                 std::vector<unsigned> tuple(m_arity);

//                 unsigned n_combinations = aptk::unrolled_pow(fl.size(), m_arity);

// 				float m = 0;

// 				// //make initial min heap have size 3, with starting values +inf
// 				// std::vector<unsigned> initialValues = {0,0,0};
// 				// std::priority_queue<float, std::vector<float>, std::greater<float>> top_3_heap(initialValues.begin(), initialValues.end());

//                 for (unsigned idx = 0; idx < n_combinations; idx++)
//                 {
//                     /**
// 					 * get tuples from indexes
// 					 */
// 					idx2tuple(tuple, fl, idx, m_arity); /*gets a tuple for checking novelty, using idx to determine the respective fluents in fl to create the tuple, & arity for tuple size*/

// 					/**
// 					 * Check if tuple is covered
// 					 */
// 					unsigned tuple_idx;
// 					unsigned tuple_count;

//                     /*if arity = 1*/
//                     tuple_idx = tuple2idx(tuple, m_arity);

// 					// tuple_count = m_tuple_counts_by_partition[n->partition()][tuple_idx];

// 					if (m_tuple_counts_by_partition[n->partition()].count(tuple_idx) > 0)
// 						tuple_count = m_tuple_counts_by_partition[n->partition()][tuple_idx];
// 					else 
// 						tuple_count = 0;

// 					// float debug_val = (float)1 / (1 + tuple_count); //DEBUG
//                     /*subtract to get negative of novelty metric, such that lower value means greater surprise*/
//                     // metric_value -= (float)1 / (1 + tuple_count);
// 					// m = -(float)1 / (1 + tuple_count);
// 					// if (m < metric_value)
// 					// 	metric_value = m;

// 					m = -(float)1 / (1 + tuple_count);
// 					if (m < metric_value)
// 						metric_value = m;

// 					// m = (float)1 / (1 + tuple_count);
// 					// if ( m > top_3_heap.top()) {
// 					// 	top_3_heap.pop();
// 					// 	top_3_heap.push(m);
// 					// }
//                 }

// 				// while (!top_3_heap.empty())
// 				// {
// 				// 	float m = top_3_heap.top();
// 				// 	top_3_heap.pop();
// 				// 	metric_value -= m;
// 				// }

// 				if (!has_state)
// 				{
// 					n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
// 					// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
// 				}
//                 // if (!has_state)
// 				//     n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()]);

//             }

// 			void compute_count_metric_rp_fl_only(Search_Node *n, float &metric_value) {

// 				if (n->partition() == std::numeric_limits<unsigned>::max())
// 					return;
				
// 				check_table_size(n);

// 				/*HARD CODED TO 1 FOR THIS METHOD*/
//                 unsigned arity = 1;
		
//                 metric_value = 0;


// 				Fluent_Set* rp_f_set = get_rp_set(n);
// 				static Fluent_Set counted(m_num_fluents);

// 				const bool has_state = n_has_state(n);
				
// 				static Fluent_Vec added, deleted, temp_fv;
// 				if (!has_state)
// 				{
// 					added.clear();
// 					deleted.clear();
// 					// temp_fv.clear();
// 					// temp_fv.assign(n->parent()->state()->fluent_vec().begin(), n->parent()->state()->fluent_vec().end());
// 					n->parent()->state()->progress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
// 				}
// 				Fluent_Vec &fl = has_state ? n->state()->fluent_vec() : n->parent()->state()->fluent_vec();

// 				/*
// 				 * Creating new Fluent_Vec with only fluents in rp_f_set, & then using that for getting metric value
// 				 * Possibly more inefficient (?) but easier implementation for testing
// 				*/
// 				Fluent_Vec rp_fl;
// 				for (unsigned f : fl) {
// 					if (rp_f_set->isset(f) && !counted.isset(f))
// 					{
// 						rp_fl.push_back(f);
// 						counted.set(f);
// 					}
// 				}
			
//                 std::vector<unsigned> tuple(m_arity);

//                 unsigned n_combinations = aptk::unrolled_pow(rp_fl.size(), m_arity); 

// 				float m = 0;


// 				for (unsigned idx = 0; idx < n_combinations; idx++)
//                 {
//                     /**
// 					 * get tuples from indexes
// 					 */
// 					idx2tuple(tuple, rp_fl, idx, m_arity); /*gets a tuple for checking novelty, using idx to determine the respective fluents in fl to create the tuple, & arity for tuple size*/

// 					/**
// 					 * Check if tuple is covered
// 					 */
// 					unsigned tuple_idx;
// 					unsigned tuple_count;

//                     /*if arity = 1*/
//                     tuple_idx = tuple2idx(tuple, m_arity);

//                     tuple_count = m_tuple_counts_by_partition[n->partition()][tuple_idx];

// 					// float debug_val = (float)1 / (1 + tuple_count); //DEBUG
//                     /*subtract to get negative of novelty metric, such that lower value means greater surprise*/
//                     // metric_value -= (float)1 / (1 + tuple_count);
// 					m = -(float)1 / (1 + tuple_count);
// 					if (m < metric_value)
// 						metric_value = m;
//                 }

// 				// //TEST: compute for all, and add with 0.1 mulyiplier
// 				// n_combinations = aptk::unrolled_pow(fl.size(), m_arity); 

// 				// for (unsigned idx = 0; idx < n_combinations; idx++)
//                 // {
//                 //     /**
// 				// 	 * get tuples from indexes
// 				// 	 */
// 				// 	idx2tuple(tuple, fl, idx, m_arity); /*gets a tuple for checking novelty, using idx to determine the respective fluents in fl to create the tuple, & arity for tuple size*/

// 				// 	/**
// 				// 	 * Check if tuple is covered
// 				// 	 */
// 				// 	unsigned tuple_idx;
// 				// 	unsigned tuple_count;

//                 //     /*if arity = 1*/
//                 //     tuple_idx = tuple2idx(tuple, m_arity);

//                 //     tuple_count = m_tuple_counts_by_partition[n->partition()][tuple_idx];

// 				// 	// float debug_val = (float)1 / (1 + tuple_count); //DEBUG
//                 //     /*subtract to get negative of novelty metric, such that lower value means greater surprise*/
//                 //     metric_value -= 0.1 * ( (float)1 / (1 + tuple_count) );
//                 // }
// 				if (!has_state)
// 				{
// 					n->parent()->state()->regress_lazy_state(m_strips_model.actions()[n->action()], &added, &deleted);
// 					// n->parent()->state()->fluent_vec().assign(temp_fv.begin(), temp_fv.end());
// 				}
// 				counted.reset();
// 			}

			Fluent_Set* get_rp_set(Search_Node *n) 
			{
				Search_Node* n_start = n;
				while (!n_start->rp_vec())
				{
					n_start = n_start->parent();
				}	
				return n_start->rp_set();
			}


			inline unsigned tuple2idx_size2(std::vector<unsigned> &indexes, unsigned arity) const
			{
				unsigned min = indexes[0] <= indexes[1] ? indexes[0] : indexes[1];
				unsigned max = indexes[0] <= indexes[1] ? indexes[1] : indexes[0];
				return min + max * m_num_fluents;
			}

			inline unsigned tuple2idx(std::vector<unsigned> &indexes, unsigned arity) const
			{
				unsigned idx = 0;
				unsigned dimension = 1;

				std::sort(indexes.begin(), indexes.end());
				for (int i = arity - 1; i >= 0; i--)
				{
					idx += indexes[i] * dimension;
					dimension *= m_num_fluents;
				}

				return idx;
			}

			inline void idx2tuple(std::vector<unsigned> &tuple, Fluent_Vec &fl, unsigned idx, unsigned arity) const
			{
				unsigned next_idx, div;
				unsigned current_idx = idx;
				int n_atoms = fl.size();

				for (int i = arity - 1; i >= 0; i--)
				{
					// MRJ: Let's use the fast version
					// n_atoms = 10
					// i = 3 (w=4) -> div = 1000
					// i = 2 (w=3) -> div = 100
					// i = 1 (w=2) -> div = 10
					// i = 0 (w=1) -> div = 1
					div = aptk::unrolled_pow(n_atoms, i);

					if (current_idx < div)
					{
						next_idx = current_idx;
						current_idx = 0;
					}
					else
					{
						// current index : 32
						// i = 3 -> next_idx = 32 % 1000 = 32, 32 / 1000 = 0
						// i = 2 -> next_idx = 32 % 100 = 32, 32 / 100 = 0
						// i = 1 -> next_idx = 32 % 10 = 2, 32 / 10 = 3
						// i = 0 -> next_idx = 32 % 1 = 0, 32 / 1 = 32
						next_idx = current_idx % div;
						const int div_res = current_idx / div;
						// if current_idx is zero and is the last index, then take next_idx
						// i = 3, current_idx = ( 32 / 1000 != 0 || i != 0 ) ? 32 / 1000 : 32 = ( F || T ) ? 0 : 32 = 0
						current_idx = (div_res != 0 || i != 0) ? div_res : next_idx;
					}
					tuple[i] = fl[current_idx];

					current_idx = next_idx;
				}
			}

			inline bool is_better(Search_Node *n, const Search_Node *new_n) const
			{
				// return false;
				return new_n->is_better(n);
			}

			const STRIPS_Problem &m_strips_model;
			// std::vector<std::vector<Search_Node *>> m_nodes_tuples_by_partition;
			// std::vector<std::vector<int>> m_tuple_counts_by_partition;
			// std::vector<std::unordered_map<int, int>> m_tuple_counts_by_partition_2;
			// std::vector<std::unordered_map<int, int>> m_tuple_counts_by_partition_1;
			// std::vector<std::vector<int>> m_tuple_counts_by_partition_2;
			std::vector<std::vector<unsigned>> m_tuple_counts_by_partition_1;
			unsigned m_arity;
			unsigned long m_num_tuples_2;
			unsigned m_num_fluents;
			unsigned m_max_memory_size_MB;
			bool m_always_full_state;
			unsigned m_partition_size;
			unsigned m_partition_size_2;
			bool m_verbose;
			bool m_rp_fl_only;

			bool m_use_threshold;
			unsigned m_count_threshold;
		};

	}

}

#endif // count_novelty_partition.hxx
