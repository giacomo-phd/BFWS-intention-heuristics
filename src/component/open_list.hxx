
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

#ifndef __OPEN_LIST__
#define __OPEN_LIST__

#include <vector>
#include <queue>
#include <boost/heap/fibonacci_heap.hpp>
#include <ext_math.hxx>
#include <random>

namespace aptk
{

	namespace search
	{

		template <typename Node>
		class Node_Comparer
		{
		public:
			bool operator()(Node *a, Node *b) const
			{
				if (dless(b->fn(), a->fn()))
					return true;
				if (dequal(b->fn(), a->fn()))
				{
					if (dequal(b->hn(), a->hn()))
					{
						if (dless(b->gn(), a->gn()))
							return true;
						return false;
					}
				}
				return false;

				// return (dless(b->fn(), a->fn()) || (dequal(a->fn(), b->fn()) && dless(b->hn(), a->hn())));
			}
		};

		template <typename Node>
		class Node_Comparer_DH
		{
		public:
			bool operator()(Node *a, Node *b) const
			{
				if (dless(b->fn(), a->fn()))
					return true;
				if (dequal(b->fn(), a->fn()))
				{
					if (dless(b->h1n(), a->h1n()))
						return true;
					if (dequal(b->h1n(), a->h1n()))
					{

						if (dless(b->h2n(), a->h2n()))
							return true;
					}
				}
				return false;

				// return (dless(b->fn(), a->fn()) || (dequal(a->fn(), b->fn()) && dless(b->hn(), a->hn())));
			}
		};

		template <typename Node>
		class Node_Comparer_3H
		{
		public:
			bool operator()(Node *a, Node *b) const
			{
				if (dless(b->h1n(), a->h1n()))
					return true;
				if (dequal(b->h1n(), a->h1n()))
				{
					if (dless(b->h2n(), a->h2n()))
						return true;
					if (dequal(b->h2n(), a->h2n()))
					{
						if (dless(b->h3n(), a->h3n()))
							return true;
						if (dequal(b->h3n(), a->h3n()))
						{
							if (dless(b->gn(), a->gn()))
								return true;
						}
					}
				}
				// if ( dless( b->gn(), a->gn() ) )  return true;
				return false;

				// return (dless(b->fn(), a->fn()) || (dequal(a->fn(), b->fn()) && dless(b->hn(), a->hn())));
			}
		};

		// template <typename Node>
		// class Alt_Node_Comparer_3H
		// {
		// public:
		// 	bool operator()(Node *a, Node *b) const
		// 	{
		// 		if (dless(b->alt_h1n(), a->alt_h1n()))
		// 			return true;
		// 		if (dequal(b->alt_h1n(), a->alt_h1n()))
		// 		{
		// 			if (dless(b->h2n(), a->h2n()))
		// 				return true;
		// 			if (dequal(b->h2n(), a->h2n()))
		// 			{
		// 				if (dless(b->h3n(), a->h3n()))
		// 					return true;
		// 			}
		// 		}
		// 		// if ( dless( b->gn(), a->gn() ) )  return true;
		// 		return false;

		// 		// return (dless(b->fn(), a->fn()) || (dequal(a->fn(), b->fn()) && dless(b->hn(), a->hn())));
		// 	}
		// };


template <class Node_Comp, class Alt_Node_Comp, class Node>
		class Double_Custom_Priority_Queue
		{
			public:
				typedef Node Node_Type;
			protected:
				std::vector<Node*> m_heap_1;
				std::vector<Node*> m_heap_2;
				int m_size_limit_1;
				int m_size_limit_2;
				int m_last_layer_first_element_1;
				int m_last_layer_first_element_2;
				int m_next_1;
				int m_next_2;
				Node_Comp m_node_comp;
				Alt_Node_Comp m_alt_node_comp;
				// std::random_device m_rd;
				std::mt19937::result_type seed = 42;
    			std::mt19937 m_gen;
				bool m_pop_alt;
				int m_alt_counter;
				int m_alt_interval;
				// float m_th_value;

			public:

				Double_Custom_Priority_Queue() : m_next_1(0), m_next_2(0), m_size_limit_1(0), m_gen(seed), m_pop_alt(false),
				m_alt_counter(0), m_alt_interval(2)
				{
					int default_max_depth = 18;
					init(default_max_depth);
				}
				~Double_Custom_Priority_Queue() {}

				void init(int max_depth)
				{
					m_size_limit_1 = pow(2, max_depth+1) - 1; //for index subtract 1
					m_last_layer_first_element_1 = (m_size_limit_1 / 2) + 1; //for index subtract 1			

					m_size_limit_2 = pow(2, max_depth-1) - 1;
					m_last_layer_first_element_2 = (m_size_limit_2 / 2) + 1;				
				}

				void set_seed(std::mt19937::result_type new_seed) {
					seed = new_seed;
					m_gen.seed(seed);
				}

				bool empty() const { return empty_1() && empty_2(); }
				bool empty_1() const { return m_heap_1.empty(); }
				bool empty_2() const { return m_heap_2.empty(); }
				std::size_t size_1() const { return m_heap_1.size(); }
				std::size_t size_2() const { return m_heap_2.size(); }

				void insert(Node *n)
				{
					Node* d_1 = NULL;
					Node* d_2 = NULL;



					if (size_1() < m_size_limit_1)
					{
						m_heap_1.push_back(n);
						std::push_heap(m_heap_1.begin(), m_heap_1.end(), Node_Comp());
					}
					else
					{
						static std::uniform_int_distribution<> distrib(m_last_layer_first_element_1, m_size_limit_1);
						int r_i = distrib(m_gen)-1;
						if (m_node_comp(m_heap_1[r_i], n))
						{
							d_1 = m_heap_1[r_i];
							m_heap_1[r_i] = n;
							std::push_heap(m_heap_1.begin(), m_heap_1.begin()+r_i+1, Node_Comp());
						}
						else
							d_1 = n;
						d_1->m_open_delete++;
					}

					if (size_2() < m_size_limit_2)
					{
						m_heap_2.push_back(n);
						std::push_heap(m_heap_2.begin(), m_heap_2.end(), Alt_Node_Comp());
					}
					else
					{
						static std::uniform_int_distribution<> distrib(m_last_layer_first_element_2, m_size_limit_2);
						int r_i = distrib(m_gen)-1;
						if (m_alt_node_comp(m_heap_2[r_i], n))
						{
							d_2 = m_heap_2[r_i];
							m_heap_2[r_i] = n;
							std::push_heap(m_heap_2.begin(), m_heap_2.begin()+r_i+1, Alt_Node_Comp());
						}
						else
							d_2 = n;
						d_2->m_open_delete++;
					}

					if (d_1 != nullptr && d_1 == d_2)
					{
						delete d_1;
						d_1 = d_2 = nullptr;
					}
					
					if (d_1 != nullptr && d_1->m_open_delete == 2)
						delete d_1;
					if (d_2 != nullptr && d_2->m_open_delete == 2)
						delete d_2;

				}


				Node* pop()
				{
					bool e1 = empty_1();
					bool e2 = empty_2();
					if (e1 && e2)
						return NULL;
					if (e1)
						return pop_2();
					else if (e2)
						return pop_1();
					else
					{
						if (m_alt_counter == 0)
						{
							m_alt_counter = ++m_alt_counter % m_alt_interval;
							return pop_2();
						}
						else
						{
							m_alt_counter = ++m_alt_counter % m_alt_interval;
							return pop_1();
						}
					}
				}

				Node* pop_1()
				{
					Node* r = m_heap_1.front();
					std::pop_heap(m_heap_1.begin(), m_heap_1.end(), Node_Comp());
					m_heap_1.pop_back();
					r->m_pop_count++;
					r->set_olp_cc(1);
					return r;
				}

				Node* pop_2()
				{
					Node* r = m_heap_2.front();
					std::pop_heap(m_heap_2.begin(), m_heap_2.end(), Alt_Node_Comp());
					m_heap_2.pop_back();
					r->m_pop_count++;
					r->set_olp_cn(1);
					return r;
				}

				Node* top_heap_1()
				{
					return m_heap_1.front();
				}

				Node* top_heap_2()
				{
					return m_heap_2.front();
				}


		};

template <class Node_Comp, class Alt_Node_Comp, class Third_Node_Comp, class Node>
class Triple_Custom_Priority_Queue
{
public:
    typedef Node Node_Type;

protected:
    // Three heaps (each stored as a vector of pointers)
    std::vector<Node*> m_heap_1;
    std::vector<Node*> m_heap_2;
    std::vector<Node*> m_heap_3;

    // Heap 1 size and “last layer” values
    int m_size_limit_1;
    int m_last_layer_first_element_1;
    // Heap 2 size and “last layer” values
    int m_size_limit_2;
    int m_last_layer_first_element_2;
    // Heap 3 size and “last layer” values
    int m_size_limit_3;
    int m_last_layer_first_element_3;

    // The three comparator functors.
    Node_Comp     m_node_comp;
    Alt_Node_Comp m_alt_node_comp;
    Third_Node_Comp m_third_node_comp;

    // A random generator for selecting a random index in the last layer.
    std::mt19937::result_type seed = 42;
    std::mt19937 m_gen;

    // For round-robin pops among the three lists.
    int m_pop_counter;

public:
    Triple_Custom_Priority_Queue() 
        : m_gen(seed), m_pop_counter(0)
    {
        // Use some default max depth.
        int default_max_depth = 18;
        init(default_max_depth);
    }

    ~Triple_Custom_Priority_Queue() {}

    // Initialize the three open lists.
    // (Here we choose different “max depths” for the three heaps.
    //  For example, heap1 uses max_depth+1, heap2 uses max_depth, and heap3 uses max_depth-1.)
    void init(int max_depth)
    {
        m_size_limit_1 = static_cast<int>(std::pow(2, max_depth + 1)) - 1;
        m_last_layer_first_element_1 = (m_size_limit_1 / 2) + 1;

        m_size_limit_2 = static_cast<int>(std::pow(2, max_depth)) - 1;
        m_last_layer_first_element_2 = (m_size_limit_2 / 2) + 1;

        m_size_limit_3 = static_cast<int>(std::pow(2, max_depth - 1)) - 1;
        m_last_layer_first_element_3 = (m_size_limit_3 / 2) + 1;
    }

    void set_seed(std::mt19937::result_type new_seed) {
        seed = new_seed;
        m_gen.seed(seed);
    }

    // Check emptiness.
    bool empty() const { return empty_1() && empty_2() && empty_3(); }
    bool empty_1() const { return m_heap_1.empty(); }
    bool empty_2() const { return m_heap_2.empty(); }
    bool empty_3() const { return m_heap_3.empty(); }

    std::size_t size_1() const { return m_heap_1.size(); }
    std::size_t size_2() const { return m_heap_2.size(); }
    std::size_t size_3() const { return m_heap_3.size(); }

    // Insert a node into all three open lists.
    // (If an open list is “full” then a node in that list is replaced at random 
    //  in the last layer. In that case the replaced node’s deletion counter is incremented.
    //  Once a node’s counter equals the number of open lists (3 in this case), it is deleted.)
    void insert(Node *n)
    {
        Node* d_1 = nullptr;
        Node* d_2 = nullptr;
        Node* d_3 = nullptr;

        // --- Heap 1 insertion ---
        if (size_1() < m_size_limit_1)
        {
            m_heap_1.push_back(n);
            std::push_heap(m_heap_1.begin(), m_heap_1.end(), m_node_comp);
        }
        else
        {
            std::uniform_int_distribution<> distrib1(m_last_layer_first_element_1, m_size_limit_1);
            int r_i = distrib1(m_gen) - 1;
            if (m_node_comp(m_heap_1[r_i], n))
            {
                d_1 = m_heap_1[r_i];
                m_heap_1[r_i] = n;
                std::push_heap(m_heap_1.begin(), m_heap_1.begin() + r_i + 1, m_node_comp);
            }
            else
                d_1 = n;
            d_1->m_open_delete++;
        }

        // --- Heap 2 insertion ---
        if (size_2() < m_size_limit_2)
        {
            m_heap_2.push_back(n);
            std::push_heap(m_heap_2.begin(), m_heap_2.end(), m_alt_node_comp);
        }
        else
        {
            std::uniform_int_distribution<> distrib2(m_last_layer_first_element_2, m_size_limit_2);
            int r_i = distrib2(m_gen) - 1;
            if (m_alt_node_comp(m_heap_2[r_i], n))
            {
                d_2 = m_heap_2[r_i];
                m_heap_2[r_i] = n;
                std::push_heap(m_heap_2.begin(), m_heap_2.begin() + r_i + 1, m_alt_node_comp);
            }
            else
                d_2 = n;
            d_2->m_open_delete++;
        }

        // --- Heap 3 insertion ---
        if (size_3() < m_size_limit_3)
        {
            m_heap_3.push_back(n);
            std::push_heap(m_heap_3.begin(), m_heap_3.end(), m_third_node_comp);
        }
        else
        {
            std::uniform_int_distribution<> distrib3(m_last_layer_first_element_3, m_size_limit_3);
            int r_i = distrib3(m_gen) - 1;
            if (m_third_node_comp(m_heap_3[r_i], n))
            {
                d_3 = m_heap_3[r_i];
                m_heap_3[r_i] = n;
                std::push_heap(m_heap_3.begin(), m_heap_3.begin() + r_i + 1, m_third_node_comp);
            }
            else
                d_3 = n;
            d_3->m_open_delete++;
        }

		if (d_1 && d_1->m_open_delete == 3)
			delete d_1;
		if (d_2 && d_2->m_open_delete == 3 && d_2 != d_1)
			delete d_2;
		if (d_3 && d_3->m_open_delete == 3 && d_3 != d_1 && d_3 != d_2)
			delete d_3;

    }

    // The pop() function cycles among the three heaps (if possible).
    // (If one of the heaps is empty, it is skipped.)
	Node* pop()
	{
		// Try up to three times (once for each open list)
		int tries = 0;
		while (tries < 3) {
			// Use the current pop counter as the index
			int idx = m_pop_counter;
			if (idx == 0) {
				if (!empty_1()) {
					// Advance counter for the next pop and return the node.
					m_pop_counter = (m_pop_counter + 1) % 3;
					return pop_1();
				}
			}
			else if (idx == 1) {
				if (!empty_2()) {
					m_pop_counter = (m_pop_counter + 1) % 3;
					return pop_2();
				}
			}
			else if (idx == 2) {
				if (!empty_3()) {
					m_pop_counter = (m_pop_counter + 1) % 3;
					return pop_3();
				}
			}
			// If the current list is empty, move to the next list.
			m_pop_counter = (m_pop_counter + 1) % 3;
			++tries;
		}
		// If all lists are empty, return nullptr.
		return nullptr;
	}

    // Pop from heap 1.
    Node* pop_1()
    {
        Node* r = m_heap_1.front();
        std::pop_heap(m_heap_1.begin(), m_heap_1.end(), m_node_comp);
        m_heap_1.pop_back();
        r->m_pop_count++;
        r->set_olp_cc(1); // Mark that the node was popped from open list 1.
        return r;
    }

    // Pop from heap 2.
    Node* pop_2()
    {
        Node* r = m_heap_2.front();
        std::pop_heap(m_heap_2.begin(), m_heap_2.end(), m_alt_node_comp);
        m_heap_2.pop_back();
        r->m_pop_count++;
        r->set_olp_cn(1); // Mark that the node was popped from open list 2.
        return r;
    }

    // Pop from heap 3.
    Node* pop_3()
    {
        Node* r = m_heap_3.front();
        std::pop_heap(m_heap_3.begin(), m_heap_3.end(), m_third_node_comp);
        m_heap_3.pop_back();
        r->m_pop_count++;
        r->set_olp_ct(1); // Mark that the node was popped from open list 3.
        return r;
    }

    // Peek at the top element of each heap.
    Node* top_heap_1() { return m_heap_1.front(); }
    Node* top_heap_2() { return m_heap_2.front(); }
    Node* top_heap_3() { return m_heap_3.front(); }
};


		template <class Node_Comp, class Node>
		class Custom_Priority_Queue
		{
			public:
				typedef Node Node_Type;
			protected:
				std::vector<Node*> m_heap;
				int m_size_limit;
				int m_last_layer_first_element;
				int m_next;
				Node_Comp m_node_comp;
				// std::random_device m_rd;
				std::mt19937::result_type seed = 42;
    			std::mt19937 m_gen;

			public:

				Custom_Priority_Queue() : m_next(0), m_size_limit(0), m_gen(seed)
				{
					// int max_depth = 17;
					// m_size_limit = pow(2, max_depth+1) - 1; //for index subtract 1
					// m_last_layer_first_element = (m_size_limit / 2) + 1; //for index subtract 1
				}
				~Custom_Priority_Queue() {}

				void init(int max_depth)
				{
					m_size_limit = pow(2, max_depth+1) - 1; //for index subtract 1
					m_last_layer_first_element = (m_size_limit / 2) + 1; //for index subtract 1					
				}

				void set_seed(std::mt19937::result_type new_seed) {
					seed = new_seed;
					m_gen.seed(seed);
				}

				bool empty() const { return m_heap.empty(); }
				std::size_t size() const { return m_heap.size(); }

				void insert(Node *n)
				{
					if (size() < m_size_limit)
					{
						m_heap.push_back(n);
						std::push_heap(m_heap.begin(), m_heap.end(), Node_Comp());
					}
					else
					{
						static std::uniform_int_distribution<> distrib(m_last_layer_first_element, m_size_limit);
						int r_i = distrib(m_gen)-1;
						if (m_node_comp(m_heap[r_i], n))
						{
							// int r_diff = m_size_limit - r;
							// if (n->h1n() < -0.9)
							// {
							// 	std::cout <<"==="<<std::endl;
							// 	std::cout << top()->h1n()<<std::endl;
							// }
							Node* d = m_heap[r_i];
							m_heap[r_i] = n;
							std::push_heap(m_heap.begin(), m_heap.begin()+r_i+1, Node_Comp());
							delete d;
							// if (n->h1n() < -0.9)
							// 	std::cout << top()->h1n()<<std::endl;
						}
						else
							delete n;
					}


				}

				Node* pop()
				{
					if (empty())
						return NULL;
					Node* r = m_heap.front();
					std::pop_heap(m_heap.begin(), m_heap.end(), Node_Comp());
					m_heap.pop_back();
					return r;
				}

				Node* top()
				{
					return m_heap.front();
				}

			protected:
				// void swap(Node* &a, Node* &b)
				// {
				// 	Node* temp = a;
				// 	a = b;
				// 	b = temp;
				// }

				// int parent_i(int curr_i)
				// {
				// 	assert(curr_i >= 0);
				// 	return ( (curr_i+1) / 2 ) - 1;
				// }

				// void heapify_down(int i)
				// {
				// 	bool cont;
				// 	if (!empty())
				// 		cont = true;
				// 	else
				// 		cont = false;

				// 	int best = i;
				// 	while (cont)
				// 	{
				// 		int l_child = 2*i;
				// 		int r_child = l_child + 1;

				// 		if ( l_child <= m_heap.size() && m_node_comp(m_heap[best], m_heap[l_child]) )
				// 			best = l_child;
				// 		if ( r_child <= m_heap.size() && m_node_comp(m_heap[best], m_heap[r_child]) )
				// 			best = r_child;

				// 		if (best != i)
				// 		{
				// 			swap(m_heap[i], m_heap[best]);
				// 			//heapify_down heap, best
				// 			i = best;
				// 		}
				// 		else
				// 			cont = false;
				// 	}
				// }

				// void heapify_up(int i)
				// {
				// 	bool cont = true;

				// 	while (cont)
				// 	{
				// 		int p = parent_i(i);
				// 		if (p == -1)
				// 			cont = false;
				// 		else
				// 		{
				// 			if (m_node_comp(m_heap[p], m_heap[i]))
				// 			{
				// 				swap(m_heap[p], m_heap[i]);
				// 				i = p;
				// 			}
				// 			else
				// 				cont = false;
				// 		}
				// 	}
				// }
		};

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		class Pruned_Open_List
		{
		public:
			typedef Node Node_Type;

			Pruned_Open_List();
			~Pruned_Open_List();

			void set_soft_limit(unsigned l) { m_soft_limit = l; }
			void set_alternating(unsigned bl, unsigned tl)
			{
				m_use_alternating = true;
				m_expanding = true;
				m_soft_bottom_limit = bl;
				m_soft_top_limit = tl;
			}
			float get_th_h1() 
			{
				if (m_node_threshold != nullptr)
					return m_node_threshold->h1n();
				else
					return 99999;
			}
			void insert(Node *);
			Node *pop();
			bool empty() const;
			float min() const;
			void clear();
			size_t size() { return m_queue.size(); }
			size_t inverse_size() { return m_inverse_queue.size(); }
			Node *top() { return m_queue.top(); }
			bool greater_than_th(Node* n);

		private:
			std::priority_queue<Node *, std::vector<Node *>, Node_Comp> m_queue;
			std::priority_queue<Node*, std::vector<Node*>, Inverse_Node_Comp> m_inverse_queue;
			unsigned m_soft_limit;
			Node* m_node_threshold;
			Inverse_Node_Comp m_greater_comp;
			unsigned m_soft_bottom_limit;
			unsigned m_soft_top_limit;
			bool m_use_alternating;
			bool m_expanding;
		};

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::Pruned_Open_List() 
		: m_node_threshold(nullptr), m_soft_limit(0), m_use_alternating(false), m_expanding(false), m_soft_top_limit(0), m_soft_bottom_limit(0)
		{
		}

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::~Pruned_Open_List()
		{
		}		

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		bool Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::greater_than_th(Node* n)
		{
			if (m_node_threshold == nullptr)
			{
				m_node_threshold = n;
				return false;
			}
			else
				return m_greater_comp(m_node_threshold, n);
		}

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		void Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::insert(Node *n)
		{
			if (m_use_alternating)
			{
				if ( (m_queue.size() > m_soft_top_limit) || (!m_expanding && m_queue.size() > m_soft_bottom_limit) )
				{
					m_expanding = false;
					if ( greater_than_th(n) )
					{
						delete n;
					}
					else
					{
						m_queue.push(n);
						m_inverse_queue.pop();
						m_inverse_queue.push(n);
						m_node_threshold = m_inverse_queue.top();
					}
				}
				else
				{
					m_expanding = true;
					m_queue.push(n);
					m_inverse_queue.push(n);
					if ( greater_than_th(n) )
						m_node_threshold = n;					
				}

			}
			else 
			{
				if (m_queue.size() > m_soft_limit)
				{
					if ( greater_than_th(n) )
					{
						delete n;
					}
					else
					{
						m_queue.push(n);
						m_inverse_queue.pop();
						m_inverse_queue.push(n);
						m_node_threshold = m_inverse_queue.top();
					}
				}
				else 
				{
					m_queue.push(n);
					m_inverse_queue.push(n);
					if ( greater_than_th(n) )
						m_node_threshold = n;
				}
			}

		}

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		Node *Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::pop()
		{
			if (empty())
				return NULL;
			Node *elem = m_queue.top();
			m_queue.pop();
			return elem;
		}

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		bool Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::empty() const
		{
			return m_queue.empty();
		}

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		float Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::min() const
		{
			if (empty())
				return 0.0f;
			return m_queue.top()->f;
		}

		template <class Node_Comp, class Inverse_Node_Comp, class Node>
		void Pruned_Open_List<Node_Comp, Inverse_Node_Comp, Node>::clear()
		{
			while (!empty())
			{
				Node *elem = pop();
				delete elem;
				//nodes in m_inverse_queue are either in open list or closed list, so not delete here
			}
		}



		template <class Node_Comp, class Node>
		class Open_List
		{

			// std::priority< Node, std::vector< Node >, Node_Comp > m_queue;
		public:
			typedef Node Node_Type;

			Open_List();
			~Open_List();

			void insert(Node *);
			Node *pop();
			bool empty() const;
			float min() const;
			void clear();
			size_t size() { return m_queue.size(); }
			Node *top() { return m_queue.top(); }
			void init(int max_depth){  } // not used, but needed for compatibility with other Open_List types
			void set_seed(std::mt19937::result_type new_seed) { } // not used, but needed for compatibility with other Open_List types

		private:
			std::priority_queue<Node *, std::vector<Node *>, Node_Comp> m_queue;
		};

		template <class Node_Comp, class Node>
		Open_List<Node_Comp, Node>::Open_List()
		{
		}

		template <typename Node_Comp, typename Node>
		Open_List<Node_Comp, Node>::~Open_List()
		{
		}

		template <typename Node_Comp, typename Node>
		void Open_List<Node_Comp, Node>::insert(Node *n)
		{
			m_queue.push(n);
		}

		template <typename Node_Comp, typename Node>
		Node *Open_List<Node_Comp, Node>::pop()
		{
			if (empty())
				return NULL;
			Node *elem = m_queue.top();
			m_queue.pop();
			return elem;
		}

		template <typename Node_Comp, typename Node>
		bool Open_List<Node_Comp, Node>::empty() const
		{
			return m_queue.empty();
		}

		template <typename Node_Comp, typename Node>
		float Open_List<Node_Comp, Node>::min() const
		{
			if (empty())
				return 0.0f;
			return m_queue.top()->f;
		}

		template <typename Node_Comp, typename Node>
		void Open_List<Node_Comp, Node>::clear()
		{
			while (!empty())
			{
				Node *elem = pop();
				delete elem;
			}
		}

		// MRJ: Open List allowing for nodes to be incrementally sorted when keys are updated. Wraps
		// Boost.Heap.Fibonacci_Heap, that implements Fibonacci heaps.
		//
		// In order to use these heaps, one needs to change the Search Node interface and is required
		// to make appropiate calls as explained below:
		//
		// 1) Interface requirements on Node classes
		//
		// The following types, attributes and methods need to be added:
		//
		// * Types:
		//
		// typedef	Fibonacci_Open_List< Node >	Open_List;
		//
		// This type is defined mostly for convenience and clarity.
		//
		// * Attributes:
		//
		// Open_List::Handle	heap_handle;
		// Open_List*		current;
		//
		// Both attributes need to be public, as the method Fibonacci_Open_List<Node>::insert() will set
		// them respectively to the address of the node inside the heap and a pointer to the Fibonacci_Open_List
		// object (to allow the node to callback Fibonacci_Open_List<Node>::update() when necessary).
		//
		// * Methods:
		//
		// bool	operator<( const Node& b ) const
		//
		// Boost heap libraries require this operator to be overloaded on heap element classes. Note that in order
		// to get an ascending ordering according to f(n1) < f(n2) - i.e. best first - the operator needs to return true whenever
		// f(n1) is greater than f(n2).
		//
		// void notify_update() {
		//	if (current) current->update(this);
		// }
		//
		// This method needs to be called right after any changes are done to the values that govern the ordering of elements in
		// the heap (i.e. right after any changes in the value of f(n)). After calling this method, the heap is guaranteed to
		// be sorted.
		//
		// void detach() {
		//	if (current) current->erase(this);
		// }
		//
		// This methods allow to remove an element from the heap. After calling this method, the heap is guaranteed to be sorted.
		//
		// 2) Example
		//
		// Below a very simple Node class example can be found
		/*
		class Node {
		public:
			typedef Fibonacci_Open_List< Node > 		Open_List;

			State			state;
			float			fn;
			Open_List::Handle	heap_handle;
			Open_List*		current;

			// NOTE: It is very important to make sure the pointer to the current heap is initialized to a known value.
			Node()
			: current(nullptr) {
			}

			bool	operator<( const Node& b ) const {
				return fn > b.fn;
			}

			void notify_update( ) {
				assert( current != nullptr );
				if ( current )
					current->update( this );
			}

			void detach() {
				assert( current != nullptr );
				if ( current )
					current->erase( this );
			}
		};
		*/
		//
		// 3) Declaring open lists
		//
		// The easiest and cleanest way of defining open lists as members of classes implementing search algorithms is as
		// follows:
		//
		// i) Instantiate the following type
		//
		// typedef Node::Open_List	My_BFS_Open_List;
		//
		// ii) Declaring the attribute as
		//
		// My_BFS_Open_List	m_primary_open;

		template <class Node>
		class Fibonacci_Open_List
		{
			struct compare_node
			{
				bool operator()(Node *n1, Node *n2) const
				{
					return *n1 < *n2;
				}
			};

			typedef typename boost::heap::fibonacci_heap<Node *, boost::heap::compare<compare_node>> Container;

			Container m_queue;

		public:
			typedef typename Container::handle_type Handle;

			typedef Node Node_Type;

			Fibonacci_Open_List();
			~Fibonacci_Open_List();

			void insert(Node *);
			void update(Node *);
			Node *pop();
			bool empty() const;
			float min() const;
			void clear();
			Node *first();
			typename Container::ordered_iterator
			begin() const { return m_queue.ordered_begin(); }
			typename Container::ordered_iterator
			end() const { return m_queue.ordered_end(); }
			void erase(Node *);
		};

		template <class Node>
		Fibonacci_Open_List<Node>::Fibonacci_Open_List()
		{
		}

		template <typename Node>
		Fibonacci_Open_List<Node>::~Fibonacci_Open_List()
		{
		}

		template <typename Node>
		void Fibonacci_Open_List<Node>::insert(Node *n)
		{
			typename Container::handle_type handle = m_queue.push(n);
			n->heap_handle = handle;
			n->current = this;
		}

		template <typename Node>
		void Fibonacci_Open_List<Node>::update(Node *n)
		{
			m_queue.update(n->heap_handle);
		}

		template <typename Node>
		void Fibonacci_Open_List<Node>::erase(Node *n)
		{
			m_queue.erase(n->heap_handle);
		}

		template <typename Node>
		Node *Fibonacci_Open_List<Node>::pop()
		{
			if (empty())
				return NULL;
			Node *elem = m_queue.top();
			m_queue.pop();
			elem->current = nullptr;
			return elem;
		}

		template <typename Node>
		bool Fibonacci_Open_List<Node>::empty() const
		{
			return m_queue.empty();
		}

		template <typename Node>
		Node *Fibonacci_Open_List<Node>::first()
		{
			if (empty())
				return nullptr;
			return m_queue.top();
		}

		template <typename Node>
		float Fibonacci_Open_List<Node>::min() const
		{
			if (empty())
				return std::numeric_limits<float>::max();
			return m_queue.top()->fn;
		}

		template <typename Node>
		void Fibonacci_Open_List<Node>::clear()
		{
			while (!empty())
			{
				Node *elem = pop();
				delete elem;
			}
		}

	}

}

#endif // Open_List.hxx
