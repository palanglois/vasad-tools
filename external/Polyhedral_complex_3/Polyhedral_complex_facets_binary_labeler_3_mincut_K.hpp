#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_BINARY_LABELER_3_MINCUT_K_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_BINARY_LABELER_3_MINCUT_K_HPP

#include<algorithm>

#ifndef NDEBUG
# include<iostream>
#endif /* NDEBUG */

#include <maxflow/Graph.hpp>

#include <Polyhedral_complex_3/Polyhedral_complex_facets_binary_labeler_3.hpp>

namespace Polyhedral_complex_3 {

template<class Comp,
       //class CostT = float,
         class CostT = double,
         class CapacityT = int>
       //class CapacityT = long long>
class Polyhedral_complex_facets_binary_labeler_3_mincut_K :
    public Polyhedral_complex_facets_binary_labeler_3<Comp,
                                                      CostT,
                                                      CapacityT> {
    public:
        typedef Polyhedral_complex_facets_binary_labeler_3<Comp,
                                                           CostT,
                                                           CapacityT> Base;
        typedef typename Base::cost_type cost_type;
        typedef typename Base::capacity_type capacity_type;
        typedef typename Base::Face Face;
        typedef typename Base::Face_handle Face_handle;

    protected:
        typedef typename Base::Faces_const_iterator Faces_const_iterator;

        typedef maxflow::Graph<CapacityT,
                               CapacityT,
                               CapacityT> Gr;

    public:
        Polyhedral_complex_facets_binary_labeler_3_mincut_K(const Comp& comp) :
            Base(comp),
            _facet_labels(0) { reserve(); }

        void clear()
        {
            _clear_facet_labels();
            Base::clear();
        }

        void reserve()
        {
            Base::reserve();
            _reserve_facet_labels();
        }

        /**********************************************************/

        bool is_facet_selected(Face_handle fh) const { return _facet_labels[fh]; }

        bool is_facet_selected(const Face& f) const { return is_facet_selected(this->_comp.facet_handle(f)); }

        void label_facet(Face_handle fh, bool selected) { _facet_labels[fh] = selected; }

        void label_facet(const Face& f, bool selected) { label_facet(this->_comp.facet_handle(f), selected); }

        /**********************************************************/

        void compute_labeling(bool free_costs = false)
        {
            assert(_facet_labels);
            assert(this->_facet_costs);
            assert(this->_edges);

#ifndef NDEBUG
            std::cout << "Polyhedral_complex_facets_binary_labeler_3_mincut_K:"
                      << std::endl << "    building graph" << std::endl;
#endif /* NDEBUG */

            /* Build a copy of the graph for maxflow computation */
            const int number_of_vertices = this->_comp.number_of_facets();
            /* Compute the # of edges */
            int number_of_edges = 0;

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it) {
                const Face& f = *it;
                const Face_handle fh = this->_comp.facet_handle(f);
                number_of_edges += this->_edges[fh].size();
            }

            Gr graph(number_of_vertices, number_of_edges);
            graph.add_node(number_of_vertices);

            /* Assign costs */
            int i = 0;

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it, ++i) {
                const Face_handle fh = Face_handle(i);

                /* Scale and round cost */
                const cost_type cost = facet_cost(fh);

                /* Convert to capacity */
                const capacity_type capacity = _map_cost_to_capacity(cost);

                graph.add_tweights(i, 0, capacity);

              //std::cout << "    facet " << fh << ": "
              //          << 0 << "/" << capacity << std::endl;
            }

            /* Deallocate the facet costs */
            if (free_costs)
                this->_clear_facet_costs();

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it) {
                const Face& f1 = *it;
                const Face_handle fh1 = this->_comp.facet_handle(f1);
                const std::map<Face_handle, int>& pairs =
                    this->_edges[fh1];

                for (typename std::map<Face_handle, int>::const_iterator it2 =
                         pairs.begin();
                     it2 != pairs.end(); ++it2) {
                    const Face_handle fh2 = it2->first;

                    /* Scale and round costs */
                    const cost_type cost1 = edge_cost(fh1, fh2);
                    const cost_type cost2 = edge_cost(fh2, fh1);

                    /* Convert to capacities */
                    const capacity_type capacity1 =
                        _map_cost_to_capacity(cost1);
                    const capacity_type capacity2 =
                        _map_cost_to_capacity(cost2);

                    graph.add_edge(fh1, fh2, capacity1, capacity2);

                  //std::cout << "    edge (" << fh1 << "," << fh2 << "): "
                  //          << capacity1 << "/" << capacity2 << std::endl;
                }
            }

            /* Deallocate the edge costs */
            if (free_costs)
                this->_clear_edge_costs();

#ifndef NDEBUG
            std::cout << "        # of vertices: "
                      << number_of_vertices << std::endl
                      << "        # of edges: "
                      << number_of_edges << std::endl
                      << "    computing minimum s-t cut" << std::endl;
#endif /* NDEBUG */

            /* Compute mincut/maxflow */
            graph.maxflow();

            /* 
             * Store result in facets: if both labels are possible, take the
             * not selected label
             */
            i = 0;

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it, ++i) {
                const Face& f = *it;
                const Face_handle fh = this->_comp.facet_handle(f);

                this->label_facet(fh,
                                  graph.what_segment(i,
                                                     Gr::SOURCE) == Gr::SINK);
            }

#ifndef NDEBUG
            const int number_of_selected_facets =
                this->_count_selected_facets();
            const int number_of_not_selected_facets =
                number_of_vertices
              - number_of_selected_facets;

            std::cout << "        # of selected facets: "
                      << number_of_selected_facets << std::endl
                      << "        # of not selected facets: "
                      << number_of_not_selected_facets << std::endl;
#endif /* NDEBUG */
        }

        /**********************************************************/

        bool operator()(Face_handle fh) const { return is_facet_selected(fh); }

        bool operator()(const Face& f) const { return is_facet_selected(f); }

    protected:
        bool* _facet_labels;

    protected:
        void _clear_facet_labels()
        {
            delete[] _facet_labels;
            _facet_labels = 0;
        }

        void _reserve_facet_labels()
        {
            _clear_facet_labels(); /* Safeguard */

            const int number_of_facets = this->_comp.number_of_facets();

            assert(_facet_labels == 0);
            _facet_labels = new bool[number_of_facets];

            std::fill(_facet_labels,
                      _facet_labels + number_of_facets,
                      false);
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_BINARY_LABELER_3_MINCUT_K_HPP */
