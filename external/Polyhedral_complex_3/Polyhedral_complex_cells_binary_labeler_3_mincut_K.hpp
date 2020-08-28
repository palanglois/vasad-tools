#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_K_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_K_HPP

#include<algorithm>

#ifndef NDEBUG
# include<iostream>
#endif /* NDEBUG */

#include <maxflow/Graph.hpp>

#include <Polyhedral_complex_3/Polyhedral_complex_cells_binary_labeler_3.hpp>

namespace Polyhedral_complex_3 {

template<class Comp,
       //class CostT = float,
         class CostT = double,
         class CapacityT = int>
       //class CapacityT = long long>
class Polyhedral_complex_cells_binary_labeler_3_mincut_K :
    public Polyhedral_complex_cells_binary_labeler_3<Comp,
                                                     CostT,
                                                     CapacityT> {
    public:
        typedef Polyhedral_complex_cells_binary_labeler_3<Comp,
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
        Polyhedral_complex_cells_binary_labeler_3_mincut_K(const Comp& comp) :
            Base(comp),
            _cell_labels(0) { reserve(); }

        void clear()
        {
            _clear_cell_labels();
            Base::clear();
        }

        void reserve()
        {
            Base::reserve();
            _reserve_cell_labels();
        }

        /**********************************************************/

        bool is_cell_inside(Face_handle ch) const { return _cell_labels[ch]; }

        bool is_cell_inside(const Face& c) const { return is_cell_inside(this->_comp.cell_handle(c)); }

        void label_cell(Face_handle ch, bool inside) { _cell_labels[ch] = inside; }

        void label_cell(const Face& c, bool inside) { label_cell(this->_comp.cell_handle(c), inside); }

        /**********************************************************/

        void compute_labeling(bool free_costs = false)
        {
            assert(_cell_labels);
            assert(this->_cell_costs);
            assert(this->_facet_costs[0]);
            assert(this->_facet_costs[1]);

#ifndef NDEBUG
            std::cout << "Polyhedral_complex_cells_binary_labeler_3_mincut_K:"
                      << std::endl << "    building graph" << std::endl;
#endif /* NDEBUG */

            /* Build a copy of the graph for maxflow computation */
            const int number_of_vertices = this->_comp.number_of_cells();
            const int number_of_edges = this->_comp.number_of_facets();

            Gr graph(number_of_vertices, number_of_edges);
            graph.add_node(number_of_vertices);

            /* Assign costs */
            int i = 0;

            for (Faces_const_iterator it = this->_comp.cells_begin();
                 it != this->_comp.cells_end(); ++it, ++i) {
                const Face_handle ch = Face_handle(i);

                /* Scale and round cost */
                const cost_type cost = cell_cost(ch);

                /* Convert to capacity */
                const capacity_type capacity = _map_cost_to_capacity(cost);

                graph.add_tweights(i, 0, capacity);
            }

            /* Deallocate the cell costs */
            if (free_costs)
                this->_clear_cell_costs();

            i = 0;

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it, ++i) {
                const Face_handle fh = Face_handle(i);

                const Face_handle ch1 = it->superface(0);
                const Face_handle ch2 = it->superface(1);

                /* Scale and round costs */
                const cost_type cost1 = facet_cost(fh, 0);
                const cost_type cost2 = facet_cost(fh, 1);

                /* Convert to capacities */
                const capacity_type capacity1 = _map_cost_to_capacity(cost1);
                const capacity_type capacity2 = _map_cost_to_capacity(cost2);

                graph.add_edge(ch1, ch2, capacity1, capacity2);
            }

            /* Deallocate the facet costs */
            if (free_costs)
                this->_clear_facet_costs();

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
             * Store result in cells: if both labels are possible, take the
             * outside label
             */
            i = 0;

            for (Faces_const_iterator it = this->_comp.cells_begin();
                 it != this->_comp.cells_end(); ++it, ++i) {
                const Face_handle ch = this->_comp.cell_handle(*it);

                this->label_cell(ch,
                                 graph.what_segment(i,
                                                    Gr::SOURCE) == Gr::SINK);
            }

#ifndef NDEBUG
            const int number_of_inside_cells =
                this->_count_inside_cells();
            const int number_of_outside_cells =
                number_of_vertices
              - number_of_inside_cells;

            std::cout << "        # of inside cells: "
                      << number_of_inside_cells << std::endl
                      << "        # of outside cells: "
                      << number_of_outside_cells << std::endl;
#endif /* NDEBUG */
        }

        /**********************************************************/

        bool operator()(Face_handle ch) const { return is_cell_inside(ch); }

        bool operator()(const Face& c) const { return is_cell_inside(c); }

    protected:
        bool* _cell_labels; // std::vector<bool> would be more space efficient

    protected:
        void _clear_cell_labels()
        {
            delete[] _cell_labels;
            _cell_labels = 0;
        }

        void _reserve_cell_labels()
        {
            _clear_cell_labels(); /* Safeguard */

            const int number_of_cells = this->_comp.number_of_cells();

            assert(_cell_labels == 0);
            _cell_labels = new bool[number_of_cells];

            std::fill(_cell_labels,
                      _cell_labels + number_of_cells,
                      false);
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_K_HPP */
