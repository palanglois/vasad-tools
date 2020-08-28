#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_HPP

#include<algorithm>
#include<cassert>

#include <Polyhedral_complex_3/Polyhedral_complex_labeler_3.hpp>

namespace Polyhedral_complex_3 {

/*!
 *  Binary labeling of the cells of a polyhedral complex.
 *
 *  \code
 *    // Allocate memory for costs and labels and tag each cell
 *    Polyhedral_complex_cells_binary_labeler_3<Comp> labeler(comp);
 *    Mesh_extractor_3<Comp, Mesh> extractor(comp);
 *
 *    // Assign cell & facet costs
 *    ...
 *
 *    // Compute the labeling and also free cell and facet costs
 *    labeler.compute_labeling();
 *    extractor.extract(mesh, labeler);
 *
 *    // Clear everything else (labels)
 *    labeler.clear();
 *
 *    or
 *
 *    // Reallocate memory for costs and labels and tag each cell
 *    labeler.reserve();
 *
 *    // Assign again cell & facet costs
 *    ...
 *
 *    // Compute again the labeling and also free cell and facet costs
 *    labeler.compute_labeling();
 *    ...
 *  \endcode
 *  \note source/sink: outside/inside, false/true
 */
template<class Comp,
       //class CostT = float,
         class CostT = double,
         class CapacityT = int>
       //class CapacityT = long long>
class Polyhedral_complex_cells_binary_labeler_3 {
    public:
        typedef CostT cost_type;
        typedef CapacityT capacity_type;

        typedef typename Comp::Face Face;
        typedef typename Comp::Face_handle Face_handle;

        typedef Cells_binary_labeler_tag Labeler_category;

    protected:
        typedef typename Comp::Faces_const_iterator Faces_const_iterator;

        typedef Polyhedral_complex_cells_binary_labeler_3<Comp,
                                                          CostT,
                                                          CapacityT> Self;

    public:
        Polyhedral_complex_cells_binary_labeler_3(const Comp& comp) :
            _comp(comp),
            _cell_costs(0)
        {
            _facet_costs[0] = 0;
            _facet_costs[1] = 0;

            reserve();
        }

        ~Polyhedral_complex_cells_binary_labeler_3() { clear(); }

        virtual void clear()
        {
            _clear_facet_costs();
            _clear_cell_costs();
        }

        void reserve()
        {
            _reserve_cell_costs();
            _reserve_facet_costs();
        }

        /**********************************************************/

        virtual bool is_cell_inside(Face_handle ch) const = 0;

        virtual bool is_cell_inside(const Face& c) const = 0;

        virtual void label_cell(Face_handle ch, bool inside) = 0;

        virtual void label_cell(const Face& c, bool inside) = 0;

        /**********************************************************/

        /* Cells ******************************/

        cost_type cell_cost(Face_handle ch) const { return _cell_costs[ch]; }

        cost_type& cell_cost(Face_handle ch) { return _cell_costs[ch]; }

        cost_type cell_cost(const Face& c) const { return cell_cost(_comp.cell_handle(c)); }

        cost_type& cell_cost(const Face& c) { return cell_cost(_comp.cell_handle(c)); }

        /* Facets *****************************/

        /* The cost a facet f from its superface i to its other superface */

        cost_type facet_cost(Face_handle fh, int i) const
        {
            assert(i >= 0 && i <= 1);

            return _facet_costs[i][fh];
        }

        cost_type& facet_cost(Face_handle fh, int i)
        {
            assert(i >= 0 && i <= 1);

            return _facet_costs[i][fh];
        }

        cost_type facet_cost(const Face& f, int i) const { return facet_cost(_comp.facet_handle(f), i); }

        cost_type& facet_cost(const Face& f, int i) { return facet_cost(_comp.facet_handle(f), i); }

        cost_type opposite_facet_cost(Face_handle fh, int i) const
        {
            assert(i >= 0 && i <= 1);

            return _facet_costs[1 - i][fh];
        }

        cost_type& opposite_facet_cost(Face_handle fh, int i)
        {
            assert(i >= 0 && i <= 1);

            return _facet_costs[1 - i][fh];
        }

        cost_type opposite_facet_cost(const Face& f, int i) const { return opposite_facet_cost(_comp.facet_handle(f), i); }

        cost_type& opposite_facet_cost(const Face& f, int i) { return opposite_facet_cost(_comp.facet_handle(f), i); }

        /**********************************************************/

    protected:
        static capacity_type _map_cost_to_capacity(cost_type cost)
        {
			return capacity_type(cost);
            //if (cost > 0)
            //    return capacity_type(cost + cost_type(0.5));
            //else if (cost < 0)
            //    return capacity_type(cost - cost_type(0.5));
            //else
            //    return capacity_type(0);
        }

    public:
        /*!
         *  Computes the labeling. Allows to free costs array
         *  as soon as they are not needed.
         */
        virtual void compute_labeling(bool free_costs = false) = 0;

        /**********************************************************/

        /*!
         *  Inside/outside predicates: to be called once the labelling
         *  has been computed.
         */
        virtual bool operator()(Face_handle ch) const = 0;

        virtual bool operator()(const Face& c) const = 0;

    protected:
        const Comp& _comp;
        cost_type* _cell_costs;
        cost_type* _facet_costs[2];

    private:
        /* Prevent from copying */
        Polyhedral_complex_cells_binary_labeler_3(const Self& labeler);

        Self& operator=(const Self& labeler);

    protected:
        void _clear_cell_costs()
        {
            delete[] _cell_costs;
            _cell_costs = 0;
        }

        void _clear_facet_costs()
        {
            delete[] _facet_costs[1];
            _facet_costs[1] = 0;

            delete[] _facet_costs[0];
            _facet_costs[0] = 0;
        }

        void _reserve_cell_costs()
        {
            _clear_cell_costs(); /* Safeguard */

            const int number_of_cells = _comp.number_of_cells();

            assert(_cell_costs == 0);
            _cell_costs = new cost_type[number_of_cells];

            std::fill(_cell_costs, _cell_costs + number_of_cells, 0);
        }

        void _reserve_facet_costs()
        {
            _clear_facet_costs(); /* Safeguard */

            const int number_of_facets = _comp.number_of_facets();

            assert(_facet_costs[0] == 0);
            _facet_costs[0] = new cost_type[number_of_facets];

            assert(_facet_costs[1] == 0);
            _facet_costs[1] = new cost_type[number_of_facets];

            std::fill(_facet_costs[0], _facet_costs[0] + number_of_facets, 0);
            std::fill(_facet_costs[1], _facet_costs[1] + number_of_facets, 0);
        }

        int _count_inside_cells() const
        {
            int number_of_inside_cells = 0;

            for (Faces_const_iterator it = _comp.cells_begin();
                 it != _comp.cells_end(); ++it) {
                const Face_handle ch = _comp.cell_handle(*it);

                if (is_cell_inside(ch))
                    ++number_of_inside_cells;
            }

            return number_of_inside_cells;
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_HPP */
