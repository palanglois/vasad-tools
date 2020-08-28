#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_BINARY_LABELER_3_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_BINARY_LABELER_3_HPP

#include<algorithm>
#include<cassert>
#include<limits>
#include<map>
#include<vector>

#include <Polyhedral_complex_3/Polyhedral_complex_labeler_3.hpp>

namespace Polyhedral_complex_3 {

/*!
 *  Binary labeling of the facets of a polyhedral complex.
 *
 *  \code
 *    // Allocate memory for costs and labels
 *    Polyhedral_complex_facets_binary_labeler_3<Comp> labeler(comp);
 *    Mesh_extractor_3<Comp, Mesh> extractor(comp);
 *
 *    // Assign data & smooth costs
 *    ...
 *
 *    // Compute the labeling
 *    labeler.compute_labeling();
 *    extractor.extract(mesh, labeler);
 *
 *    // Clear everything else (labels)
 *    labeler.clear();
 *
 *    or
 *
 *    // Reallocate memory for costs and labels
 *    labeler.reserve();
 *
 *    // Assign again facet data & smooth costs
 *    ...
 *
 *    // Compute again the labeling
 *    labeler.compute_labeling();
 *    ...
 *  \endcode
 *  \note source/sink: selected/not selected, false/true
 */
template<class Comp,
       //class CostT = float,
         class CostT = double,
         class CapacityT = int>
       //class CapacityT = long long>
class Polyhedral_complex_facets_binary_labeler_3 {
    public:
        typedef CostT cost_type;
        typedef CapacityT capacity_type;

        typedef typename Comp::Face Face;
        typedef typename Comp::Face_handle Face_handle;

        typedef Facets_binary_labeler_tag Labeler_category;

    protected:
        typedef typename Comp::Faces_const_iterator Faces_const_iterator;

        typedef Polyhedral_complex_facets_binary_labeler_3<Comp> Self;

    public:
        Polyhedral_complex_facets_binary_labeler_3(const Comp& comp) :
            _comp(comp),
            _facet_costs(0),
            _edges(0)
        {
            reserve();
        }

        ~Polyhedral_complex_facets_binary_labeler_3() { clear(); }

        virtual void clear()
        {
            _clear_edge_costs();
            _clear_facet_costs();
        }

        void reserve()
        {
            _reserve_facet_costs();
            _reserve_edge_costs();
        }

        /**********************************************************/

        virtual bool is_facet_selected(Face_handle fh) const = 0;

        virtual bool is_facet_selected(const Face& f) const = 0;

        virtual void label_facet(Face_handle fh, bool selected) = 0;

        virtual void label_facet(const Face& f, bool selected) = 0;

        /**********************************************************/

        /* Facets *****************************/

        cost_type facet_cost(Face_handle fh) const { return _facet_costs[fh]; }

        cost_type& facet_cost(Face_handle fh) { return _facet_costs[fh]; }

        cost_type facet_cost(const Face& f) const { return facet_cost(_comp.facet_handle(f)); }

        cost_type& facet_cost(const Face& f) { return facet_cost(_comp.facet_handle(f)); }

        /* Edges ******************************/

        static cost_type max_edge_cost() { return std::numeric_limits<cost_type>::max(); }

        const cost_type* edge_costs(Face_handle fh1, Face_handle fh2) const
        {
            if (fh1 > fh2)
                std::swap(fh1, fh2);

            const std::map<Face_handle, int>& pairs = _edges[fh1];
            const typename std::map<Face_handle, int>::const_iterator it =
                pairs.find(fh2);

            if (it == pairs.end())
                return 0;

            const int i = it->second;

            return &_edge_costs[i];
        }

        cost_type edge_cost(Face_handle fh1, Face_handle fh2) const
        {
            const cost_type* costs = edge_costs(fh1, fh2);

            if (costs == 0)
                return max_edge_cost();

            return costs[fh1 < fh2 ? 0 : 1];
        }

        cost_type* edge_costs(Face_handle fh1, Face_handle fh2)
        {
            if (fh1 > fh2)
                std::swap(fh1, fh2);

            std::map<Face_handle, int>& pairs = _edges[fh1];
            const typename std::map<Face_handle, int>::const_iterator it =
                pairs.find(fh2);

            if (it == pairs.end()) {
                const int old_size = _edge_costs.size();
                const int new_size = old_size + 2;
                const int i = old_size;

                _edge_costs.resize(new_size, 0);
                pairs[fh2] = i;

                return &_edge_costs[i];
            }

            const int i = it->second;

            return &_edge_costs[i];
        }

        cost_type& edge_cost(Face_handle fh1, Face_handle fh2)
        {
            cost_type* costs = edge_costs(fh1, fh2);

            assert(costs != 0);

            return costs[fh1 < fh2 ? 0 : 1];
        }

        cost_type edge_cost(const Face& f1, const Face& f2) const { return edge_cost(_comp.facet_handle(f1), _comp_facet_handle(f2)); }

        cost_type& edge_cost(const Face& f1, const Face& f2) { return edge_cost(_comp.facet_handle(f1), _comp.facet_handle(f2)); }

        cost_type opposite_edge_cost(Face_handle fh1, Face_handle fh2) const { return edge_cost(fh2, fh1); }

        cost_type& opposite_edge_cost(Face_handle fh1, Face_handle fh2) { return edge_cost(fh2, fh1); }

        cost_type opposite_edge_cost(const Face& f1, const Face& f2) const { return opposite_edge_cost(_comp.facet_handle(f1), _comp.facet_handle(f2)); }

        cost_type& opposite_edge_cost(const Face& f1, const Face& f2) { return opposite_edge_cost(_comp.facet_handle(f1), _comp.facet_handle(f2)); }

        /**********************************************************/

    protected:
        static capacity_type _map_cost_to_capacity(cost_type cost)
        {
            if (cost > 0)
                return capacity_type(cost + cost_type(0.5));
            else if (cost < 0)
                return capacity_type(cost - cost_type(0.5));
            else
                return capacity_type(0);
        }

    public:
        /*! Computes the labeling. */
        virtual void compute_labeling(bool free_costs = false) = 0;

        /**********************************************************/

        /*!
         *  Selected/not selected predicates: to be called once the labelling
         *  has been computed.
         */
        virtual bool operator()(Face_handle ch) const = 0;

        virtual bool operator()(const Face& c) const = 0;

    protected:
        const Comp& _comp;
        cost_type* _facet_costs;
        std::map<Face_handle, int>* _edges;
        std::vector<cost_type> _edge_costs;

    private:
        /* Prevent from copying */
        Polyhedral_complex_facets_binary_labeler_3(const Self& labeler);

        Self& operator=(const Self& labeler);

    protected:
        void _clear_facet_costs()
        {
            delete[] _facet_costs;
            _facet_costs = 0;
        }

        void _clear_edge_costs()
        {
            _edge_costs.clear();

            delete[] _edges;
            _edges = 0;
        }

        void _reserve_facet_costs()
        {
            _clear_facet_costs();

            const int number_of_facets = _comp.number_of_facets();

            assert(_facet_costs == 0);
            _facet_costs = new cost_type[number_of_facets];

            std::fill(_facet_costs,
                      _facet_costs + number_of_facets,
                      0);
        }

        void _reserve_edge_costs()
        {
            _clear_edge_costs();

            const int number_of_facets = _comp.number_of_facets();

            assert(_edges == 0);
            _edges = new std::map<Face_handle, int>[number_of_facets];
        }

        int _count_selected_facets() const
        {
            int number_of_selected_facets = 0;

            for (Faces_const_iterator it = _comp.facets_begin();
                 it != _comp.facets_end(); ++it) {
                const Face_handle fh = _comp.facet_handle(*it);

                if (is_facet_selected(fh))
                    ++number_of_selected_facets;
            }

            return number_of_selected_facets;
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_BINARY_LABELER_3_HPP */
