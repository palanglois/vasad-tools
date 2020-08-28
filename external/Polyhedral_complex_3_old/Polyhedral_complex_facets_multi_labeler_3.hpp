#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_MULTI_LABELER_3_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_MULTI_LABELER_3_HPP

/* Variable-length arrays */
#ifndef _ALLOCA
# ifdef _MSC_VER
#  include <malloc.h>
#  define _ALLOCA(type, name, length) \
       type* name = static_cast<type*>(_alloca((length) * sizeof(type)))
# else
#  define _ALLOCA(type, name, length) \
       type name[length]
# endif /* _MSC_VER */
#endif /* _ALLOCA */

#include<algorithm>
#include<cassert>
#include<limits>
#include<map>
#include<vector>

#include <Polyhedral_complex_3/Polyhedral_complex_labeler_3.hpp>

namespace Polyhedral_complex_3 {

/*!
 *  Multi-labeling of the facets of a polyhedral complex.
 *
 *  \code
 *    // Allocate memory for costs and labels
 *    Polyhedral_complex_facets_multi_labeler_3<Comp> labeler(comp,
 *                                                            number_of_labels);
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
 */
template<class Comp>
class Polyhedral_complex_facets_multi_labeler_3 {
    public:
        typedef int cost_type;
        typedef int label_type;

        typedef typename Comp::Face Face;
        typedef typename Comp::Face_handle Face_handle;

        typedef Facets_multi_labeler_tag Labeler_category;

    protected:
        typedef typename Comp::Faces_const_iterator Faces_const_iterator;

        typedef Polyhedral_complex_facets_multi_labeler_3<Comp> Self;

    public:
        Polyhedral_complex_facets_multi_labeler_3(const Comp& comp,
                                                  int number_of_labels) :
            _comp(comp),
            _number_of_labels(number_of_labels),
            _facet_labels(0),
            _facet_data_costs(0),
            _facet_pairs(0)
        {
            assert(number_of_labels > 1);
            reserve();
        }

        ~Polyhedral_complex_facets_multi_labeler_3() { clear(); }

        void clear()
        {
            _clear_facet_smooth_costs();
            _clear_facet_data_costs();
            _clear_facet_labels();
        }

        void reserve()
        {
            _reserve_facet_labels();
            _reserve_facet_data_costs();
            _reserve_facet_smooth_costs();
        }

        /**********************************************************/

        int number_of_labels() const { return _number_of_labels; } 

        /**********************************************************/

        label_type facet_label(Face_handle fh) const { return _facet_labels[fh]; }

        label_type facet_label(const Face& f) const { return facet_label(_comp.facet_handle(f)); }

        void label_facet(Face_handle fh, label_type l) { _facet_labels[fh] = l; }

        void label_facet(const Face& f, label_type l) { label_facet(_comp.facet_handle(f), l); }

        /**********************************************************/

        /* Data costs *************************/

        cost_type facet_data_cost(Face_handle fh, label_type l) const { return _facet_data_costs[fh * _number_of_labels + l]; }

        cost_type& facet_data_cost(Face_handle fh, label_type l) { return _facet_data_costs[fh * _number_of_labels + l]; }

        cost_type facet_data_cost(const Face& f, label_type l) const { return facet_data_cost(_comp.facet_handle(f), l); }

        cost_type& facet_data_cost(const Face& f, label_type l) { return facet_data_cost(_comp.facet_handle(f), l); }

        /* Smooth costs ***********************/

        static cost_type max_facet_smooth_cost() { return std::numeric_limits<cost_type>::max(); }

        const cost_type* facet_smooth_costs(Face_handle fh1, Face_handle fh2) const
        {
            if (fh1 > fh2)
                std::swap(fh1, fh2);

            const std::map<Face_handle, int>& pairs = _facet_pairs[fh1];
            const typename std::map<Face_handle, int>::const_iterator it =
                pairs.find(fh2);

            if (it == pairs.end())
                return 0;

            const int i = it->second;

            return &_facet_smooth_costs[i];
        }

        cost_type facet_smooth_cost(Face_handle fh1, Face_handle fh2,
                                    label_type l1, label_type l2) const
        {
            if (fh1 > fh2)
                std::swap(l1, l2);

            const cost_type* costs = smooth_costs(fh1, fh2);;

            if (costs == 0)
                return max_facet_smooth_cost();

            return costs[l1 + l2 * _number_of_labels];
        }

        cost_type* facet_smooth_costs(Face_handle fh1, Face_handle fh2)
        {
            if (fh1 > fh2)
                std::swap(fh1, fh2);

            std::map<Face_handle, int>& pairs = _facet_pairs[fh1];
            const typename std::map<Face_handle, int>::const_iterator it =
                pairs.find(fh2);

            if (it == pairs.end()) {
                const int old_size = _facet_smooth_costs.size();
                const int new_size = old_size + _number_of_labels
                                              * _number_of_labels;
                const int i = old_size;

                _facet_smooth_costs.resize(new_size, 0);
                pairs[fh2] = i;

                return &_facet_smooth_costs[i];
            }

            const int i = it->second;

            return &_facet_smooth_costs[i];
        }

        cost_type& facet_smooth_cost(Face_handle fh1, Face_handle fh2,
                                      label_type l1, label_type l2)
        {
            if (fh1 > fh2)
                std::swap(l1, l2);

            cost_type* costs = facet_smooth_costs(fh1, fh2);;

            assert(costs != 0);

            return costs[l1 + l2 * _number_of_labels];
        }

        cost_type facet_smooth_cost(const Face& f1, const Face& f2, label_type l1, label_type l2) const { return facet_smooth_cost(_comp.facet_handle(f1), _comp_facet_handle(f2), l1, l2); }

        cost_type& facet_smooth_cost(const Face& f1, const Face& f2, label_type l1, label_type l2) { return facet_smooth_cost(_comp.facet_handle(f1), _comp.facet_handle(f2), l1, l2); }

        /**********************************************************/

    public:
        /*! Computes the labeling. */
        virtual void compute_labeling() = 0;

        /**********************************************************/

        /*! To be called once the labelling has been computed. */
        label_type operator()(Face_handle fh) const { return facet_label(fh); }

        label_type operator()(const Face& f) const { return facet_label(f); }

    protected:
        const Comp& _comp;
        const int _number_of_labels;
        label_type* _facet_labels;
        cost_type* _facet_data_costs;
        std::map<Face_handle, int>* _facet_pairs;
        std::vector<cost_type> _facet_smooth_costs;

    private:
        /* Prevent from copying */
        Polyhedral_complex_facets_multi_labeler_3(const Self& labeler);

        Self& operator=(const Self& labeler);

    protected:
        void _clear_facet_labels()
        {
            delete[] _facet_labels;
            _facet_labels = 0;
        }

        void _clear_facet_data_costs()
        {
            delete[] _facet_data_costs;
            _facet_data_costs = 0;
        }

        void _clear_facet_smooth_costs()
        {
            _facet_smooth_costs.clear();

            delete[] _facet_pairs;
            _facet_pairs = 0;
        }

        void _reserve_facet_labels()
        {
            _clear_facet_labels();

            const int number_of_facets = _comp.number_of_facets();

            assert(_facet_labels == 0);
            _facet_labels = new label_type[number_of_facets];

            std::fill(_facet_labels,
                      _facet_labels + number_of_facets,
                      0);
        }

        void _reserve_facet_data_costs()
        {
            _clear_facet_data_costs();

            const int number_of_facets = _comp.number_of_facets();

            assert(_facet_data_costs == 0);
            const int size = number_of_facets * _number_of_labels;
            _facet_data_costs = new cost_type[size];

            std::fill(_facet_data_costs,
                      _facet_data_costs + size,
                      0);
        }

        void _reserve_facet_smooth_costs()
        {
            _clear_facet_smooth_costs();

            const int number_of_facets = _comp.number_of_facets();

            assert(_facet_pairs == 0);
            _facet_pairs = new std::map<Face_handle, int>[number_of_facets];
        }

        int _count_facets(label_type l) const
        {
            int number_of_facets = 0;

            for (Faces_const_iterator it = _comp.facets_begin();
                 it != _comp.facets_end(); ++it) {
                const Face_handle fh = _comp.facet_handle(*it);

                if (facet_label(fh) == l)
                    ++number_of_facets;
            }

            return number_of_facets;
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_MULTI_LABELER_3_HPP */
