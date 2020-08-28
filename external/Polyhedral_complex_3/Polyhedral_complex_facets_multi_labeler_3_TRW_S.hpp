#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_MULTI_LABELER_3_TRW_S_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_MULTI_LABELER_3_TRW_S_HPP

#include<algorithm>

#ifndef NDEBUG
# include<iostream>
#endif /* NDEBUG */

#include <TRW_S/MRFEnergy.hpp>

#include <Polyhedral_complex_3/Polyhedral_complex_facets_multi_labeler_3.hpp>

namespace Polyhedral_complex_3 {

template<class Comp>
class Polyhedral_complex_facets_multi_labeler_3_TRW_S :
    public Polyhedral_complex_facets_multi_labeler_3<Comp> {
    public:
        typedef Polyhedral_complex_facets_multi_labeler_3<Comp> Base;
        typedef typename Base::label_type label_type;
        typedef typename Base::cost_type cost_type;
        typedef typename Base::Face Face;
        typedef typename Base::Face_handle Face_handle;

    protected:
        typedef typename Base::Faces_const_iterator Faces_const_iterator;

        typedef TRW_S::MRFEnergy<TRW_S::TypeGeneral> MRF;

    public:
        Polyhedral_complex_facets_multi_labeler_3_TRW_S(const Comp& comp, int number_of_labels, int number_of_iterations = 30) : Base(comp, number_of_labels), _number_of_iterations(number_of_iterations) { }

        /**********************************************************/

        void compute_labeling()
        {
            assert(this->_facet_labels);
            assert(this->_facet_data_costs);

            const int number_of_labels = this->_number_of_labels;
            const int number_of_labels2 = number_of_labels * number_of_labels;
            const int number_of_nodes = this->_comp.number_of_facets();
#ifndef NDEBUG
            int number_of_edges = 0;

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it) {
                const Face& f = *it;
                const Face_handle fh = this->_comp.facet_handle(f);
                number_of_edges += this->_facet_pairs[fh].size();
            }

            std::cout << "Polyhedral_complex_facets_multi_labeler_3_TRW_S:"
                      << std::endl
                      << "    # of labels: " << number_of_labels << std::endl
                      << "    # of nodes: " << number_of_nodes << std::endl
                      << "    # of edges: " << number_of_edges << std::endl;
#endif /* NDEBUG */

            MRF* mrf = new MRF(MRF::GlobalSize());
            MRF::NodeId* nodes = new MRF::NodeId[number_of_nodes];

            /* Facets data costs */

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it) {
                const Face& f = *it;
                const Face_handle fh = this->_comp.facet_handle(f);

                /* Copy the costs */
                _ALLOCA(MRF::REAL, D, number_of_labels);

                for (int i = 0; i != number_of_labels; ++i)
                    D[i] = facet_data_cost(fh, i);

                nodes[fh] = mrf->AddNode(MRF::LocalSize(number_of_labels),
                                         MRF::NodeData(D));
            }

            /* Facets smooth costs */

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it) {
                const Face& f1 = *it;
                const Face_handle fh1 = this->_comp.facet_handle(f1);
                const std::map<Face_handle, int>& pairs =
                    this->_facet_pairs[fh1];

                for (typename std::map<Face_handle, int>::const_iterator it2 =
                         pairs.begin();
                     it2 != pairs.end(); ++it2) {
                    const Face_handle fh2 = it2->first;
                    const int i = it2->second;

                    const cost_type* facet_smooth_costs =
                        &this->_facet_smooth_costs[i];

                    _ALLOCA(MRF::REAL, V, number_of_labels2);

                    /* Copy the costs */
                    for (int j = 0; j != number_of_labels2; ++j)
                        V[j] = facet_smooth_costs[j];

                    const MRF::NodeId node1 = nodes[fh1];
                    const MRF::NodeId node2 = nodes[fh2];

                    mrf->AddEdge(node1, node2,
                                 MRF::EdgeData(TRW_S::TypeGeneral::GENERAL,
                                               V));
                }
            }

            mrf->SetAutomaticOrdering();

            MRF::Options options;
            options.m_iterMax = _number_of_iterations;
            options.m_printIter = 1;
            options.m_printMinIter = 1;

            MRF::REAL lowerBound;
            MRF::REAL energy;
            mrf->Minimize_TRW_S(options, lowerBound, energy);
          //mrf->Minimize_BP(options, energy);

            /* Get the labeling */

            for (Faces_const_iterator it = this->_comp.facets_begin();
                 it != this->_comp.facets_end(); ++it) {
                const Face& f = *it;
                const Face_handle fh = this->_comp.facet_handle(f);

                const MRF::NodeId node = nodes[fh];
                label_facet(fh, mrf->GetSolution(node));
            }

#ifndef NDEBUG
            for (int i = 0; i != number_of_labels; ++i) {
                const int n = this->_count_facets(i);

                std::cout << "        # of " << i << " facets: " << n
                          << std::endl;
            }
#endif /* NDEBUG */

            delete[] nodes;
            delete mrf;
        }

    protected:
        const int _number_of_iterations;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_FACETS_MULTI_LABELER_3_TRW_S_HPP */
