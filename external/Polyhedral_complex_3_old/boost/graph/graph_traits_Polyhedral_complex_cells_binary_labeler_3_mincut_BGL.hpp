#ifndef BOOST_GRAPH_TRAITS_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_BGL_HPP
#define BOOST_GRAPH_TRAITS_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_BGL_HPP

#include <boost/graph/graph_traits.hpp>
#include <Polyhedral_complex_3/Polyhedral_complex_cells_binary_labeler_3_mincut_BGL.hpp>

namespace Polyhedral_complex_3 {

template<class Comp,
         class CostT,
         class CapacityT>
class Polyhedral_complex_cells_binary_labeler_3_mincut_BGL;

} /* Polyhedral_complex_3 */

namespace boost {

template<class Comp,
         class CostT,
         class CapacityT>
struct graph_traits<
    Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<
        Comp,
        CostT,
        CapacityT
    >
>
{
    public:
        typedef Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<
            Comp,
            CostT,
            CapacityT
        > Labeler;

        typedef typename Labeler::vertices_size_type vertices_size_type;
        typedef typename Labeler::vertex_descriptor vertex_descriptor;
        typedef typename Labeler::vertex_iterator vertex_iterator;

        typedef typename Labeler::edges_size_type edges_size_type;
        typedef typename Labeler::edge_descriptor edge_descriptor;
        typedef typename Labeler::edge_iterator edge_iterator;

        typedef typename Labeler::degree_size_type degree_size_type;
        typedef typename Labeler::out_edge_iterator out_edge_iterator;

        typedef directed_tag directed_category;
        typedef disallow_parallel_edge_tag edge_parallel_category; 

    public:
        struct traversal_category : public virtual incidence_graph_tag,
                                    public virtual edge_list_graph_tag,
                                    public virtual vertex_list_graph_tag { };

    public:
        static vertex_descriptor null_vertex() { return vertex_descriptor(-3); }

};

template<class Comp, class CostT, class CapacityT>
std::pair<typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_iterator,
		  typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_iterator>
vertices(const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g) { return g.vertices(); }

template<class Comp, class CostT, class CapacityT>
std::pair<typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::edge_iterator,
          typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::edge_iterator>
edges(const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.edges(); }

template<class Comp, class CostT, class CapacityT>
typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertices_size_type
num_vertices(const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.num_vertices(); }

template<class Comp, class CostT, class CapacityT>
typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::edges_size_type
num_edges(const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.num_edges(); }

template<class Comp, class CostT, class CapacityT>
typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::degree_size_type
out_degree(typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_descriptor v,
		   const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.out_degree(v); }

template<class Comp, class CostT, class CapacityT>
std::pair<typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::out_edge_iterator,
          typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::out_edge_iterator>
out_edges(typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_descriptor v,
		const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.out_edges(v); }

template<class Comp, class CostT, class CapacityT>
std::pair<typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::edge_descriptor, bool>
edge(typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_descriptor u,
		typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_descriptor v,
		const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.edge(u, v); } 

template<class Comp, class CostT, class CapacityT>
typename boost::graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_descriptor
source(typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::edge_descriptor e,
		const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.source(e); }

template<class Comp, class CostT, class CapacityT>
typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::vertex_descriptor
target(typename graph_traits<Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT> >::edge_descriptor e,
		const Polyhedral_complex_3::Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<Comp, CostT, CapacityT>& g)
{ return g.target(e); }
} /* boost */

#endif /* BOOST_GRAPH_TRAITS_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_BGL_HPP */
