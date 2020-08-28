#ifndef POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_POLY_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_POLY_HPP

#include <Polyhedral_complex_3/Mesh_3_writer.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
class Mesh_3_writer_POLY : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet_handle Facet_handle;
        typedef typename Base::Vertex_handle Vertex_handle;

    public:
        Mesh_3_writer_POLY(const Mesh& mesh) : Base(mesh, false) { }

    protected:
        void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const { }

        /* Node list: no attribute, no boundary marker */
        void _write_vertices_begin(std::ostream& stream, int number_of_vertices) const { stream << "# node list\n" << number_of_vertices << " 3 0 0\n"; }

        void _write_vertex_begin(std::ostream& stream, Vertex_handle vh) const { stream << (vh + 1) << ' '; }

        /* Facet list: no boundary markers */
        void _write_facets_begin(std::ostream& stream, int number_of_facets) const { stream << "# facet list\n" << number_of_facets << " 0\n"; }

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_indices) const { stream << "1 0\n" << number_of_indices << ' '; }

        void _write_facet_vertex_index(std::ostream& stream, Facet_handle fh, int index) const { stream << (index + 1); }

        void _write_end(std::ostream& stream) const { stream << "# hole list\n0\n# region list\n0\n"; }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_POLY_HPP */
