#ifndef POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_WAVEFRONT_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_WAVEFRONT_HPP

#include <Polyhedral_complex_3/Mesh_3_writer.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
class Mesh_3_writer_wavefront : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet_handle Facet_handle;
        typedef typename Base::Vertex_handle Vertex_handle;

    public:
        Mesh_3_writer_wavefront(const Mesh& mesh) : Base(mesh) { }

    protected:
        void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const { };

        void _write_vertex_begin(std::ostream& stream, Vertex_handle vh) const { stream << "v "; }

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_indices) const { stream << "f "; }

        void _write_facet_vertex_index(std::ostream& stream, Facet_handle fh, int index) const { stream << (index + 1); }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_WAVEFRONT_HPP */
