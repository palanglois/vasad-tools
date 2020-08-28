#ifndef POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_PLY_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_PLY_HPP

#include <Polyhedral_complex_3/Mesh_3_writer.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
class Mesh_3_writer_PLY : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet_handle Facet_handle;

    public:
        Mesh_3_writer_PLY(const Mesh& mesh) : Base(mesh, true) { }

    protected:
        void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const
        {
            stream << "ply\n"
                      "format ascii 1.0\n"
                      "element vertex " << number_of_vertices << "\n"
                      "property float x\n"
                      "property float y\n"
                      "property float z\n"
                      "element face " << number_of_facets << "\n"
                      "property list uchar int vertex_index\n"
                      "property float nx\n"
                      "property float ny\n"
                      "property float nz\n"
                      "end_header\n";
        }

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_vertices) const { stream << number_of_vertices << ' '; }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_PLY_HPP */
