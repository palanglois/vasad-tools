#ifndef POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_OFF_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_OFF_HPP

#include <Polyhedral_complex_3/Mesh_3_writer.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh>
class Mesh_3_writer_OFF : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet_handle Facet_handle;

    public:
        Mesh_3_writer_OFF(const Mesh& mesh) : Base(mesh) { }

    protected:
        void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const { stream << "OFF\n" << number_of_vertices << ' ' << number_of_facets << ' ' << "0\n"; }

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_indices) const { stream << number_of_indices << ' '; }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_OFF_HPP */
