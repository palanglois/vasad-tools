#ifndef POLYHEDRAL_COMPLEX_3_MESH_WITH_FACET_INFO_3_WRITER_PLY_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_WITH_FACET_INFO_3_WRITER_PLY_HPP

#include <Polyhedral_complex_3/Mesh_3_writer.hpp>

namespace Polyhedral_complex_3 {

template<class Mesh,
         class Colormap>
class Mesh_with_facet_info_3_writer_PLY : public Mesh_3_writer<Mesh> {
    protected:
        typedef Mesh_3_writer<Mesh> Base;
        typedef typename Base::Facet Facet;
        typedef typename Base::Vertex_handle Vertex_handle;
        typedef typename Base::Facet_handle Facet_handle;
        typedef typename Mesh::info_type info_type;

        typedef typename Colormap::Color Color;

    public:
        Mesh_with_facet_info_3_writer_PLY(const Mesh& mesh, const Colormap& colormap) : Base(mesh, true /* triangulate */), _colormap(colormap) { }

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
                      "property uchar red\n"
                      "property uchar green\n"
                      "property uchar blue\n"
                      "end_header\n";
        }

        void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_vertices) const { stream << number_of_vertices << ' '; }

        void _write_facet_end(std::ostream& stream, Facet_handle fh) const
        {
            const Facet& f = this->_mesh.facet(fh);
            const info_type info = f.info();
            unsigned char r, g, b;

            if (info < 0) { /* No info -> grey */
                r = 127;
                g = 127;
                b = 127;
            } else {
                const Color& color = _colormap[info];
                r = color.r();
                g = color.g();
                b = color.b();
            }

            stream << ' ' << int(r)
                   << ' ' << int(g)
                   << ' ' << int(b);

            Base::_write_facet_end(stream, fh);
        }

    protected:
        const Colormap& _colormap;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_PLY_HPP */
