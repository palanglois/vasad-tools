#ifndef POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_HPP

#include<cassert>
#include<iostream>

namespace Polyhedral_complex_3 {

template<class Mesh>
class Mesh_3_writer {
    protected:
        typedef typename Mesh::value_type value_type;

        typedef typename Mesh::Tuple_3 Tuple_3;
        typedef typename Mesh::Vertices_const_iterator
            Vertices_const_iterator;
        typedef typename Mesh::Vertex_handle Vertex_handle;

        typedef typename Mesh::Facet Facet;
        typedef typename Mesh::Facets_const_iterator
            Facets_const_iterator;
        typedef typename Mesh::Facet_handle Facet_handle;

        typedef typename Mesh::Vertex_handles_const_iterator
            Vertex_handles_const_iterator;

    public:
        Mesh_3_writer(const Mesh& mesh,
                      bool triangulate = false) :
            _mesh(mesh),
            _triangulate(triangulate) { }

        void write(std::ostream& stream) const
        {
            const int number_of_vertices = _mesh.number_of_vertices();
            int number_of_facets;

            const Facets_const_iterator first_facet = _mesh.facets_begin();
            const Facets_const_iterator  last_facet = _mesh.facets_end();

            if (!_triangulate) {
                number_of_facets = _mesh.number_of_facets();
            } else {
                number_of_facets = 0;

                /* Compute the # of triangles */
                for (Facets_const_iterator it = first_facet;
                     it != last_facet; ++it) {
                    const Facet& f = *it;
                    const int number_of_vertices = f.number_of_vertices();

                    assert(number_of_vertices >= 3);

                    number_of_facets += number_of_vertices - 2;
                }
            }

            _write_header(stream,
                          number_of_vertices,
                          number_of_facets);

            _write_begin(stream);

            /* Vertices ***************************/

            _write_vertices_begin(stream, number_of_vertices);

            const Vertices_const_iterator first_vertex = _mesh.vertices_begin();
            const Vertices_const_iterator  last_vertex = _mesh.vertices_end();

            for (Vertices_const_iterator it = first_vertex;
                 it != last_vertex; ++it) {
                const Vertex_handle vh = _mesh.vertex_handle(*it);
                const Tuple_3& p = _mesh.vertex(vh);

                _write_vertex_begin(stream, vh);
                _write_vertex(stream, vh, p.x, p.y, p.z);
                _write_vertex_end(stream, vh);
            }

            _write_vertices_end(stream);

            /* Facets *****************************/

            _write_facets_begin(stream, number_of_facets);

            for (Facets_const_iterator it = first_facet;
                 it != last_facet; ++it) {
                const Facet& f = *it;
                const Facet_handle fh = _mesh.facet_handle(f);

                if (!_triangulate) {
                    const int number_of_indices = f.number_of_vertices();
                    _write_facet_begin(stream, fh, number_of_indices);

                    const Vertex_handles_const_iterator first =
                        f.vertex_handles_begin();
                    const Vertex_handles_const_iterator last =
                        f.vertex_handles_end();

                    for (Vertex_handles_const_iterator it = first;
                         it != last; ++it) {
                        if (it != first)
                            stream << ' ';

                        const int index = *it;
                        _write_facet_vertex_index(stream, fh, index);
                    }

                    const Tuple_3 n = _mesh.compute_facet_normal(fh);
                    auto norm = sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
                    stream << " " << n.x << " " << n.y << " " << n.z ;

                    _write_facet_end(stream, fh);
                } else {
                    const Vertex_handles_const_iterator first =
                        f.vertex_handles_begin();
                    const Vertex_handles_const_iterator last =
                        f.vertex_handles_end();

                    Vertex_handles_const_iterator it = first;
                    const int index0 = *it;
                    ++it;

                    int index1 = *it;
                    ++it;

                    do {
                        const int index2 = *it;

                        _write_facet_begin(stream, fh, 3);
                        _write_facet_vertex_index(stream, fh, index0);
                        stream << ' ';
                        _write_facet_vertex_index(stream, fh, index1);
                        stream << ' ';
                        _write_facet_vertex_index(stream, fh, index2);

                        const Tuple_3 n = _mesh.compute_facet_normal(fh);
                        auto norm = sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
                        if(norm == 0.) norm = 1.;
                        stream << " " << n.x/norm << " " << n.y/norm << " " << n.z/norm ;

                        _write_facet_end(stream, fh);

                        index1 = index2;
                    } while (++it != last);
                }
            }

            _write_facets_end(stream);

            _write_end(stream);
        }

    protected:
        virtual void _write_header(std::ostream& stream, int number_of_vertices, int number_of_facets) const = 0;

        virtual void _write_begin(std::ostream& stream) const { }

        /* Vertices ***************************/

        virtual void _write_vertices_begin(std::ostream& stream, int number_of_vertices) const { }

        virtual void _write_vertex_begin(std::ostream& stream, Vertex_handle vh) const { }

        virtual void _write_vertex(std::ostream& stream, Vertex_handle vh, value_type x, value_type y, value_type z) const { stream << x << ' ' << y << ' ' << z; }

        virtual void _write_vertex_end(std::ostream& stream, Vertex_handle vh) const { stream << '\n'; }

        virtual void _write_vertices_end(std::ostream& stream) const { }

        /* Facets *****************************/

        virtual void _write_facets_begin(std::ostream& stream, int number_of_facets) const { }

        virtual void _write_facet_begin(std::ostream& stream, Facet_handle fh, int number_of_indices) const { }

        virtual void _write_facet_vertex_index(std::ostream& stream, Facet_handle fh, int index) const { stream << index; }

        virtual void _write_facet_end(std::ostream& stream, Facet_handle fh) const { stream << '\n'; }

        virtual void _write_facets_end(std::ostream& stream) const { }

        /**************************************/

        virtual void _write_end(std::ostream& stream) const { }

    protected:
        const Mesh& _mesh;
        const bool _triangulate;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_WRITER_HPP */
