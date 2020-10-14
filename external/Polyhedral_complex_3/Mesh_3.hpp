#ifndef POLYHEDRAL_COMPLEX_3_MESH_3_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_3_HPP

#include<algorithm>
#include<vector>

namespace Polyhedral_complex_3 {

template<class T = double>
class Mesh_3 {
    public:
        typedef T value_type;
        typedef int info_type;

    public:
        struct Tuple_3 {
            public:
                Tuple_3() { }
     
                Tuple_3(value_type x,
                        value_type y,
                        value_type z) : x(x), y(y), z(z) { }

                Tuple_3 operator+(const Tuple_3& v) const
                {
                    return Tuple_3(x + v.x,
                                   y + v.y,
                                   z + v.z);
                }

                Tuple_3 operator-(const Tuple_3& v) const
                {
                    return Tuple_3(x - v.x,
                                   y - v.y,
                                   z - v.z);
                }

                Tuple_3 operator/(value_type f) const
                {
                    return Tuple_3(x / f,
                                   y / f,
                                   z / f);
                }

                static Tuple_3 cross(const Tuple_3& v1, const Tuple_3& v2)
                {
                    return Tuple_3(v1.y * v2.z - v1.z * v2.y,
                                   v1.z * v2.x - v1.x * v2.z,
                                   v1.x * v2.y - v1.y * v2.x);
                }

                static value_type dot(const Tuple_3& v1, const Tuple_3& v2)
                {
                    return (v1.x * v2.x
                          + v1.y * v2.y
                          + v1.z * v2.z);
                }

            public:
                value_type x, y, z;
        };

        class Facet {
            public:
                typedef int Vertex_handle;

                typedef std::vector<Vertex_handle>::const_iterator
                    Vertex_handles_const_iterator;
                typedef std::vector<Vertex_handle>::iterator
                    Vertex_handles_iterator;

            public:
                Facet() : _info(-1), _is_bounded(true) { }

                Facet(info_type info) : _info(info), _is_bounded(true) { }

                bool is_bounded() const { return _is_bounded; }

                bool& is_bounded() { return _is_bounded; }

                int number_of_vertex_handles() const { return int(_vertex_handles.size()); }

                int number_of_vertices() const
                {
                    const int n = number_of_vertex_handles();

                    return (is_bounded() ? n : n - 2);
                }

                Vertex_handles_const_iterator vertex_handles_begin() const { return _vertex_handles.begin(); }

                Vertex_handles_iterator vertex_handles_begin() { return _vertex_handles.begin(); }

                Vertex_handles_const_iterator vertex_handles_end() const { return _vertex_handles.end(); }

                Vertex_handles_iterator vertex_handles_end() { return _vertex_handles.end(); }

                Vertex_handle vertex_handle(int i) const { return _vertex_handles[i]; }

                Vertex_handle& vertex_handle(int i) { return _vertex_handles[i]; }

                void insert(Vertex_handle vh)
                {
                    assert(!has_vertex_handle(vh));

                    _vertex_handles.push_back(vh);
                }

                bool has_vertex_handle(Vertex_handle vh) const
                {
                    Vertex_handles_const_iterator first =
                        _vertex_handles.begin();
                    Vertex_handles_const_iterator last =
                        _vertex_handles.end();

                    return (std::find(first, last, vh) != last);
                }

                void flip_normal()
                {
                    const int n = number_of_vertex_handles();
                    const int m = n / 2;

                    for (int i = 0; i != m; ++i)
                        std::swap(_vertex_handles[        i],
                                  _vertex_handles[n - 1 - i]);
                }

                info_type info() const { return _info; }

                info_type& info() { return _info; }

            protected:
                std::vector<Vertex_handle> _vertex_handles;
                bool _is_bounded;
            public:
                info_type _info;
        };

    public:
        typedef typename std::vector<Tuple_3>::const_iterator
            Vertices_const_iterator;
        typedef typename std::vector<Tuple_3>::iterator
            Vertices_iterator;

        typedef typename Facet::Vertex_handle Vertex_handle;
        typedef typename Facet::Vertex_handles_const_iterator
            Vertex_handles_const_iterator;
        typedef typename Facet::Vertex_handles_iterator
            Vertex_handles_iterator;
        typedef int Facet_handle;

        typedef typename std::vector<Facet>::const_iterator
            Facets_const_iterator;
        typedef typename std::vector<Facet>::iterator
            Facets_iterator;

    public:
        Vertex_handle insert(const Tuple_3& v)
        {
            _vertices.push_back(v);

            return (number_of_vertices() - 1);
        }

        Facet_handle insert(const Facet& f)
        {
            _facets.push_back(f);

            return (number_of_facets() - 1);
        }

        void clear()
        {
            _facets.clear();
            _vertices.clear();
        }

        /* Vertices ***************************/

        Vertices_const_iterator vertices_begin() const { return _vertices.begin(); }

        Vertices_iterator vertices_begin() { return _vertices.begin(); }

        Vertices_const_iterator vertices_end() const { return _vertices.end(); }

        Vertices_iterator vertices_end() { return _vertices.end(); }

        int number_of_vertices() const { return int(_vertices.size()); }

        const Tuple_3& vertex(Vertex_handle vh) const { return _vertices[vh]; }

        Tuple_3& vertex(Vertex_handle vh) { return _vertices[vh]; }

        const Tuple_3& vertex(Facet_handle fh, int i) const { return vertex(facet(fh).vertex_handle(i)); }

        Tuple_3& vertex(Facet_handle fh, int i) { return vertex(facet(fh).vertex_handle(i)); }

        Vertex_handle vertex_handle(const Tuple_3& v) const { return Vertex_handle(&v - &_vertices[0]); }

        /* Facets *****************************/

        Facets_const_iterator facets_begin() const { return _facets.begin(); }

        Facets_iterator facets_begin() { return _facets.begin(); }

        Facets_const_iterator facets_end() const { return _facets.end(); }

        Facets_iterator facets_end() { return _facets.end(); }

        int number_of_facets() const { return int(_facets.size()); }

        const Facet& facet(Facet_handle fh) const { return _facets[fh]; }

        Facet& facet(Facet_handle fh) { return _facets[fh]; }

        Facet_handle facet_handle(const Facet& f) const { return Facet_handle(&f - &_facets[0]); }

        /**************************************/

        /*! Computes the (non-normalized) normal of the facet. */
        Tuple_3 compute_facet_normal(const Facet& f) const
        {
            const int number_of_vertex_handles = f.number_of_vertex_handles();

            const Vertex_handle vh1 = f.vertex_handle(1);
            const Vertex_handle vh2 = f.vertex_handle(2);

            const Tuple_3& p1 = vertex(vh1);
            const Tuple_3& p2 = vertex(vh2);

            Tuple_3 p0;

            if (number_of_vertex_handles == 3) {
                const Vertex_handle vh0 = f.vertex_handle(0);

                p0 = vertex(vh0);
            } else {
                p0 = Tuple_3(0, 0, 0);

                for (int i = 0; i != number_of_vertex_handles; ++i) {
                    const Vertex_handle vh = f.vertex_handle(i);
                    const Tuple_3& p = vertex(vh);

                    p0 = p0 + (p - p0) / (i + 1);
                }
            }

            const Tuple_3 v10 = p1 - p0;
            const Tuple_3 v20 = p2 - p0;

            return Tuple_3::cross(v10, v20);
        }

        Tuple_3 compute_facet_normal(Facet_handle fh) const { return compute_facet_normal(facet(fh)); }

        /*! Changes the orientation of the facet. */
        void flip_facet_normal(Facet& f) { f.flip_normal(); }

        void flip_facet_normal(Facet_handle fh) { flip_facet_normal(facet(fh)); }

    protected:
        std::vector<Tuple_3> _vertices;
        std::vector<Facet> _facets;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_3_HPP */
