#ifndef POLYHEDRAL_COMPLEX_3_MESH_CLIPPER_3_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_CLIPPER_3_HPP

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
#include<vector>

namespace Polyhedral_complex_3 {

template<class Mesh>
class Mesh_clipper_3 {
    public:
        typedef typename Mesh::value_type value_type;
        typedef typename Mesh::Tuple_3 Tuple_3;
        typedef typename Mesh::Facet Facet;
        typedef typename Mesh::Vertex_handle Vertex_handle;
        typedef typename Mesh::Facets_const_iterator Facets_const_iterator;
        typedef typename Mesh::Vertex_handles_const_iterator
            Vertex_handles_const_iterator;

        typedef typename std::vector<Tuple_3> Tuple_list_3;

    public:
        static void clip(const Mesh& mesh,
                         value_type xmin, value_type ymin, value_type zmin,
                         value_type xmax, value_type ymax, value_type zmax,
                         Mesh& clipped_mesh)
        {
            clipped_mesh.clear();

            /* Iterate over all the facets. */
            for (Facets_const_iterator it = mesh.facets_begin();
                 it != mesh.facets_end(); ++it) {
                const Facet& facet = *it;

                const Tuple_list_3 polygon = _facet_to_polygon(mesh, facet);
                const int num_vertices = polygon.size();

                Tuple_list_3 clipped_polygon(num_vertices + 6);

                const int num_clipped_vertices =
                    _clip(num_vertices, &polygon[0],
                          xmin, ymin, zmin,
                          xmax, ymax, zmax,
                          &clipped_polygon[0]);

                if (num_clipped_vertices < 3)
                    continue;

                clipped_polygon.resize(num_clipped_vertices);

                /* Add the new vertices and the new facet to the new mesh */
                _insert_facet_from_polygon(clipped_mesh, clipped_polygon);
            }
        }

    protected:
        static Tuple_list_3 _facet_to_polygon(const Mesh& mesh,
                                              const Facet& facet)
        {
            Tuple_list_3 polygon;

            for (Vertex_handles_const_iterator it =
                     facet.vertex_handles_begin();
                 it != facet.vertex_handles_end(); ++it) {
                const Vertex_handle& handle = *it;
                const Tuple_3& vertex = mesh.vertex(handle);

                polygon.push_back(vertex);
            }

            return polygon;
        }

        static void _insert_facet_from_polygon(Mesh& mesh,
                                               const Tuple_list_3& polygon)
        {
            Facet facet;

            for (typename Tuple_list_3::const_iterator it = polygon.begin();
                 it != polygon.end(); ++it) {
                const Tuple_3& vertex = *it;

                const Vertex_handle handle = mesh.insert(vertex);

                facet.insert(handle);
            }

            mesh.insert(facet);
        }

        /*!
         *  Specialized Sutherland-Hodgman clipping algorithm for clipping
         *  a polygon against an AABB.
         */
        static int _clip(int num_in_vertices,
                         const Tuple_3* in_vertices,
                         value_type xmin, value_type ymin, value_type zmin,
                         value_type xmax, value_type ymax, value_type zmax,
                         /* Array of (num_in_vertices + 6) vertices */
                         Tuple_3* out_vertices)
        {
            assert(num_in_vertices >= 3);

            const int max_num_out_vertices = num_in_vertices + 6;

            _ALLOCA(Tuple_3, tmp_vertices, max_num_out_vertices);

            std::copy(in_vertices, in_vertices + num_in_vertices,
                      out_vertices);
            int num_out_vertices = num_in_vertices;

            /*
             * NOTE: We alternate between out_vertices and tmp_vertices as
             *       input to the _clip() method to avoid having to copy
             *       back the clipped vertices.
             */

            /* Clip against x - xmin >= 0 *****************************/
            if ((num_out_vertices = _clip(num_out_vertices, out_vertices, 
                                          1, 0, 0, -xmin,
                                          tmp_vertices)) == 0)
                return 0;

            /* Clip against -x + xmax >= 0 ****************************/
            if ((num_out_vertices = _clip(num_out_vertices, tmp_vertices, 
                                          -1,  0, 0, xmax,
                                          out_vertices)) == 0)
                return 0;

            /* Clip against y - ymin >= 0 *****************************/
            if ((num_out_vertices = _clip(num_out_vertices, out_vertices, 
                                          0, 1, 0, -ymin,
                                          tmp_vertices)) == 0)
                return 0;

            /* Clip against -y + ymax >= 0 ****************************/
            if ((num_out_vertices = _clip(num_out_vertices, tmp_vertices, 
                                          0, -1, 0, ymax,
                                          out_vertices)) == 0)
                return 0;

            /* Clip against z - zmin >= 0 *****************************/
            if ((num_out_vertices = _clip(num_out_vertices, out_vertices, 
                                          0, 0, 1, -zmin,
                                          tmp_vertices)) == 0)
                return 0;

            /* Clip against -z + zmax >= 0 ****************************/
            if ((num_out_vertices = _clip(num_out_vertices, tmp_vertices, 
                                          0, 0, -1, zmax,
                                          out_vertices)) == 0)
                return 0;

            return num_out_vertices;
        }

        /*!
         *  Checks whether the given vertex is inside the half-space:
         *    h(v) = a x + b y + c z + d >= 0.
         */
        static bool _is_inside(const Tuple_3& vertex,
                               value_type a, value_type b, value_type c,
                               value_type d,
                               value_type& h)
        {
            h = a * vertex.x + b * vertex.y + c * vertex.z + d;

            return (h >= 0);
        }

        /*!
         *  Computes the intersection point between vertex 0 and 1, reusing
         *  previous half-space function evaluations at vertex 0 and 1:
         *
         *    h(v0) = a x0 + b y0 + c z0 + d (>= 0, for instance)
         *    h(v1) = a x1 + b y1 + c z1 + d ( < 0, then)
         *
         *    h(v) = a x + b y + c z + d = 0 with x = x0 + t (x1 - x0)
         *                                        y = y0 + t (y1 - y0)
         *
         *      for t = -h(v0) / (h(v1) - h(v0))
         *
         */
        static void _compute_intersection(const Tuple_3& in_vertex0,
                                          const Tuple_3& in_vertex1,
                                          Tuple_3& out_vertex,
                                          value_type h0,
                                          value_type h1)
        {
            const value_type t = -h0 / (h1 - h0);

            out_vertex.x = in_vertex0.x + t * (in_vertex1.x - in_vertex0.x);
            out_vertex.y = in_vertex0.y + t * (in_vertex1.y - in_vertex0.y);
            out_vertex.z = in_vertex0.z + t * (in_vertex1.z - in_vertex0.z);
        }

        /*! Clips against the half-space a x + b y + c z + d >= 0. */
        static int _clip(int num_in_vertices,
                         const Tuple_3* in_vertices,
                         /* Inward oriented line equation */
                         value_type a, value_type b, value_type c, 
                         value_type d,
                         Tuple_3* out_vertices)
        {
            const Tuple_3 *last_vertex = &in_vertices[num_in_vertices - 1];

            value_type last_h, current_h;
            bool last_is_inside = _is_inside(*last_vertex, a, b, c, d, last_h);
            bool current_is_inside;
          
            int num_out_vertices = 0;

            for (int i = 0; i != num_in_vertices; ++i) {
                const Tuple_3* current_vertex = in_vertices + i;

                /* Current vertex is inside */
                if ((current_is_inside =
                         _is_inside(*current_vertex, a, b, c, d, current_h))) {
                    /*
                     * Last vertex is outside and current one is not on the
                     * clipping line
                     */
                    if (!last_is_inside && current_h != 0) {
                        _compute_intersection(*last_vertex, *current_vertex,
                                              out_vertices[num_out_vertices],
                                              last_h, current_h);
                        num_out_vertices++;

                        if (num_out_vertices > num_in_vertices + 1)
                            return 0;
                    }

                    out_vertices[num_out_vertices] = *current_vertex;
                    num_out_vertices++;

                    if (num_out_vertices > num_in_vertices + 1)
                        return 0;
                } else { /* Current vertex is outside */
                    /* Last vertex is inside and not on the clipping line */
                    if (last_is_inside && last_h != 0) {
                        _compute_intersection(*current_vertex, *last_vertex,
                                              out_vertices[num_out_vertices],
                                              current_h, last_h);
                        num_out_vertices++;

                        if (num_out_vertices > num_in_vertices + 1)
                            return 0;
                    }
                }

                last_vertex = current_vertex;
                last_h = current_h;
                last_is_inside = current_is_inside;
            }

            return num_out_vertices;
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_CLIPPER_3_HPP */
