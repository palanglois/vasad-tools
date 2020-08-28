#ifndef POLYHEDRAL_COMPLEX_3_MESH_EXTRACTOR_3_HPP
#define POLYHEDRAL_COMPLEX_3_MESH_EXTRACTOR_3_HPP

#include<cassert>
#include<map>
#include<vector>

#include <CGAL/basic.h>

#include <Polyhedral_complex_3/Polyhedral_complex_labeler_3.hpp>

namespace Polyhedral_complex_3 {

/*!
 *  Extracts a (non-manifold) polygonal mesh from an inside/outside
 *  selection of cells of a polyhedral complex.
 *
 *  \note If the complex does have a bounding box and a labeler is
 *        not given, infinite facets are not extracted.
 */
template<class Comp,
         class Mesh>
class Mesh_extractor_3 {
    protected:
        typedef typename Comp::Face Face;
        typedef typename Comp::Face_handle Face_handle;

    protected:
        typedef typename Comp::Faces_const_iterator Faces_const_iterator;
        typedef typename Comp::Plane_handle Plane_handle;
        typedef typename Comp::Plane Plane;
        typedef typename Comp::Point Point;
        typedef typename Comp::Vector Vector;

        typedef typename Mesh::Tuple_3 Tuple_3;
        typedef typename Mesh::Facet Mesh_facet;
        typedef typename Mesh::Vertex_handle Mesh_vertex_handle;
        typedef typename Mesh::Facet_handle Mesh_facet_handle;
        typedef typename Mesh::info_type info_type;

    public:
        /*! Constructs a mesh extractor for the complex. */
        Mesh_extractor_3(const Comp& comp) : _comp(comp) { }

        /*! Extracts a mesh using no labeler. */
        void extract(Mesh& mesh,
                     bool keep_bounded_bbox_facet = false) const { _extract(mesh, keep_bounded_bbox_facet); }

        /*! Extracts a mesh using a labeler. */
        template<class Labeler>
        void extract(Mesh& mesh,
                     const Labeler& labeler,
                     bool keep_bounded_bbox_facet = false) const { _extract(mesh, labeler, keep_bounded_bbox_facet, typename Labeler::Labeler_category()); }

    protected:
        /*!
         *  Extracts a mesh from the complex.
         *  \param[out] mesh a mesh
         */
        void _extract(Mesh& mesh,
                      bool keep_bounded_bbox_facet) const
        {
            typedef std::map<Face_handle, Mesh_vertex_handle> Vertex_map;
            typedef std::map<Face_handle, Mesh_vertex_handle> Edge_map;

            typedef typename Vertex_map::const_iterator
                Vertex_map_const_iterator;
            typedef typename Edge_map::const_iterator
                Edge_map_const_iterator;

            mesh.clear();

            Vertex_map vertex_map;
            Edge_map edge_map;

            const bool has_bbox = _comp.has_bbox();

            const Faces_const_iterator first = _comp.facets_begin();
            const Faces_const_iterator  last = _comp.facets_end();

            /* Iterate over all the facets */
            for (Faces_const_iterator it = first; it != last; ++it) {
                const Face& f = *it;

                if(! f.to_draw){
                	continue;
                }

                /*
                 * If the complex has a bounding box, skip the unbounded
                 * facets of the bounding box.
                 */
                const bool is_bbox_facet = _comp.is_bbox_facet(f);

                if (has_bbox && is_bbox_facet
                 && !keep_bounded_bbox_facet)
                       continue;

                std::vector<Face_handle> hs;
                const bool is_bounded =
                    _comp.facet_to_polygon(f, std::back_inserter(hs));

                if (has_bbox && is_bbox_facet
                 && keep_bounded_bbox_facet && !is_bounded)
                    continue;

                const Plane_handle plh = _comp.facet_plane(f);

                /* Create a new facet of the final polygonal mesh */
                Mesh_facet mf(plh);
                mf.is_bounded() = is_bounded;

                const int num_hs = hs.size();

                for (int i = 0; i != num_hs; ++i) {
                    const Face_handle fh = hs[i];

                    Point p = CGAL::ORIGIN;

                    Mesh_vertex_handle mvh = -1;

                    /* Unbounded face and first or last handle: edge handle */
                    const bool is_edge = !is_bounded && (i == 0
                                                      || i == num_hs - 1);

                    if (is_edge) { /* Check if we already met this edge */
                        Edge_map_const_iterator it2 = edge_map.find(fh);

                        if (it2 != edge_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.edge(fh).point();
                    } else { /* Check if we already met this vertex */
                        Vertex_map_const_iterator it2 = vertex_map.find(fh);

                        if (it2 != vertex_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.point(fh);
                    }

                    /*
                     * Add the vertex to the vertices and update the
                     * corresponding map
                     */
                    if (mvh == -1) {
                        const Tuple_3 tuple(CGAL::to_double(p.x()),
                                            CGAL::to_double(p.y()),
                                            CGAL::to_double(p.z()));
                        mvh = mesh.insert(tuple);

                        if (is_edge)
                            edge_map[fh] = mvh;
                        else
                            vertex_map[fh] = mvh;
                    }

                    mf.insert(mvh);
                }

                const Mesh_facet_handle mfh = mesh.insert(mf);

                /* Ensure the orientation of the facet is correct */

                /* Normal of the plane */
                const Plane& pl = _comp.plane(plh);
                const Vector u = pl.orthogonal_vector();
                const Tuple_3 v = Tuple_3(CGAL::to_double(u.x()),
                                          CGAL::to_double(u.y()),
                                          CGAL::to_double(u.z()));

                /* Normal of the facet */
                const Tuple_3 n = mesh.compute_facet_normal(mfh);

                const double d = Tuple_3::dot(n, v);

                /* Facet oriented like the plane */
                if (d < 0)
                    mesh.flip_facet_normal(mfh);
            }

            edge_map.clear();
            vertex_map.clear();
        }

        /*!
         *  Extracts a mesh from the complex.
         *  \param[out] mesh a mesh
         *  \param[in] labeler a cell labeler
         */
        template<class Labeler>
        void _extract(Mesh& mesh,
                      const Labeler& labeler,
                      bool keep_bounded_bbox_facet,
                      Cells_binary_labeler_tag tag) const
        {
            typedef std::map<Face_handle, Mesh_vertex_handle> Vertex_map;
            typedef std::map<Face_handle, Mesh_vertex_handle> Edge_map;

            typedef typename Vertex_map::const_iterator
                Vertex_map_const_iterator;
            typedef typename Edge_map::const_iterator
                Edge_map_const_iterator;

            mesh.clear();

            Vertex_map vertex_map;
            Edge_map edge_map;

            const bool has_bbox = _comp.has_bbox();

            const Faces_const_iterator first = _comp.facets_begin();
            const Faces_const_iterator  last = _comp.facets_end();

            /* Iterate over all the facets */
            for (Faces_const_iterator it = first; it != last; ++it) {
                const Face& f = *it;
                const Face_handle ch1 = f.superface(0);
                const Face_handle ch2 = f.superface(1);

				if (!_comp.is_cell_bounded(ch1) || !_comp.is_cell_bounded(ch2)) continue;

                /*
                 * If the complex has a bounding box, skip the unbounded
                 * facets of the bounding box.
                 */
                const bool is_bbox_facet = _comp.is_bbox_facet(f);

                if (has_bbox && is_bbox_facet
                 && !keep_bounded_bbox_facet)
                       continue;

                const bool inside1 = labeler(ch1);
                const bool inside2 = labeler(ch2);

                /* Skip facets not lying on the interface */
                if (inside1 == inside2)
                    continue;

                std::vector<Face_handle> hs;
                const bool is_bounded =
                    _comp.facet_to_polygon(f, std::back_inserter(hs));

                if (has_bbox && is_bbox_facet
                 && keep_bounded_bbox_facet && !is_bounded)
                    continue;

                const Plane_handle plh = _comp.facet_plane(f);

                /* Create a new facet of the final polygonal mesh */
                Mesh_facet mf(plh);
                mf.is_bounded() = is_bounded;

                const int num_hs = int(hs.size());

                for (int i = 0; i != num_hs; ++i) {
                    const Face_handle fh = hs[i];

                    Point p = CGAL::ORIGIN;

                    Mesh_vertex_handle mvh = -1;

                    /* Unbounded face and first or last handle: edge handle */
                    const bool is_edge = !is_bounded && (i == 0
                                                      || i == num_hs - 1);

                    if (is_edge) { /* Check if we already met this edge */
                        Edge_map_const_iterator it2 = edge_map.find(fh);

                        if (it2 != edge_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.edge(fh).point();
                    } else { /* Check if we already met this vertex */
                        Vertex_map_const_iterator it2 = vertex_map.find(fh);

                        if (it2 != vertex_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.point(fh);
                    }

                    /*
                     * Add the vertex to the vertices and update the
                     * corresponding map
                     */
                    if (mvh == -1) {
                        const Tuple_3 tuple(CGAL::to_double(p.x()),
                                            CGAL::to_double(p.y()),
                                            CGAL::to_double(p.z()));
                        mvh = mesh.insert(tuple);

                        if (is_edge)
                            edge_map[fh] = mvh;
                        else
                            vertex_map[fh] = mvh;
                    }

                    mf.insert(mvh);
                }

                const Mesh_facet_handle mfh = mesh.insert(mf);

                /* Ensure the orientation of the facet is correct */

                /* Normal of the facet */
                const Tuple_3 n = mesh.compute_facet_normal(mfh);
                const Point p = _comp.cell(ch1).point();
                const Tuple_3 Mc = Tuple_3(CGAL::to_double(p.x()),
                                           CGAL::to_double(p.y()),
                                           CGAL::to_double(p.z()));
                const Tuple_3& Mf = mesh.vertex(mfh, 0);

                /* Inward pointing vector */
                const Tuple_3 v = Mc - Mf;

                if (inside1) {
                    if (Tuple_3::dot(n, v) > 0)
                        mesh.flip_facet_normal(mfh);
                } else { /* !inside1 */
                    if (Tuple_3::dot(n, v) < 0)
                        mesh.flip_facet_normal(mfh);
                }
            }

            edge_map.clear();
            vertex_map.clear();
        }

        /*!
         *  Extracts a mesh from the complex.
         *  \param[out] mesh a mesh
         *  \param[in] labeler a facet labeler
         */
        template<class Labeler>
        void _extract(Mesh& mesh,
                      const Labeler& labeler,
                      bool keep_bounded_bbox_facet,
                      Facets_multi_labeler_tag tag) const
        {
            typedef std::map<Face_handle, Mesh_vertex_handle> Vertex_map;
            typedef std::map<Face_handle, Mesh_vertex_handle> Edge_map;

            typedef typename Vertex_map::const_iterator
                Vertex_map_const_iterator;
            typedef typename Edge_map::const_iterator
                Edge_map_const_iterator;

            mesh.clear();

            Vertex_map vertex_map;
            Edge_map edge_map;

            const bool has_bbox = _comp.has_bbox();

            const Faces_const_iterator first = _comp.facets_begin();
            const Faces_const_iterator  last = _comp.facets_end();

            /* Iterate over all the facets */
            for (Faces_const_iterator it = first; it != last; ++it) {
                const Face& f = *it;

                /*
                 * If the complex has a bounding box, skip the unbounded
                 * facets of the bounding box.
                 */
                const bool is_bbox_facet = _comp.is_bbox_facet(f);

                if (has_bbox && is_bbox_facet
                 && !keep_bounded_bbox_facet)
                       continue;

                const Face_handle fh = _comp.facet_handle(f);

                const int label = labeler(fh) - 1; // XXX

                /* Skip facets not lying on the interface */
                if (label == 0)
                    continue;

                std::vector<Face_handle> hs;
                const bool is_bounded =
                    _comp.facet_to_polygon(f, std::back_inserter(hs));

                if (has_bbox && is_bbox_facet
                 && keep_bounded_bbox_facet && !is_bounded)
                    continue;

                const Plane_handle plh = _comp.facet_plane(f);

                /* Create a new facet of the final polygonal mesh */
                Mesh_facet mf(plh);
                mf.is_bounded() = is_bounded;

                const int num_hs = hs.size();

                for (int i = 0; i != num_hs; ++i) {
                    const Face_handle fh = hs[i];

                    Point p = CGAL::ORIGIN;

                    Mesh_vertex_handle mvh = -1;

                    /* Unbounded face and first or last handle: edge handle */
                    const bool is_edge = !is_bounded && (i == 0
                                                      || i == num_hs - 1);

                    if (is_edge) { /* Check if we already met this edge */
                        Edge_map_const_iterator it2 = edge_map.find(fh);

                        if (it2 != edge_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.edge(fh).point();
                    } else { /* Check if we already met this vertex */
                        Vertex_map_const_iterator it2 = vertex_map.find(fh);

                        if (it2 != vertex_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.point(fh);
                    }

                    /*
                     * Add the vertex to the vertices and update the
                     * corresponding map
                     */
                    if (mvh == -1) {
                        const Tuple_3 tuple(CGAL::to_double(p.x()),
                                            CGAL::to_double(p.y()),
                                            CGAL::to_double(p.z()));
                        mvh = mesh.insert(tuple);

                        if (is_edge)
                            edge_map[fh] = mvh;
                        else
                            vertex_map[fh] = mvh;
                    }

                    mf.insert(mvh);
                }

                const Mesh_facet_handle mfh = mesh.insert(mf);

                /* Ensure the orientation of the facet is correct */

                /* Normal of the facet */
                const Tuple_3 n = mesh.compute_facet_normal(mfh);
                const Face_handle ch1 = f.superface(0);
                const Point p = _comp.cell(ch1).point();
                const Tuple_3 Mc = Tuple_3(CGAL::to_double(p.x()),
                                           CGAL::to_double(p.y()),
                                           CGAL::to_double(p.z()));
                const Tuple_3& Mf = mesh.vertex(mfh, 0);

                /* Inward pointing vector */
                const Tuple_3 v = Mc - Mf;

                if (label > 0) { /* Facet oriented towards first superface */
                    if (Tuple_3::dot(n, v) < 0)
                        mesh.flip_facet_normal(mfh);
                } else { /* Facet oriented towards second superface */
                    if (Tuple_3::dot(n, v) > 0)
                        mesh.flip_facet_normal(mfh);
                }
            }

            edge_map.clear();
            vertex_map.clear();
        }

        /*!
         *  Extracts a mesh from the complex.
         *  \param[out] mesh a mesh
         *  \param[in] labeler a facet labeler
         */
        template<class Labeler>
        void _extract(Mesh& mesh,
                      const Labeler& labeler,
                      bool keep_bounded_bbox_facet,
                      Facets_binary_labeler_tag tag) const
        {
            typedef std::map<Face_handle, Mesh_vertex_handle> Vertex_map;
            typedef std::map<Face_handle, Mesh_vertex_handle> Edge_map;

            typedef typename Vertex_map::const_iterator
                Vertex_map_const_iterator;
            typedef typename Edge_map::const_iterator
                Edge_map_const_iterator;

            mesh.clear();

            Vertex_map vertex_map;
            Edge_map edge_map;

            const bool has_bbox = _comp.has_bbox();

            const Faces_const_iterator first = _comp.facets_begin();
            const Faces_const_iterator  last = _comp.facets_end();

            /* Iterate over all the facets */
            for (Faces_const_iterator it = first; it != last; ++it) {
                const Face& f = *it;

                /*
                 * If the complex has a bounding box, skip the unbounded
                 * facets of the bounding box.
                 */
                const bool is_bbox_facet = _comp.is_bbox_facet(f);

                if (has_bbox && is_bbox_facet
                 && !keep_bounded_bbox_facet)
                       continue;

                const Face_handle fh = _comp.facet_handle(f);

                const bool selected = labeler(fh);

                /* Skip facets not selected */
                if (!selected)
                    continue;

                std::vector<Face_handle> hs;
                const bool is_bounded =
                    _comp.facet_to_polygon(f, std::back_inserter(hs));

                if (has_bbox && is_bbox_facet
                 && keep_bounded_bbox_facet && !is_bounded)
                    continue;

                const Plane_handle plh = _comp.facet_plane(f);

                /* Create a new facet of the final polygonal mesh */
                Mesh_facet mf(plh);
                mf.is_bounded() = is_bounded;

                const int num_hs = hs.size();

                for (int i = 0; i != num_hs; ++i) {
                    const Face_handle fh = hs[i];

                    Point p = CGAL::ORIGIN;

                    Mesh_vertex_handle mvh = -1;

                    /* Unbounded face and first or last handle: edge handle */
                    const bool is_edge = !is_bounded && (i == 0
                                                      || i == num_hs - 1);

                    if (is_edge) { /* Check if we already met this edge */
                        Edge_map_const_iterator it2 = edge_map.find(fh);

                        if (it2 != edge_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.edge(fh).point();
                    } else { /* Check if we already met this vertex */
                        Vertex_map_const_iterator it2 = vertex_map.find(fh);

                        if (it2 != vertex_map.end())
                            mvh = it2->second;
                        else
                            p = _comp.point(fh);
                    }

                    /*
                     * Add the vertex to the vertices and update the
                     * corresponding map
                     */
                    if (mvh == -1) {
                        const Tuple_3 tuple(CGAL::to_double(p.x()),
                                            CGAL::to_double(p.y()),
                                            CGAL::to_double(p.z()));
                        mvh = mesh.insert(tuple);

                        if (is_edge)
                            edge_map[fh] = mvh;
                        else
                            vertex_map[fh] = mvh;
                    }

                    mf.insert(mvh);
                }

                const Mesh_facet_handle mfh = mesh.insert(mf);

                /* Ensure the orientation of the facet is correct */

                /* Normal of the plane */
                const Plane pl = _comp.plane(plh);
                const Vector u = pl.orthogonal_vector();
                const Tuple_3 v = Tuple_3(CGAL::to_double(u.x()),
                                          CGAL::to_double(u.y()),
                                          CGAL::to_double(u.z()));

                /* Normal of the facet */
                const Tuple_3 n = mesh.compute_facet_normal(mfh);

                /* Facet oriented like the plane */
                if (Tuple_3::dot(n, v) < 0)
                    mesh.flip_facet_normal(mfh);
            }

            edge_map.clear();
            vertex_map.clear();
        }

    protected:
        const Comp& _comp;
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_MESH_EXTRACTOR_3_HPP */
