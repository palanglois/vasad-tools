#ifndef POLYHEDRAL_COMPLEX_3_BSP_COMPLEX_3_HPP
#define POLYHEDRAL_COMPLEX_3_BSP_COMPLEX_3_HPP

#include<algorithm>
#include<queue>
#include<vector>

#include <Polyhedral_complex_3/Arrangement_3.hpp>
#include <Polyhedral_complex_3/BSP_tree_3.hpp>
#include <Polyhedral_complex_3/Polyhedral_complex_queries_3.hpp>

namespace Polyhedral_complex_3 {

template<class K = CGAL::Exact_predicates_exact_constructions_kernel>
class BSP_complex_3 : public Arrangement_3<K> {
    public:
        typedef K Kernel;
        typedef typename Kernel::Point_3 Point;
        typedef typename Kernel::Plane_3 Plane;
        typedef typename Kernel::Vector_3 Vector;

        typedef Arrangement_3<K> Arr;

        /* Faces */
        typedef typename Arr::Face Face;
        typedef typename Arr::Faces_iterator Faces_iterator;
        typedef typename Arr::Faces_const_iterator Faces_const_iterator;
        typedef typename Arr::Face_handle Face_handle;

        /* Planes */
        typedef typename Arr::Plane_handle Plane_handle;
        typedef typename Arr::Planes_const_iterator Planes_const_iterator;

        /* {Super,sub}faces iterators */
        typedef typename Arr::Superfaces_iterator Superfaces_iterator;
        typedef typename Arr::Superfaces_const_iterator
            Superfaces_const_iterator;

        typedef typename Arr::Subfaces_iterator Subfaces_iterator;
        typedef typename Arr::Subfaces_const_iterator
            Subfaces_const_iterator;

        typedef BSP_tree_3<K> Tree;
        typedef typename Tree::Node Node;
        typedef typename Tree::Node_handle Node_handle;
        typedef typename Tree::Partitioner Partitioner;
        typedef typename Tree::Partitioner_handle Partitioner_handle;

    public:
        BSP_complex_3(const Point& p,
                      const Vector& u,
                      const Vector& v,
                      const Vector& w) :
            Arr() {this->set_bbox(p, u, v, w);}

        BSP_complex_3(const CGAL::Bbox_3& bbox) :
            Arr() {
        	this->set_bbox(bbox);
        }

        BSP_complex_3(double xmin, double ymin, double zmin,
                      double xmax, double ymax, double zmax) :
            Arr(xmin, ymin, zmin,
                xmax, ymax, zmax) { }

        /*! Builds the BSP complex from the given tree (inorder). */
        bool build(const Tree& tree)
        {
            typedef std::pair<Face_handle, Node_handle> Pair;

            assert(this->has_bbox());
            assert(this->number_of_planes() == this->number_of_bbox_planes());

            const Face_handle ch =
                Polyhedral_complex_3::find_bounded_cell(*this);
            const Node_handle nh = tree.root_handle();

            std::queue<Pair> Q;
            Q.push(Pair(ch, nh));

            while (!Q.empty()) {
                const Pair pair = Q.front();
                Q.pop();

                const Face_handle ch = pair.first;
                const Node_handle nh = pair.second;

                const Node& n = tree.node(nh);

                if (n.is_leaf())
                    continue;

                const Partitioner_handle ph = n.partitioner_handle();
                const Plane& pl = tree.partitioner(ph);
                const Node_handle negative_nh = n.negative_node_handle();
                const Node_handle positive_nh = n.positive_node_handle();

                Plane_handle plh;
                Face_handle negative_ch;
                Face_handle positive_ch;

                const bool b = this->partition(ch, pl, plh,
                                               negative_ch,
                                               positive_ch);

                if (!b)
                    return false;

                Q.push(Pair(negative_ch, negative_nh));
                Q.push(Pair(positive_ch, positive_nh));
            }

            return true;
        }

        int number_of_partitions() const { return (this->number_of_planes() - this->number_of_bbox_planes()); }

        bool partition(const Face& c, const Plane& pl) { return partition(this->cell_handle(c), pl); }

        /*!
         *  Partitions a cell with the given plane.
         *  \param[in] ch handle to the cell to be partitioned
         *  \param[in] pl the partitioning plane
         *  \param[out] plh the assigned plane handle
         *  \param[out] negative_ch the assigned negative cell handle
         *  \param[out] positive_ch the assigned positive cell handle
         *  \return true if the cell was successfully split
         */
        bool partition(Face_handle ch, const Plane& pl,
                       Plane_handle& plh,
                       Face_handle& negative_ch,
                       Face_handle& positive_ch)
        {
            /* Ensure the cell is bounded */
            if (!this->is_cell_bounded(ch)) {
//#ifndef NDEBUG
                std::cout << "BSP_complex_3: cell is not bounded" << std::endl;
//#endif /* NDEBUG */
                return false;
            }

            /* Check whether this plane actually intersects the cell */
          //if (!_has_cell_vertices_on_both_sides(pl, ch)) {
//#ifndef NDEBUG
          //    std::cout << "BSP_complex_3: plane does not intersect the cell"
          //              << std::endl;
//#endif /* NDEBUG */
          //    return false;
          //}

            this->_split_cell_begin(ch);
            const bool b = this->insert(pl);
            this->_split_cell_end();

            if (b) {
                plh = this->number_of_planes() - 1;
                negative_ch = ch;
                positive_ch = this->number_of_cells() - 1;
            } else {
//#ifndef NDEBUG
          //    std::cout << "BSP_complex_3: plane insertion failed"
          //              << std::endl;
          //    return false;
//#endif /* NDEBUG */
            }

          //return b;
            return true;
        }

    protected:
        /*!
         *  Inserts a new plane in the list unless it is already in it (or
         *  it is outside the bounding box).
         *  \return true if the plane was actually inserted
         */
        virtual bool _insert_plane(const Plane& pl)
        {
            if (this->has_bbox()
             && !this->_has_bbox_vertices_on_both_sides(pl))
                return false;

            this->_planes.push_back(pl);

            return true;
        }

        /*! Checks whether the plane intersects the (bounded) cell */
        bool _has_cell_vertices_on_both_sides(const Plane& pl, Face_handle ch)
        {
            const Face& c = this->cell(ch);

            /* FIXME: dimension-dependent code */
            /* Get all the cell vertices */
            std::vector<Face_handle> vhs;

            for (Subfaces_const_iterator it = c.subfaces_begin();
                 it != c.subfaces_end(); ++it) {
                const Face_handle fh = *it;
                const Face& f = this->facet(fh);

                for (Subfaces_const_iterator it2 = f.subfaces_begin();
                     it2 != f.subfaces_end(); ++it2) {
                    const Face_handle eh = *it2;
                    const Face& e = this->edge(eh);

                    vhs.insert(vhs.end(), e.subfaces_begin(),
                                          e.subfaces_end());
                }
            }

            std::sort(vhs.begin(), vhs.end());
            typename std::vector<Face_handle>::const_iterator new_end =
                std::unique(vhs.begin(), vhs.end());

            /* Check that we have at least a vertex on both sides */
            bool has_cell_vertex_on_positive_side = false;
            bool has_cell_vertex_on_negative_side = false;

            for (typename std::vector<Face_handle>::const_iterator it =
                    vhs.begin();
                 it != new_end; ++it) {
                const Point p = this->point(*it);

                const int p_side = pl.oriented_side(p);

                if (p_side > 0) {
                    has_cell_vertex_on_positive_side = true;

                    if (has_cell_vertex_on_negative_side)
                        return true;
                } else if (p_side < 0) {
                    has_cell_vertex_on_negative_side = true;

                    if (has_cell_vertex_on_positive_side)
                        return true;
                }
            }

            return false;
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_BSP_COMPLEX_3_HPP */
