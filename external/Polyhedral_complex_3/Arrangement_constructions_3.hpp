#ifndef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_CONSTRUCTIONS_3_HPP
#define POLYHEDRAL_COMPLEX_3_ARRANGEMENT_CONSTRUCTIONS_3_HPP

#include<cassert>

#include <CGAL/intersections.h>

namespace Polyhedral_complex_3 {

template<class K>
class Arrangement_constructions_3 {
    public:
        typedef K Kernel;

        typedef typename Kernel::FT FT;

        typedef typename Kernel::Line_3 Line;
        typedef typename Kernel::Object_3 Object;
        typedef typename Kernel::Plane_3 Plane;
        typedef typename Kernel::Point_3 Point;

    public:
        /*! Computes the squared distance of a point to a plane. */
        static FT squared_distance(const Plane& pl, const Point& p) { return CGAL::squared_distance(p, pl); }

        /*! Computes the intersection point of a plane and a line. */
        static Point intersection(const Plane& pl, const Line& l)
        {
            const Object obj = CGAL::intersection(pl, l);
            const Point* p = CGAL::object_cast<Point>(&obj);

            assert(p != 0);

            return *p;
        }

        /*! Computes the intersection line of three planes. */
        static Line intersection(const Plane& pl1, const Plane& pl2)
        {
            const Object obj = CGAL::intersection(pl1, pl2);
            const Line* l = CGAL::object_cast<Line>(&obj);

            assert(l != 0);

            return *l;
        }

         /*! Computes the intersection point of three planes (that meet). */
        static Point intersection(const Plane& pl1, const Plane& pl2, const Plane& pl3) { return intersection(pl1, intersection(pl2, pl3)); }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_CONSTRUCTIONS_3_HPP */
