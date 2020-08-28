#ifndef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_PREDICATES_3_HPP
#define POLYHEDRAL_COMPLEX_3_ARRANGEMENT_PREDICATES_3_HPP

#include<cassert>

#include <CGAL/intersections.h>

namespace Polyhedral_complex_3 {

template<class K>
class Arrangement_predicates_3 {
    public:
        typedef K Kernel;

        typedef typename Kernel::Line_3 Line;
        typedef typename Kernel::Object_3 Object;
        typedef typename Kernel::Plane_3 Plane;
        typedef typename Kernel::Point_3 Point;
        typedef typename Kernel::Ray_3 Ray;
        typedef typename Kernel::Segment_3 Segment;

    public:
        /*! On which side of the plane lies the point? */
        static int oriented_side(const Plane& pl, const Point& p) { return pl.oriented_side(p); }

        /*! Does the plane intersect the closure of the ray? */
        static bool do_intersect_cl(const Plane& pl, const Ray& r) { return CGAL::do_intersect(pl, r); }

        /*! Does the plane intersect the closure of the segment? */
        static bool do_intersect_cl(const Plane& pl, const Segment& s) { return CGAL::do_intersect(pl, s); }

        /*! Does the plane intersect the ray? */
        static bool do_intersect(const Plane& pl, const Ray& r)
        {
            const Object obj = CGAL::intersection(pl, r);
            const Point* p = CGAL::object_cast<Point>(&obj);

            if (p != 0) {
                return (*p != r.source());
            } else if (CGAL::object_cast<Ray>(&obj) != 0) {
                return true; /* r \subseteq pl */
            } else
                return false;
        }

        /*! Does the plane intersect the segment? */
        static bool do_intersect(const Plane& pl, const Segment& s)
        {
            const Object obj = CGAL::intersection(pl, s);
            const Point* p = CGAL::object_cast<Point>(&obj);

            if (p != 0) {
                return (*p != s.source() && *p != s.target());
            } else if (CGAL::object_cast<Segment>(&obj) != 0) {
                return true; /* s \subseteq pl */
            } else
                return false;
        }

        /*! Does the plane intersect the line? */
        static bool do_intersect(const Plane& pl, const Line& l) { return CGAL::do_intersect(pl, l); }

        /*! Does the plane contain the point? */
        static bool do_contain(const Plane& pl, const Point& p) { return (oriented_side(pl, p) == CGAL::ON_ORIENTED_BOUNDARY); }

        /*! Does the plane contain the ray? */
        static bool do_contain(const Plane& pl, const Ray& r)
        {
            const Object obj = CGAL::intersection(pl, r);
            const Ray* r2 = CGAL::object_cast<Ray>(&obj);

            return (r2 != 0);
        }

        /*! Does the plane contain the segment? */
        static bool do_contain(const Plane& pl, const Segment& s)
        {
            const Object obj = CGAL::intersection(pl, s);
            const Segment* s2 = CGAL::object_cast<Segment>(&obj);

            return (s2 != 0);
        }

        /*! Is the line parallel to the plane? */
        static bool is_parallel(const Plane& pl, const Line& l)
        {
            const Object obj = CGAL::intersection(pl, l);
            const Point* p = CGAL::object_cast<Point>(&obj);

            return (p == 0);
        }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_PREDICATES_3_HPP */
