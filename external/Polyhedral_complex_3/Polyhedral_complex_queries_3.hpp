#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_QUERIES_3_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_QUERIES_3_HPP

#include<cassert>
#include<iterator>
#include<utility>

//#include <CGAL/spatial_sort.h>

namespace Polyhedral_complex_3 {

/*! Finds a bounded cell in the polyhedral complex. */
template<class Comp>
typename Comp::Face_handle find_bounded_cell(const Comp& comp)
{
    typedef typename Comp::Faces_const_iterator Faces_const_iterator;
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;

    for (Faces_const_iterator it = comp.cells_begin();
         it != comp.cells_end(); ++it) {
        const Face& c = *it;

        if (comp.is_cell_bounded(c))
            return comp.cell_handle(c);
    }

    return Face_handle();
}

/*! Finds an unbounded cell in the polyhedral complex. */
template<class Comp>
typename Comp::Face_handle find_unbounded_cell(const Comp& comp)
{
    typedef typename Comp::Faces_const_iterator Faces_const_iterator;
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;

    for (Faces_const_iterator it = comp.cells_begin();
         it != comp.cells_end(); ++it) {
        const Face& c = *it;

        if (!comp.is_cell_bounded(c))
            return comp.cell_handle(c);
    }

    assert(0); /* There is always unbounded cells in the polyhedral complex */

    return Face_handle();
}

/*! Checks whether a segment intersects the closure of a facet. */
template<class Comp>
bool do_intersect_facet_cl(const Comp& comp,
                           const typename Comp::Segment& s,
                           typename Comp::Face_handle fh)
{
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Plane_handle Plane_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Face Face;
    typedef typename Comp::Object Object;
    typedef typename Comp::Point Point;
    typedef typename Comp::Vector Vector;
    typedef typename Comp::Segment Segment;
    typedef typename Comp::Plane Plane;

    const Face& f = comp.facet(fh);
    const Point f_point = f.point();
    const Plane_handle plh = comp.facet_plane(fh);
    const Plane& pl = comp.plane(plh);

    const Object obj = CGAL::intersection(pl, s);
    const Point* p = CGAL::object_cast<Point>(&obj);

    if (p != 0) {
        for (Subfaces_const_iterator it = f.subfaces_begin();
             it != f.subfaces_end(); ++it) {
            const Face_handle eh = *it;
            const Face& e = comp.edge(eh);
            const Point e_point = e.point();
            const Point e_point1 = comp.point(e.subface(0));

			// TODO: replace construction by plane-only predicates

            const Vector u = e_point - e_point1;
            const Vector v = f_point - e_point1;

            /* The normal to the edge */
            const Vector n = (u * u) * v - (u * v) * u;
            const Vector w = *p - e_point1;

            if ((n * v) * (n * w) < 0)
                return false;
        }

        return true;
    }
    
    if (CGAL::object_cast<Segment>(&obj) == 0)
        return false;

    const Point s_point1 = s.source();
    const Point s_point2 = s.target();

    for (Subfaces_const_iterator it = f.subfaces_begin();
         it != f.subfaces_end(); ++it) {
        const Face_handle eh = *it;
        const Face& e = comp.edge(eh);
        const Point e_point = e.point();
        const Point e_point1 = comp.point(e.subface(0));

        const Vector u = e_point - e_point1;
        const Vector v = f_point - e_point1;

        /* The normal to the edge */
        const Vector n = (u * u) * v - (u * v) * u;
        const Vector w1 = s_point1 - e_point1;
        const Vector w2 = s_point2 - e_point1;

        if ((n * v) * (n * w1) < 0
         && (n * v) * (n * w2) < 0)
            return false;
    }

    return true;
}

/*! Finds the plane normal coordinate with maximum absolute value. */
template<class K>
int find_max_coord(const typename K::Plane_3& pl)
{
    typedef typename K::RT RT;

    const RT n[3] = {
        CGAL::abs(pl.a()),
        CGAL::abs(pl.b()),
        CGAL::abs(pl.c()),
    };

    int i = 0;

    if (n[1] > n[i])
        i = 1;

    if (n[2] > n[i])
        i = 2;

    return i;
}

/*! Checks whether a segment intersects the closure of a facet. */
template<class Comp>
bool do_intersect_facet_cl2(const Comp& comp,
                            const typename Comp::Segment& s,
                            typename Comp::Face_handle fh)
{
    typedef typename Comp::Kernel Kernel;
    typedef typename Kernel::Point_2 Point_2;
    typedef typename Kernel::Vector_2 Vector_2;

    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Plane_handle Plane_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Face Face;
    typedef typename Comp::Object Object;
    typedef typename Comp::Point Point;
    typedef typename Comp::Vector Vector;
    typedef typename Comp::Segment Segment;
    typedef typename Comp::Plane Plane;

    const Face& f = comp.facet(fh);
    const Point f_point = f.point();
    const Plane_handle plh = comp.facet_plane(fh);
    const Plane pl = comp.plane(plh);

    const Object obj = CGAL::intersection(pl, s);
    const Point* p = CGAL::object_cast<Point>(&obj);

    if (p != 0) {
        const int i1 = find_max_coord<Kernel>(pl);
        const int i2 = (i1 + 1) % 3;
        const int i3 = (i1 + 2) % 3;

        const Point_2 f2_point(f_point[i2],
                               f_point[i3]);
        const Point_2 p2_point((*p)[i2],
                               (*p)[i3]);

        for (Subfaces_const_iterator it = f.subfaces_begin();
             it != f.subfaces_end(); ++it) {
            const Face_handle eh = *it;
            const Face& e = comp.edge(eh);

            const Point e_point = e.point();
            const Point_2 e2_point(e_point[i2],
                                   e_point[i3]);

            const Point e_point1 = comp.point(e.subface(0));
            const Point_2 e2_point1(e_point1[i2],
                                    e_point1[i3]);

            /* The normal to the edge */
            const Vector_2 n(-(e2_point.y() - e2_point1.y()),
                               e2_point.x() - e2_point1.x() );
            const Vector_2 v(e2_point1, f2_point);
            const Vector_2 w(e2_point1, p2_point);

            if ((n * v) * (n * w) < 0)
                return false;
        }

        return true;
    }
    
    if (CGAL::object_cast<Segment>(&obj) == 0)
        return false;

    const int i1 = find_max_coord<Kernel>(pl);
    const int i2 = (i1 + 1) % 3;
    const int i3 = (i1 + 2) % 3;

    const Point s_point1 = s.source();
    const Point_2 s2_point1(s_point1[i2],
                            s_point1[i3]);

    const Point s_point2 = s.target();
    const Point_2 s2_point2(s_point2[i2],
                            s_point2[i3]);

    const Point_2 f2_point(f_point[i2],
                           f_point[i3]);
    const Point_2 p2_point((*p)[i2],
                           (*p)[i3]);

    for (Subfaces_const_iterator it = f.subfaces_begin();
         it != f.subfaces_end(); ++it) {
        const Face_handle eh = *it;
        const Face& e = comp.edge(eh);

        const Point e_point = e.point();
        const Point_2 e2_point(e_point[i2],
                               e_point[i3]);

        const Point e_point1 = comp.point(e.subface(0));
        const Point_2 e2_point1(e_point1[i2],
                                e_point1[i3]);

        /* The normal to the edge */
        const Vector_2 n(-(e2_point.y() - e2_point1.y()),
                           e2_point.x() - e2_point1.x() );
        const Vector_2 v(e2_point1, f2_point);
        const Vector_2 w1(e2_point1, s2_point1);
        const Vector_2 w2(e2_point1, s2_point2);

        if ((n * v) * (n * w1) < 0
         && (n * v) * (n * w2) < 0)
            return false;
    }

    return true;
}

/*!
 *  Checks whether a given point is inside a cell.
 *  \param[in] comp a polyhedral complex
 *  \param[in] q a point
 *  \param[in] c a cell
 *  \return a boolean indicating whether q lies inside c
 */
template<class Comp>
bool is_inside(const Comp& comp,
               const typename Comp::Point& q,
               const typename Comp::Face& c)
{
    typedef typename Comp::Point Point;
    typedef typename Comp::Plane Plane;
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Plane_handle Plane_handle;

    Point c_point = c.point();

    for (Subfaces_const_iterator it = c.subfaces_begin();
         it != c.subfaces_end(); ++it) {
        const Face_handle fh = *it;

        const Plane_handle plh = comp.facet_plane(fh);
        const Plane& pl = comp.plane(plh);

        const int q_side = pl.oriented_side(q);
        const int c_side = pl.oriented_side(c_point);

        assert(c_side != 0);

        if (q_side != c_side && q_side != 0)
            return false;
    }

    return true;
}

/*!
 *  Finds a cell containing the given point.
 *  \param[in] comp a polyhedral complex
 *  \param[in] q a point
 *  \param[in] ch0 an optional cell to start searching from
 *  \return a face handle to a cell containing the point
 */
template<class Comp>
typename Comp::Face_handle
find_containing_cell(const Comp& comp,
                     const typename Comp::Point& q,
                     typename Comp::Face_handle ch0 =
                         typename Comp::Face_handle())
{
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Plane_handle Plane_handle;
    typedef typename Comp::Point Point;
    typedef typename Comp::Segment Segment;
    typedef typename Comp::Plane Plane;

    Face_handle ch = (ch0 == Face_handle())
                   ? find_unbounded_cell(comp)
                   : ch0;
    const Face* c = &comp.cell(ch);
    Point c_point = c->point();

    bool inside = false;

    Face_handle out_fh = Face_handle();

    do {
        inside = true;

        for (Subfaces_const_iterator it = c->subfaces_begin();
             it != c->subfaces_end(); ++it) {
            const Face_handle fh = *it;

            /* We are on the other side of f so keep inside true */
            if (fh == out_fh)
                continue;

            const Plane_handle plh = comp.facet_plane(fh);
            const Plane& pl = comp.plane(plh);

            const int q_side = pl.oriented_side(q);
            const int c_side = pl.oriented_side(c_point);

            assert(c_side != 0);

            if (q_side != c_side && q_side != 0) {
                inside = false;

				if (do_intersect_facet_cl(comp, c_point, q, comp.facet(fh))) {
              //if (do_intersect_facet_cl(comp, s, fh)) {
              //if (do_intersect_facet_cl2(comp, s, fh)) {
                    out_fh = fh;

                    break;
                }
            }
        }

        /* Update the cell handle, the pointer and the ref. point */
        if (!inside) {
            assert(out_fh != Face_handle());

            const Face& f = comp.facet(out_fh);

            const Face_handle ch1 = f.superface(0);
            const Face_handle ch2 = f.superface(1);

            ch = (ch1 != ch) ? ch1 : ch2;

            c = &comp.cell(ch);
            c_point = c->point();
        }
    } while (!inside);

    return ch;
}

/*!
 *  Locates the given points in the polyhedral complex.
 *  \param[in] comp a polyhedral complex
 *  \param[in] first an iterator for the range of query points
 *  \param[in] last an iterator for the range of query points
 *  \param[out] it an output iterator for the query results
 *  \param[in] ch0 an optional cell to start searching from
 *  \note
 *    CGAL::spatial_sort() should be called before to improve space locality.
 */
template<class Comp,
         class InputIterator,
         class OutputIterator>
void find_containing_cells(const Comp& comp,
                           InputIterator first,
                           InputIterator last,
                           OutputIterator it,
                           typename Comp::Face_handle ch0 =
                               typename Comp::Face_handle())
{
    for (InputIterator it2 = first; it2 != last; ++it2)
        *it++ = (ch0 = find_containing_cell(comp, *it2, ch0));
}

/*! Finds the facet of the cell contained in the given plane. */
template<class Comp>
typename Comp::Face_handle
find_facet(const Comp& comp,
           const typename Comp::Face& c,
           typename Comp::Plane_handle plh)
{
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Face_handle Face_handle;

    for (Subfaces_const_iterator it = c.subfaces_begin();
         it != c.subfaces_end(); ++it) {
        const Face_handle fh = *it;

        if (comp.facet_plane(fh) == plh)
            return fh;
    }

    return Face_handle();
}

/*! Finds the facet index of the cell contained in the given plane. */
template<class Comp>
int find_facet_index(const Comp& comp,
                     const typename Comp::Face& c,
                     typename Comp::Plane_handle plh)
{
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Face_handle Face_handle;

    int index = 0;

    for (Subfaces_const_iterator it = c.subfaces_begin();
         it != c.subfaces_end(); ++it, ++index) {
        const Face_handle fh = *it;

        if (comp.facet_plane(fh) == plh)
            return index;
    }

    return -1;
}

/*!
 *  Finds the cell lying behind a plane according to the given vector.
 *  \param[in] comp a polyhedral complex
 *  \param[in] ch a handle to a cell in the polyhedral complex
 *  \param[in] plh a handle to a plane of the polyhedral complex
 *  \param[in] v a direction vector
 *  \param[out] fh_i a handle to the facet f of the cell contained in the plane
 *                   and the index of the adjacent cell in its superfaces list
 *  \return a handle to ch or to its adjacent cell (through f)
 */
template<class Comp>
typename Comp::Face_handle
find_cell_behind(const Comp& comp,
                 typename Comp::Face_handle ch,
                 typename Comp::Plane_handle plh,
                 const typename Comp::Vector& v,
                 std::pair<typename Comp::Face_handle, int>& fh_i)
{
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::FT FT;
    typedef typename Comp::Point Point;
    typedef typename Comp::Vector Vector;
    typedef typename Comp::Plane Plane;

    const Face& c = comp.cell(ch);
    const Point c_point = c.point();

    /* Find the facet */
    const Face_handle pl_fh = find_facet(comp, c, plh);

    if (pl_fh == Face_handle())
        return Face_handle();

  //assert(pl_fh != Face_handle());

    const Face& f = comp.facet(pl_fh);
    const Point f_point = f.point();

    const Vector u = c_point - f_point;

    const Plane& pl = comp.plane(plh);
    const Vector n = pl.orthogonal_vector();

    const Face_handle ch1 = f.superface(0);
    const Face_handle ch2 = f.superface(1);

    Face_handle behind_ch = ch;

    /* Return a handle to the other cell adjacent to f */
    if ((n * u) * (n * v) >= 0)
        behind_ch = (ch1 != ch) ? ch1 : ch2;

    fh_i = std::make_pair(pl_fh, (behind_ch == ch1) ? 0 : 1);

    return behind_ch;
}


// Test the intersection between a line (pq) and a facet of a polyhedral complex
template<class Comp>
bool do_intersect_facet_cl(const Comp& comp, const typename Comp::Point& p, const typename Comp::Point& q, const typename Comp::Face& f)
{
	typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Face Face;
    typedef typename Comp::Point Point;
    
	Point f_c = f.point();

	for (Subfaces_const_iterator sit = f.subfaces_begin() ; sit != f.subfaces_end() ; ++sit)
	{
		const Face& e = comp.edge(*sit);
		const Point p1 = comp.point(e.subface(0));
		const Point p2 = (e.number_of_subfaces() < 2) ? e.point() : comp.point(e.subface(1));
		
		if ( CGAL::orientation(p,p1,p2,q) * CGAL::orientation(p,p1,p2,f_c) < 0 ) return false;
	}

	return true;
}


/*!
 *  Finds all the facets intersected by the given segment and also the cells
 *  where the ending points are located.
 *  \param[in] comp a polyhedral complex
 *  \param[in] p the starting point
 *  \param[in] q the ending point
 *  \param[out] p_ch a handle to a cell containing p
 *  \param[out] it an iterator to a sequence of handles of intersected faces
 *  \param[out] q_ch a handle to a cell containing q
 *  \return a face handle to a cell containing the point
 */
template<class Comp,
class OutputIterator>
void segment_search(const Comp& comp,
                    const typename Comp::Point& p,
                    const typename Comp::Point& q,
                    typename Comp::Face_handle& p_ch,
                    OutputIterator it,
                    typename Comp::Face_handle& q_ch,
                    typename Comp::Face_handle p_ch0 =
                        typename Comp::Face_handle(),
                    bool p_c0_is_p_c = false,
                    bool stop = false)
{
    typedef typename Comp::Plane_handle Plane_handle;
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Point Point;
    typedef typename Comp::Segment Segment;
    typedef typename Comp::Plane Plane;

    p_ch = p_c0_is_p_c ? p_ch0 : find_containing_cell(comp, p, p_ch0);

    Face_handle ch = p_ch;
    const Face* c = &comp.cell(ch);
    Point c_point = c->point();

	bool inside = false;

    Face_handle out_fh = Face_handle();

    do {
        inside = true;

        for (Subfaces_const_iterator fit = c->subfaces_begin(); fit != c->subfaces_end(); ++fit)
		{
            const Face_handle fh = *fit;
			
			/* We are on the other side of f so keep inside true */
            if (fh == out_fh) continue;

            const Plane_handle plh = comp.facet_plane(fh);
            const Plane& pl = comp.plane(plh);

            const int q_side = pl.oriented_side(q);
            const int c_side = pl.oriented_side(c_point);

            assert(c_side != 0);

            if (q_side != c_side && q_side != 0) {
                inside = false;

                if (do_intersect_facet_cl(comp, p, q, comp.facet(fh)))
				{
                    out_fh = fh;
                    break;
                }
            }
        }

        /* Update the cell handle, the pointer and the ref. point */
        if (!inside) {
            assert(out_fh != Face_handle());

            const Face& f = comp.facet(out_fh);

            const Face_handle ch1 = f.superface(0);
            const Face_handle ch2 = f.superface(1);

            if (ch == ch1) {
                *it++ = std::make_pair(out_fh, 0);
                ch = ch2;
            } else { /* ch == ch2 */
                *it++ = std::make_pair(out_fh, 1);
                ch = ch1;
            }

            c = &comp.cell(ch);
            c_point = c->point();
        }
    } while (!inside && !stop);

    q_ch = ch;
}


/*!
 *  Finds all the facets intersected by the given segment and also the cells
 *  where the ending points are located.
 *  \param[in] comp a polyhedral complex
 *  \param[in] p the starting point
 *  \param[in] q the ending point
 *  \param[out] p_ch a handle to a cell containing p
 *  \param[out] it an iterator to a sequence of handles of intersected faces
 *  \param[out] itCell an iterator to a sequence
 *  of pairs <handles of intersected cell, length of segment in intersected cell>
 *  \param[out] q_ch a handle to a cell containing q
 *  \return a face handle to a cell containing the point
 */
template<class Comp,
class OutputIterator,
class OutputIteratorCell>
void segment_search_advanced(const Comp& comp,
                             const typename Comp::Point &p,
                             const typename Comp::Point &q,
                             typename Comp::Face_handle &p_ch,
                             OutputIterator it,
                             OutputIteratorCell itCell,
                             typename Comp::Face_handle &q_ch,
                             typename Comp::Face_handle p_ch0 =
                             typename Comp::Face_handle(),
                             bool p_c0_is_p_c = false,
                             bool stop = false)
{
    typedef typename Comp::Plane_handle Plane_handle;
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;
    typedef typename Comp::Point Point;
    typedef typename Comp::Segment Segment;
    typedef typename Comp::Plane Plane;

    p_ch = p_c0_is_p_c ? p_ch0 : find_containing_cell(comp, p, p_ch0);

    Face_handle ch = p_ch;
    const Face* c = &comp.cell(ch);
    Point c_point = c->point();

	bool inside = false;

    Face_handle out_fh = Face_handle();

    Point curPoint = p;

    do {
        inside = true;
        double dist = -1.;
        Point newPoint;

        for (Subfaces_const_iterator fit = c->subfaces_begin(); fit != c->subfaces_end(); ++fit)
		{
            const Face_handle fh = *fit;

			/* We are on the other side of f so keep inside true */
            if (fh == out_fh) continue;

            const Plane_handle plh = comp.facet_plane(fh);
            const Plane& pl = comp.plane(plh);

            const int q_side = pl.oriented_side(q);
            const int c_side = pl.oriented_side(c_point);

            assert(c_side != 0);

            if (q_side != c_side && q_side != 0) {
                inside = false;

                if (do_intersect_facet_cl(comp, p, q, comp.facet(fh)))
				{
                    auto facetPlane = comp.plane(comp.facet_plane(comp.facet(fh)));
                    auto intersection = CGAL::intersection(facetPlane, Segment(p, q));
                    assert(intersection);
                    if (const Point *s = boost::get<Point>(&*intersection)) {
                        dist = sqrt(CGAL::to_double((*s - curPoint).squared_length()));
                        newPoint = *s;
                    }

                    out_fh = fh;
                    break;
                }
            }
        }

        /* Update the cell handle, the pointer and the ref. point */
        if (!inside) {
            assert(out_fh != Face_handle());

            const Face& f = comp.facet(out_fh);

            const Face_handle ch1 = f.superface(0);
            const Face_handle ch2 = f.superface(1);

            *itCell++ = std::make_pair(ch, dist);

            if (ch == ch1) {
                *it++ = std::make_pair(out_fh, 0);
                ch = ch2;
            } else { /* ch == ch2 */
                *it++ = std::make_pair(out_fh, 1);
                ch = ch1;
            }
            curPoint = newPoint;
            c = &comp.cell(ch);
            c_point = c->point();
        }
    } while (!inside && !stop);

    *itCell++ = std::make_pair(ch, sqrt(CGAL::to_double((q - curPoint).squared_length())));

    q_ch = ch;
}


/*!
 *  Find all the adjacent facets to a facet whithin one cell
 *  \param[in] comp a polyhedral complex
 *  \param[in] fh the facet of the cell
 *  \param[in] ch the cell whose facets are inspected
 *  \param[out] it an iterator to a sequence of pair of handles of facets
 */
template<class Comp,
         class OutputIterator>
void find_adjacent_facets(const Comp& comp,
                          typename Comp::Face_handle fh,
                          typename Comp::Face_handle ch,
                          OutputIterator it)
{
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Plane_handle Plane_handle;
    typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;

    const Plane_handle plh = comp.facet_plane(fh);

    const Face& c = comp.cell(ch);

    for (Subfaces_const_iterator it2 = c.subfaces_begin();
         it2 != c.subfaces_end(); ++it2) {
        const Face_handle f1h = *it2;

        /* Skip the given facet of the cell */
        if (f1h == fh)
            continue;

        const Face& f1 = comp.facet(f1h);

        /* Only two superfaces per facet */
        const Face_handle ch1 = f1.superface(0);
        const Face_handle ch2 = f1.superface(1);

        const Face_handle neighbor_ch = (ch1 != ch) ? ch1 : ch2;
        const Face& neighbor_c = comp.cell(neighbor_ch);

        for (Subfaces_const_iterator it3 = neighbor_c.subfaces_begin();
             it3 != neighbor_c.subfaces_end(); ++it3) {
            const Face_handle f2h = *it3;

            if (comp.facet_plane(f2h) == plh) {
                *it++ = std::make_pair(f1h, f2h);
                break;
            }
        }
    }
}

/*!
 *  Finds all the facets around an edge (in order).
 *  \param eh a handle to an edge
 *  \param it an iterator to output the sequence of facets
 */
template<class Comp,
         class OutputIterator>
void find_facets_around_edge(const Comp& comp,
                             typename Comp::Face_handle eh,
                             OutputIterator it)
{
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Superfaces_const_iterator Superfaces_const_iterator;

    const Face& e = comp.edge(eh);
    int number_of_facets = e.number_of_superfaces();

    /* The first facet is chosen arbitrarily */
    Face_handle fh = e.superface(0);
    const Face& f = comp.facet(fh);
    Face_handle ch = f.superface(0);

    *it++ = fh;
    --number_of_facets;

    while (number_of_facets > 0) {
        const Face_handle prev_fh = fh;
        const Face_handle prev_ch = ch;

        /* Find the other facet adjacent to the cell */
        for (Superfaces_const_iterator it2 = e.superfaces_begin();
             it2 != e.superfaces_end(); ++it2) {
            const Face_handle next_fh = *it2;

            if (next_fh == prev_fh)
                continue;

            const Face& next_f = comp.facet(next_fh);
            const Face_handle next_ch1 = next_f.superface(0);
            const Face_handle next_ch2 = next_f.superface(1);

            if (next_ch1 == prev_ch) {
                fh = next_fh;
                ch = next_ch2;
            } else if (next_ch2 == prev_ch) {
                fh = next_fh;
                ch = next_ch1;
            } else
                continue;
        }

        assert(fh != prev_fh);

        *it++ = fh;
        --number_of_facets;
    }
}

/*!
 *  Finds all the facets around an edge (in order) starting from (and
 *  excluding) a given facet.
 *  \param eh a handle to an edge
 *  \param fh a handle to a facet
 *  \param it an iterator to output the sequence of facets
 *  \return a boolean indicating whether the edge is adjacent to the facet
 */
template<class Comp,
         class OutputIterator>
bool find_facets_around_edge(const Comp& comp,
                             typename Comp::Face_handle eh,
                             typename Comp::Face_handle fh,
                             OutputIterator it)
{
    typedef typename Comp::Face Face;
    typedef typename Comp::Face_handle Face_handle;
    typedef typename Comp::Superfaces_const_iterator Superfaces_const_iterator;

    const Face& e = comp.edge(eh);

    if (!e.has_superface(fh))
        return false;

    int number_of_facets = e.number_of_superfaces();

    const Face& f = comp.facet(fh);
    Face_handle ch = f.superface(0);

    /* Skip the selected facet */
    --number_of_facets;

    while (number_of_facets > 0) {
        const Face_handle prev_fh = fh;
        const Face_handle prev_ch = ch;

        /* Find the other facet adjacent to the cell */
        for (Superfaces_const_iterator it2 = e.superfaces_begin();
             it2 != e.superfaces_end(); ++it2) {
            const Face_handle next_fh = *it2;

            if (next_fh == prev_fh)
                continue;

            const Face& next_f = comp.facet(next_fh);
            const Face_handle next_ch1 = next_f.superface(0);
            const Face_handle next_ch2 = next_f.superface(1);

            if (next_ch1 == prev_ch) {
                fh = next_fh;
                ch = next_ch2;
            } else if (next_ch2 == prev_ch) {
                fh = next_fh;
                ch = next_ch1;
            } else
                continue;
        }

        assert(fh != prev_fh);

        *it++ = fh;
        --number_of_facets;
    }

    return true;
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_QUERIES_3_HPP */
