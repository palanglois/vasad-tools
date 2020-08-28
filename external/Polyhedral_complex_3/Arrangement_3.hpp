#ifndef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_HPP
#define POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_HPP

#include<algorithm>
#include<cassert>
#include<iostream>
#include<queue>
#include<vector>
#include<set>

#include <CGAL/basic.h>
#include <CGAL/Bbox_3.h>

/* Default 3D Kernel */
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

/* dD Kernel (needed for CGAL::linear_rank()) */
#include <CGAL/Homogeneous_d.h>
#include <CGAL/predicates_d.h>

/* 3D Kernel */
#include <CGAL/squared_distance_3.h>

#include <Polyhedral_complex_3/Arrangement_predicates_3.hpp>
#include <Polyhedral_complex_3/Arrangement_constructions_3.hpp>
#include <Polyhedral_complex_3/Arrangement_face_base_3.hpp>

//#define POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE

namespace Polyhedral_complex_3 {



template<class K = CGAL::Exact_predicates_exact_constructions_kernel,
		class K_d = CGAL::Homogeneous_d<typename K::FT> >
class Arrangement_3 {
public:
	typedef K Kernel;

	typedef typename Kernel::FT FT;
	typedef typename Kernel::RT RT;

	typedef typename Kernel::Object_3 Object;
	typedef typename Kernel::Point_3 Point;
	typedef typename Kernel::Vector_3 Vector;
	typedef typename Kernel::Line_3 Line;
	typedef typename Kernel::Ray_3 Ray;
	typedef typename Kernel::Segment_3 Segment;
	typedef typename Kernel::Plane_3 Plane;
	typedef typename Kernel::Iso_cuboid_3 Iso_cuboid;

	struct Plane_handle;

public:
	struct Face_handle {
	public:



		Face_handle() : _i(-1) { }

		Face_handle(int i) : _i(i) { }

		bool operator==(Face_handle fh) const { return (_i == fh._i); }

		operator int() const { return _i; }


	protected:
		int _i;
	};




public:
	typedef Arrangement_face_base_3<K,
			Face_handle> Face;

protected:
	typedef typename Kernel::Aff_transformation_3 Aff_transformation;

	typedef Arrangement_constructions_3<K> Constructions;
	typedef Arrangement_predicates_3<K> Predicates;

	/* FIXME: dimension-dependent code */
	enum { _dimension = 3 };
	enum { _number_of_face_lists = _dimension + 3 };
	enum { _number_of_bbox_planes = 2 * _dimension };
	/* FIXME: dimension-dependent code */
	enum { _number_of_bbox_vertices = 8 };

	typedef std::vector<Plane> Plane_list;
	typedef std::vector<Face> Face_list;

public:
	typedef typename Plane_list::const_iterator Planes_const_iterator;

	typedef typename Face_list::iterator Faces_iterator;
	typedef typename Face_list::const_iterator Faces_const_iterator;

	//typedef typename Face::Face_handle Face_handle;

	typedef typename Face::Superfaces_iterator Superfaces_iterator;
	typedef typename Face::Superfaces_const_iterator
			Superfaces_const_iterator;

	typedef typename Face::Subfaces_iterator Subfaces_iterator;
	typedef typename Face::Subfaces_const_iterator
			Subfaces_const_iterator;

public:
	struct Plane_handle {
	public:
		Plane_handle() : _i(-1) { }

		Plane_handle(int i) : _i(i) { }

		bool operator==(Plane_handle plh) const { return (_i == plh._i); }

		operator int() const { return _i; }




	protected:
		int _i;
	};

protected:
	typedef std::vector<Face_handle> Face_handle_list;

	typedef typename Face::Superface_list Superface_list;
	typedef typename Face::Subface_list Subface_list;

	typedef std::vector<Plane_handle> Plane_handle_list;

protected:
	Face_list _face_lists[_number_of_face_lists];
	Plane_handle_list _plane_handles;
	Plane_list _planes;

	bool _has_bbox;

	std::set<Face_handle> _split_ch; /* Restrict the construction to these cells */

	///MODIF
public:

	int plane_number_before_ghosts;
	std::vector<bool> consistent_orientation;
	std::vector<double> ref_a;
	std::vector<double> ref_b;
	std::vector<double> ref_c;
	std::vector<double> ref_d;

	bool save(const std::string& filename){
		std::cout << "Saving arrangement to: " << filename << std::endl;
		std::ofstream os(filename.c_str());
		if(! os.is_open()){return false;}
		std::cout << "| faces " << _number_of_face_lists << std::endl;
		for(int i=0; i<_number_of_face_lists; i++){
			std::cout << _face_lists[i].size() << std::endl;
			os << _face_lists[i].size() << std::endl;
			for(int j=0; j<_face_lists[i].size(); j++){
				os << _face_lists[i][j];
			}
		}
		std::cout << "| plane_handles " <<_plane_handles.size()<< std::endl;
		os << _plane_handles.size() << std::endl;
		for(int i=0; i<_plane_handles.size(); i++){
			os << (int) _plane_handles[i] << std::endl;
		}
		std::cout << "| planes " <<_planes.size()<< std::endl;
		os << _planes.size() << std::endl;
		for(int i=0; i<_planes.size(); i++){
			os << CGAL::to_double(_planes[i].a()) << ' ';
			os << CGAL::to_double(_planes[i].b()) << ' ';
			os << CGAL::to_double(_planes[i].c()) << ' ';
			os << CGAL::to_double(_planes[i].d()) << std::endl;
		}
		os << consistent_orientation.size() << std::endl;
		for(int i=0; i<consistent_orientation.size(); i++){
			os << consistent_orientation[i] << std::endl;
		}

		os << ref_a.size() << std::endl;
		for(int i=0; i<ref_a.size(); i++){
			os << ref_a[i] << " " << ref_b[i] << " " << ref_c[i] <<" "<< ref_d[i] << std::endl;
		}

		std::cout << "| split "<< _split_ch.size()<< std::endl;
		os << _split_ch.size() << std::endl;
		for(typename std::set<Face_handle>::iterator itset = _split_ch.begin(); itset != _split_ch.end(); itset++){
			os << int(*itset) << std::endl;
		}

		std::cout << "| bbox "<< _has_bbox << std::endl;
		os << _has_bbox << std::endl;
		os << plane_number_before_ghosts << std::endl;
		return true;
	}

	bool load(const std::string& filename){
		std::cout << "Loading arrangement from: " << filename << std::endl;
		std::ifstream is(filename.c_str());
		if(! is.is_open()){return false;}


		std::cout << "| faces" << std::endl;
		for(int i=0; i<_number_of_face_lists; i++){
			int temp_nbr_faces;
			is >> temp_nbr_faces;

			std::cout << i << " " << temp_nbr_faces << std::endl;

			_face_lists[i].clear();
			for(int j=0; j<temp_nbr_faces; j++){
				Face f;
				is >> f;
				_face_lists[i].push_back(f);
			}
		}

		std::cout << "| plane handles" << std::endl;
		int temp_i;
		is >> temp_i;
		_plane_handles.clear();
		for(int i=0; i< temp_i; i++){
			int temp;
			is >> temp;
			_plane_handles.push_back( Plane_handle(temp) );
		}

		std::cout << "| planes" << std::endl;
		is >> temp_i;
		_planes.clear();
		for(int i=0; i<temp_i; i++){
			double ta, tb, tc,td;
			is >> ta >> tb >> tc >> td;
			_planes.push_back(Plane(ta,tb,tc,td));
		}
		is >> temp_i;
		consistent_orientation.clear();
		for(int i=0; i<temp_i; i++){
			bool b_temp;
			is >> b_temp;
			consistent_orientation.push_back(b_temp);
		}
		is >> temp_i;
		ref_a.clear();
		ref_b.clear();
		ref_c.clear();
		ref_d.clear();
		for(int i=0; i<temp_i; i++){
			double pa,pb,pc,pd;
			is >> pa >> pb >> pc >> pd;
			ref_a.push_back(pa);
			ref_b.push_back(pb);
			ref_c.push_back(pc);
			ref_d.push_back(pd);
		}

		std::cout << "| split" << std::endl;
		is >> temp_i;
		_split_ch.clear();
		for(int i=0; i<temp_i; i++){
			int temp;
			is >> temp;
			_split_ch.insert(temp);
		}

		is >> _has_bbox;
		std::cout <<"| bbox " <<  _has_bbox << std::endl;
		is >> plane_number_before_ghosts;
		return true;
	}

	///END MODIF


public:
	/*!
	 *  Set the bounding box of the arrangement:
	 *  - the first 6 planes are always those of the bounding box,
	 *  - the first 8 vertices are always those of the bounding box,
	 *  - the only unbounded faces are those outside the bounding box,
	 *  - planes not intersecting the bounding box are not stored.
	 *  Must be called before inserting any other plane
	 */
	void set_bbox(const Point& p, const Vector& u, const Vector& v, const Vector& w)
	{
		assert( number_of_planes() == 0 );
		_insert_bbox_planes(p, u, v, w);
		_has_bbox = true;
	}

	void set_bbox(const CGAL::Bbox_3& bbox)
	{
		assert( number_of_planes() == 0 );
		_insert_bbox_planes(bbox);
		_has_bbox = true;
	}

	void set_bbox(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
	{
		assert( number_of_planes() == 0 );
		_insert_bbox_planes(xmin, ymin, zmin, xmax, ymax, zmax);
		_has_bbox = true;
	}

	/*! Creates an empty arrangement. */
	Arrangement_3() : _has_bbox(false) {
		plane_number_before_ghosts=0;
	}

	/*!
	 *  Inserts the given hyplerplane in the arrangement.
	 *  \return false if the plane was already in the arrangement
	 */
	bool insert(const Plane& pl)
	{
		if (_insert_plane(pl)) {
			_update_arrangement(1);

			return true;
		}

		return false;
	}

	/*!
	 *  Inserts the given hyplerplanes in the arrangement.
	 *  \return the number of actually inserted planes
	 */
	template<class InputIterator>
	int insert(InputIterator first, InputIterator last)
	{
		const int m = _insert_planes(first, last);

		if (m != 0)
			_update_arrangement(m);

		return m;
	}

	/*! Returns the (hardcoded) dimension of the arrangement. */
	static int dimension() { return _dimension; }

	/*! Returns the (hardcoded) # of bounding-box planes. */
	static int number_of_bbox_planes() { return _number_of_bbox_planes; }

	/*! Returns the (hardcoded) # of bounding-box vertices. */
	static int number_of_bbox_vertices() { return _number_of_bbox_vertices; }

	/* FIXME: does not take into account the bounding box */
	/*! Clears the arrangement. */
	void clear()
	{
		_clear_face_lists();
		_clear_plane_handles();
		_clear_planes();
	}

	/*! Computes the # of k-faces in a simple arrangement of n planes. */
	static int maximum_number_of_faces(int k, int n)
	{
		const int d = dimension();

		assert(k <= d);

		int f = 0;

		for (int i = 0; i <= k; ++i)
			f += _binomial(d - i, k - i) * _binomial(n, d - i);

		return f;
	}

	bool has_bbox() const { return _has_bbox; }


	void insert_splittable_cell(Face_handle ch) { _split_ch.insert(ch); }
	void clear_splittable_cell() { _split_ch.clear(); }

protected:
	void _split_cell_begin(Face_handle ch) { clear_splittable_cell(); insert_splittable_cell(ch); }

	void _split_cell_end() { clear_splittable_cell(); }

	bool _is_face_splittable(int k, Face_handle fh) const
	{
		assert(k >= 0);
		assert(k <= 3);

		if (k == 3)
			return (_split_ch.find(fh) != _split_ch.end());

		const Face& f = face(k, fh);

		for (Superfaces_const_iterator it = f.superfaces_begin();
				it != f.superfaces_end(); ++it) {
			const Face_handle gh = *it;

			if (_is_face_splittable(k + 1, gh))
				return true;
		}

		return false;
	}

	void _clear_face_lists()
	{
		for (int i = 0; i != _number_of_face_lists; ++i)
			_face_lists[i].clear();
	}

	void _clear_plane_handles() { _plane_handles.clear(); }

	void _clear_planes() { _planes.clear(); }

	/*! Computes the binomial coefficient "n choose k". */
	static int _binomial(int n, int k)
	{
		if (k == 0 || k == n)
			return 1;

		if (k > n)
			return 0;

		if (n - k > k)
			k = n - k;

		int b = 1;
		int m = 0;

		while (k < n) {
			++k;
			++m;
			b = (b * k) / m;
		}

		return b;
	}

protected:
	enum Color {
		WHITE = 0, /* cl f \cap h  =  \emptyset */
		PINK = 1, /* cl f \cap h \ne \emptyset
					  and f \cap h  =  \emptyset */
		RED = 2, /*    f \cap h \ne \emptyset
							   and f  not contained in h */
		CRIMSON = 3, /*    f \subseteq h */
		/**************************************/
		GREEN = 4, /* temporary */
		/**************************************/
		GREY = 5, /* cl f \cap h \ne \emptyset
										 and f \cap h  =  \emptyset */
		BLACK = 6, /*    f \subseteq h */
	};

protected:
	static const char* _color_to_str(Color color)
	{
		static const char *color_strs[] = {
				"white",
				"pink",
				"red",
				"crimson",
				"green",
				"grey",
				"black",
		};

		return color_strs[color];
	}

public:
	/* k-faces ****************************/

	Faces_iterator faces_begin(int k)
	{
		assert(k >= -1 && k <= dimension() + 1);

		return _face_list(k).begin();
	}

	Faces_const_iterator faces_begin(int k) const
	{
		assert(k >= -1 && k <= dimension() + 1);

		return _face_list(k).begin();
	}

	Faces_iterator faces_end(int k)
	{
		assert(k >= -1 && k <= dimension() + 1);

		return _face_list(k).end();
	}

	Faces_const_iterator faces_end(int k) const
	{
		assert(k >= -1 && k <= dimension() + 1);

		return _face_list(k).end();
	}

	int number_of_faces(int k) const
	{
		assert(k >= -1 && k <= dimension() + 1);

		return int(_face_list(k).size());
	}

	const Face& face(int k, int i) const
	{
		assert(k >= -1 && k <= dimension() + 1);

		return _face(k, i);
	}

	Face& face(int k, int i)
	{
		assert(k >= -1 && k <= dimension() + 1);

		return _face(k, i);
	}

	Face_handle face_handle(int k, const Face& f) const
	{
		return Face_handle(&f - &_face_lists[k + 1][0]);
	}

	/* d-faces: cells *********************/

	Faces_const_iterator cells_begin() const { return faces_begin(dimension()); }

	Faces_iterator cells_begin() { return faces_begin(dimension()); }

	Faces_const_iterator cells_end() const { return faces_end(dimension()); }

	Faces_iterator cells_end() { return faces_end(dimension()); }

	int number_of_cells() const { return number_of_faces(dimension()); }

	const Face& cell(int i) const { return face(dimension(), i); }

	Face& cell(int i) { return face(dimension(), i); }

	Face_handle cell_handle(const Face& c) const { return face_handle(dimension(), c); }

	Face_handle cell_neighbor(const Face& c, int i) const
	{
		const Face_handle fh = c.subface(i);
		const Face& f = facet(fh);

		const Face_handle ch = cell_handle(c);
		const Face_handle ch1 = f.superface(0);
		const Face_handle ch2 = f.superface(1);

		return (ch1 != ch ? ch1 : ch2);
	}

	Face_handle cell_neighbor(Face_handle ch, int i) const { return cell_neighbor(cell(ch), i); }

	/* 2-faces: facets ********************/

	Faces_const_iterator facets_begin() const { return faces_begin(2); }

	Faces_iterator facets_begin() { return faces_begin(2); }

	Faces_const_iterator facets_end() const { return faces_end(2); }

	Faces_iterator facets_end() { return faces_end(2); }

	int number_of_facets() const { return number_of_faces(2); }

	const Face& facet(int i) const { return face(2, i); }

	Face& facet(int i) { return face(2, i); }

	Face_handle facet_handle(const Face& f) const { return face_handle(2, f); }

	Plane_handle facet_plane(int i) const { return _plane_handles[i]; }

	Plane_handle facet_plane(const Face& f) const { return facet_plane(facet_handle(f)); }

	/* 1-faces: edges *********************/

	Faces_const_iterator edges_begin() const { return faces_begin(1); }

	Faces_iterator edges_begin() { return faces_begin(1); }

	Faces_const_iterator edges_end() const { return faces_end(1); }

	Faces_iterator edges_end() { return faces_end(1); }

	int number_of_edges() const { return number_of_faces(1); }

	const Face& edge(int i) const { return face(1, i); }

	Face& edge(int i) { return face(1, i); }

	Face_handle edge_handle(const Face& e) const { return face_handle(1, e); }

	/* 0-faces: vertices ******************/

	Faces_const_iterator vertices_begin() const { return faces_begin(0); }

	Faces_iterator vertices_begin() { return faces_begin(0); }

	Faces_const_iterator vertices_end() const { return faces_end(0); }

	Faces_iterator vertices_end() { return faces_end(0); }

	int number_of_vertices() const { return number_of_faces(0); }

	const Face& vertex(int i) const { return face(0, i); }

	Face& vertex(int i) { return face(0, i); }

	Face_handle vertex_handle(const Face& v) const { return face_handle(0, v); }

	/* Planes *****************************/

	Planes_const_iterator planes_begin() const { return _planes.begin(); }

	Planes_const_iterator planes_end() const { return _planes.end(); }

	const Plane& plane(Plane_handle plh) const { return _planes[plh]; }

	Plane_handle plane_handle(const Plane& pl) const { return Plane_handle(&pl - &_planes[0]); }

	int number_of_planes() const { return int(_planes.size()); }

	/* Bounding box ***********************/

	bool is_bbox_facet(Face_handle fh) const { assert(has_bbox()); return (facet_plane(fh) < number_of_bbox_planes()); }

	bool is_bbox_facet(const Face& f) const { return is_bbox_facet(facet_handle(f)); }

protected:
	enum { _empty_face_handle = 0 };

	enum { _complete_face_handle = 0 };

protected:
	const Face_list& _face_list(int k) const { return _face_lists[k + 1]; }

	Face_list& _face_list(int k) { return _face_lists[k + 1]; }

	const Face& _face(int k, int i) const { return _face_list(k)[i]; }

	Face& _face(int k, int i) { return _face_list(k)[i]; }

	/* The empty face *********************/

	const Face& _empty_face() const { return _face(-1, 0); }

	Face& _empty_face() { return _face(-1, 0); }

	/* The complete face ******************/

	const Face& _complete_face() const { return _face(dimension() + 1, 0); }

	Face& _complete_face() { return _face(dimension() + 1, 0); }

	Planes_const_iterator _find_plane(const Plane& pl) const
	{
		Planes_const_iterator first = planes_begin();
		Planes_const_iterator last = planes_end();

		return std::find(first, last, pl);
	}

	void _insert_bbox_planes(const CGAL::Bbox_3& bbox)
	{
		_insert_bbox_planes(bbox.xmin(), bbox.ymin(), bbox.zmin(),
				bbox.xmax(), bbox.ymax(), bbox.zmax());
	}

	void _insert_bbox_planes(double xmin, double ymin, double zmin,
			double xmax, double ymax, double zmax)
	{
		/* Computer the AABB center */
		const double cx = 0.5 * (xmin + xmax);
		const double cy = 0.5 * (ymin + ymax);
		const double cz = 0.5 * (zmin + zmax);

		const Point p(cx, cy, cz);

		/* Computer the AABB extent */
		const Vector u(xmax - cx,         0,         0);
		const Vector v(        0, ymax - cy,         0);
		const Vector w(        0,         0, zmax - cz);

		_insert_bbox_planes(p, u, v, w);
	}

	void _insert_bbox_planes(const Point& p,
			const Vector& u,
			const Vector& v,
			const Vector& w)
	{
		assert(!CGAL::coplanar(p, p + u, p + v, p + w));
		assert(!has_bbox());

		Plane planes[_number_of_bbox_planes] = {
				/* Inward oriented plane normals */
				Plane(p - u,  u),
				Plane(p - v,  v),
				Plane(p - w,  w),
				/* */
				Plane(p + u, -u),
				Plane(p + v, -v),
				Plane(p + w, -w),
		};

		const int d = dimension();
		const int n = 2 * d;

		assert(n == number_of_bbox_planes());

#ifndef NDEBUG
		const int m =
#endif /* NDEBUG */
				insert(planes, planes + n);

		assert(m == n);
	}

	/*!
	 *  Inserts a new plane in the list unless it is already in it (or
	 *  it is outside the bounding box).
	 *  \return true if the plane was actually inserted
	 */
	virtual bool _insert_plane(const Plane& pl)
	{
		Planes_const_iterator it = _find_plane(pl);

		/* FIXME: Allowed for BSP complex */
		if (it != planes_end())
			return false;

		it = _find_plane(pl.opposite());

		if (it != planes_end())
			return false;

		if (has_bbox() && !_has_bbox_vertices_on_both_sides(pl))
			return false;

		_planes.push_back(pl);

		return true;
	}

	bool _has_bbox_vertices_on_both_sides(const Plane& pl)
	{
		bool has_bbox_vertex_on_positive_side = false;
		bool has_bbox_vertex_on_negative_side = false;

		for (int i = 0; i != _number_of_bbox_vertices; ++i) {
			const Point p = vertex(i).point();

			const int p_side = pl.oriented_side(p);

			if (p_side > 0) {
				has_bbox_vertex_on_positive_side = true;

				if (has_bbox_vertex_on_negative_side)
					return true;
			} else if (p_side < 0) {
				has_bbox_vertex_on_negative_side = true;

				if (has_bbox_vertex_on_positive_side)
					return true;
			}
		}

		return false;
	}

	/*!
	 *  Inserts the given planes in the list if there are not already in it.
	 *  \param first an iterator for the range of planes
	 *  \param last an iterator for the range of planes
	 *  \return the number of actually inserted planes.
	 */
	template<class InputIterator>
	int _insert_planes(InputIterator first, InputIterator last)
	{
		int count = 0;

		for (InputIterator it = first; it != last; ++it)
			if (_insert_plane(*it))
				++count;

		return count;
	}

	/*! Updates the arrangement with the m last inserted planes. */
	void _update_arrangement(int m)
	{
		const int n = int(number_of_planes());
		const int d = dimension();

		if (n < d) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
			std::cerr << "Arrangement_3::_update_arrangement(): "
					"arrangements of less than "
					<< d << " planes are not supported" << std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

			return;
		}

		if (number_of_vertices() != 0) {
			/*
			 * There is already a vertex:
			 *   simply insert all the new planes
			 */
			for (int i = 0; i != m; ++i) {
				const Plane_handle plh = Plane_handle(i + n - m);
				_increment_arrangement(plh);
			}

			return;
		}

#ifdef _MSC_VER
		Plane_handle plhs[_dimension];
#else
		Plane_handle plhs[d];
#endif /* _MSC_VER */

		if (!_choose_initial_planes(plhs)) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
			std::cerr << "Arrangement_3::_update_arrangement(): "
					"can't find a simple sub-arrangement of "
					<< d << " planes" << std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

			return;
		}

		/* Compute the (simple) arrangement of the given d planes */
		_compute_initial_arrangement(plhs);

		for (int i = 0; i != n; ++i) { /* Now insert all the planes */
			const Plane_handle plh = Plane_handle(i);

			/* Check whether the plane was one the initial planes */
			int j;

			for (j = 0; j != d; ++j)
				if (plh == plhs[j])
					break;

			if (j != d)
				continue;

			_increment_arrangement(plh);
		}
	}

	/*!
	 *  Computes an initial (simple) arrangement of d planes (to be
	 *  called only after _choose_initial_planes()).
	 *  \param[in] plhs an array of handles to the initial planes
	 */
	void _compute_initial_arrangement(const Plane_handle* plhs)
	{
#ifndef NDEBUG
		const int n = int(number_of_planes());
#endif /* NDEBUG */
		const int d = dimension();

		assert(n >= d);

		/* Reset the face lists */
		_clear_face_lists();

		/* 3^d proper faces + 2 improper faces */
		const int number_of_proper_faces = d * d * d;

		/* Position vectors of the proper faces */
#ifdef _MSC_VER
		int H[_dimension * _dimension * _dimension][_dimension];
#else
		int H[number_of_proper_faces][d];
#endif /* _MSC_VER */

		/* Generate all position d-vectors. */
		for (int i = 0; i < d; i++)
			H[0][i] = -1;

		for (int i = 1; i < number_of_proper_faces; i++) {
			for (int j = 0; j < d; j++)
				H[i][j] = H[i - 1][j];

			for (int j = 0; j < d; j++)
				if (H[i][j] == 1)
					for (int k = 0; k <= j; k++)
						H[i][k] = -1;
				else {
					++H[i][j];
					break;
				}
		}

		_face_lists[                        0].resize(1);
		_face_lists[_number_of_face_lists - 1].resize(1);

		/* Improper (-1)-face */
		_empty_face().point() = CGAL::ORIGIN;
		_empty_face().info() = WHITE;

		/* Improper (d + 1)-face */
		_complete_face().point() = CGAL::ORIGIN;
		_complete_face().info() = WHITE;

		/* Compute dimension */
#ifdef _MSC_VER
		std::vector<int> face_indices[_dimension + 1];
#else
		std::vector<int> face_indices[d + 1];
#endif /* _MSC_VER */

		for (int i = 0; i != number_of_proper_faces; ++i) {
			const int k = _count_non_zero(d, H[i]);
			Face_list& faces = _face_list(k);

			/* Allocate space for the plane handles of the d-1 faces */
			if (k == d - 1)
				_plane_handles.push_back(Plane_handle());

			/* Add one k-face */
			const int number_of_faces = int(faces.size());
			faces.resize(number_of_faces + 1);

			/* Store the index of this k-face */
			face_indices[k].push_back(i);
		}

		/* FIXME: dimension-dependent code */
		/* The only vertex of the arrangement */
		const Point p = Constructions::intersection(plane(plhs[0]),
				plane(plhs[1]),
				plane(plhs[2]));

		for (int k = 0; k != d + 1; ++k) {
			int i = 0;

			for (Faces_iterator it = faces_begin(k);
					it != faces_end(k); ++it, ++i) {
				Face& f = *it;

				f.info() = WHITE;

				const int face_index = face_indices[k][i];
				const int* h = H[face_index];

				/* Update incidence */
				if (k == 0) { /* The 0-faces (vertices) */
					f.insert_subface(_empty_face_handle);
					_empty_face().insert_superface(Face_handle(i));
				}

				if (k == d) { /* The d-faces (cells) */
					f.insert_superface(_complete_face_handle);
					_complete_face().insert_subface(Face_handle(i));
				}

				if (k != 0) { /* k \in [1, d] */
					int j = 0;

					for (Faces_iterator it2 = faces_begin(k - 1);
							it2 != faces_end(k - 1); ++it2, ++j) {
						Face& f2 = *it2;

						const int face_index2 = face_indices[k - 1][j];
						const int* h2 = H[face_index2];

						if (_count_mismatch(d, h, h2) == 1) {
							f.insert_subface(j);
							f2.insert_superface(i);
						}
					}
				}

				/* Compute the reference point of the face */
				if (k == 0) {
					f.point() = p;
				} else if (k == 1) {
					const int j = _find_non_zero(d, h);

					assert(j < d);

					const Plane& pl1 = plane(plhs[(j + 1) % d]);
					const Plane& pl2 = plane(plhs[(j + 2) % d]);

					const Line l = Constructions::intersection(pl1, pl2);
					const Vector v = l.to_vector();

					const int e_side = h[j] > 0 ? 1 : -1;
					const int q_side =
							Predicates::oriented_side(plane(plhs[j]), p + v);

					assert(q_side != 0);

					if (e_side == q_side)
						f.point() = p + v;
					else
						f.point() = p - v;
				} else { /* The subface are up-to-date */
					f.point() = _compute_face_point(k, i);
				}

				if (k == d - 1) {
					const int j = _find_zero(d, h);

					assert(j != d);

					_plane_handles[i] = plhs[j];
				}
			}
		}
	}

	/*! Returns the number of mismatching coef. in the position vector. */
	static int _count_mismatch(int n, const int* h1, const int* h2)
	{
		int count = 0;

		for (int i = 0; i != n; ++i)
			if (h1[i] != h2[i])
				++count;

		return count;
	}

	/*! Returns the number of non-null coef. in the position vector. */
	static int _count_non_zero(int n, const int* h)
	{
		int count = 0;

		for (int i = 0; i != n; ++i)
			if (h[i] != 0)
				++count;

		return count;
	}

	/*! Finds the (1st) null coef. in the position vector. */
	static int _find_zero(int n, const int* h)
	{
		for (int i = 0; i != n; ++i)
			if (h[i] == 0)
				return i;

		return n; /* == i */
	}

	/*! Finds the (1st) non-null coef. in the position vector. */
	static int _find_non_zero(int n, const int* h)
	{
		for (int i = 0; i != n; ++i)
			if (h[i] != 0)
				return i;

		return n; /* == i */
	}

public:
	const Point& point(const Face& f) const { return f.point(); }

	const Point& point(Face_handle vh) const { return point(vertex(vh)); }

	/*! Returns the affine hull of the given face. */
	Line line(const Face& f) const
	{
#ifndef NDEBUG
		const int n = f.number_of_subfaces();
#endif /* NDEBUG */

		assert(n >= 1);

		const Point& p = point(f.subface(0));
		const Point& q = point(f);

		return Line(p, q);
	}

	/*! Returns the affine hull of the given face. */
	Line line(Face_handle eh) const { return line(edge(eh)); }

	Ray ray(const Face& f) const
	{
		assert(f.is_ray());

		const Point& p = point(f.subface(0));
		const Point& q = point(f);

		return Ray(p, q); /* Ray with source p and passing through q */
	}

	Ray ray(Face_handle eh) const { return ray(edge(eh)); }

	Segment segment(const Face& f) const
	{
		assert(f.is_segment());

		const Point& p = point(f.subface(0));
		const Point& q = point(f.subface(1));

		return Segment(p, q);
	}

	Segment segment(Face_handle eh) const { return segment(edge(eh)); }

	/* FIXME: subsubfaces are visited several times */
	bool is_face_bounded(int k, const Face& f) const
	{
		if (k <= 0)
			return true;

		if (k > dimension())
			return false;

		if (k == 1)
			return f.is_segment();

		for (Subfaces_const_iterator it = f.subfaces_begin();
				it != f.subfaces_end(); ++it) {
			const Face_handle gh = *it;
			const Face& g = face(k - 1, gh);

			if (!is_face_bounded(k - 1, g))
				return false;
		}

		return true;
	}

	bool is_cell_bounded(const Face& f) const { return is_face_bounded(dimension(), f); }

	bool is_cell_bounded(Face_handle ch) const { return is_cell_bounded(cell(ch)); }

	bool is_facet_bounded(const Face& f) const { return is_face_bounded(2, f); }

	bool is_facet_bounded(Face_handle fh) const { return is_facet_bounded(facet(fh)); }

	bool is_edge_bounded(const Face& f) const { return f.is_segment(); }

	bool is_edge_bounded(Face_handle eh) const { return is_edge_bounded(edge(eh)); }

	/*!
	 *  Converts a facet to an oriented polygon.
	 *  \param f a face to polygonize
	 *  \param it an iterator to output the sequence of points
	 *  \return false if the polygon is not bounded in which case, the
	 *  first and last vertices are points along the semi-infinite edges.
	 */
	template<class OutputIterator>
	bool facet_to_polygon(const Face& f, OutputIterator it) const
	{
		/* Check whether the facet is bounded */
		Face_handle eh0 = f.subface(0);

		Subfaces_const_iterator it2;

		for (it2 = f.subfaces_begin(); it2 != f.subfaces_end(); ++it2) {
			const Face_handle eh = *it2;

			if (edge(eh).is_ray()) {
				eh0 = eh;
				break;
			}
		}

		const bool is_bounded = (it2 == f.subfaces_end());

		Face_handle eh = eh0;
		int i = 0;

		do {
			const Face& e = edge(eh);

			if (!is_bounded && eh == eh0)
				it++ = eh;

			const Face_handle vh = e.subface(i);
			const Face& v = vertex(vh);

			it++ = vh;

			Superfaces_const_iterator it2;

			for (it2 = v.superfaces_begin();
					it2 != v.superfaces_end(); ++it2) {
				const Face_handle eh2 = *it2;

				if (eh2 != eh && f.has_subface(eh2)) {
					eh = eh2; /* Update the current edge */
					const Face& e = edge(eh);

					if (!is_bounded && e.is_ray()) {
						it++ = eh;
						eh = eh0;
					} else
						i = (e.subface(0) != vh) ? 0 : 1;

					break;
				}
			}
		} while (eh != eh0);

		return is_bounded;
	}

	/*! Dumps the whole incidence graph on the given stream. */
	void dump_incidence_graph(std::ostream& stream) const
	{
		const int d = dimension();

		for (int k = -1; k <= d + 1; ++k) {
			const int number_of_k_faces = number_of_faces(k);

			stream << "# of " << k << "-faces: " << number_of_k_faces
					<< std::endl;

			for (Faces_const_iterator it = faces_begin(k);
					it != faces_end(k); ++it) {
				const Face& f = *it;
				const Face_handle fh = face_handle(k, f);

				stream << "    " << k << "-face " << fh << ": " << &f
						<< std::endl << "        # of superfaces: "
						<< f.number_of_superfaces()
						<< std::endl << "        superfaces:";

				for (Superfaces_const_iterator it2 = f.superfaces_begin();
						it2 != f.superfaces_end(); ++it2) {
					const Face_handle gh = *it2;
					stream << " (" << (k + 1) << ',' << gh << ") "
							<< static_cast<const void*>(&face(k + 1, gh));
				}

				stream << std::endl << "        # of subfaces: "
						<< f.number_of_subfaces()
						<< std::endl << "        subfaces:";

				for (Subfaces_const_iterator it2 = f.subfaces_begin();
						it2 != f.subfaces_end(); ++it2) {
					const Face_handle gh = *it2;
					stream << " (" << (k - 1) << ',' << gh << ") "
							<< static_cast<const void*>(&face(k - 1, gh));
				}

				stream << std::endl << "        point: " << f.point()
																											<< std::endl << "        color: "
																											<< _color_to_str(Color(f.info())) << std::endl;

				if (k == 2) { /* 2-faces (facet) */
					stream << "        plane: " << plane(facet_plane(fh))
																												<< std::endl;
				}

				if (k == 1) { /* 1-faces (edge) */
					if (f.is_segment())
						stream << "        segment: " << segment(f)
						<< std::endl;

					if (f.is_ray())
						stream << "        ray: " << ray(f)
						<< std::endl;

					stream << "        line: " << line(f) << std::endl;
				}
			}
		}
	}

protected:
	/*! Computes the squared distance between the plane and the vertex. */
	FT _squared_distance(const Plane& pl, const Face& v) const { return Constructions::squared_distance(pl, point(v)); }

	/*! Does the plane contain the vertex? */
	bool _do_contain_vertex(const Plane& pl, const Face& v) const { return Predicates::do_contain(pl, point(v)); }

	/*! Does the plane contain the vertex? */
	bool _do_contain_vertex(const Plane& pl, Face_handle vh) const { return _do_contain_vertex(pl, vertex(vh)); }

	/*! Does the plane intersect the closure of the edge? */
	bool _do_intersect_edge_cl(const Plane& pl, const Face& e) const
	{
		if (e.is_segment())
			return Predicates::do_intersect_cl(pl, segment(e));

		if (e.is_ray())
			return Predicates::do_intersect_cl(pl, ray(e));

		/* The current arrangement has a vertex so no edge is a line. */

		assert(0);

		return false;
	}

	/*! Does the plane intersect the closure of the edge? */
	bool _do_intersect_edge_cl(const Plane& pl, Face_handle eh) const { return _do_intersect_edge_cl(pl, edge(eh)); }

	/*! Does the plane intersect the edge? */
	bool _do_intersect_edge(const Plane& pl, const Face& e) const
	{
		if (e.is_segment())
			return Predicates::do_intersect(pl, segment(e));

		if (e.is_ray())
			return Predicates::do_intersect(pl, ray(e));

		/* The current arrangement has a vertex so no edge is a line. */

		assert(0);

		return false;
	}

	/*! Does the plane intersect the edge? */
	bool _do_intersect_edge(const Plane& pl, Face_handle eh) const { return _do_intersect_edge(pl, edge(eh)); }

	/*! Does the plane contain the edge? */
	bool _do_contain_edge(const Plane& pl, const Face& e) const
	{
		if (e.is_segment())
			return Predicates::do_contain(pl, segment(e));

		if (e.is_ray())
			return Predicates::do_contain(pl, ray(e));

		/* The current arrangement has a vertex so no edge is a line. */

		assert(0);

		return false;
	}

	/*! Does the plane contain the edge? */
	bool _do_contain_edge(const Plane& pl, Face_handle eh) const { return _do_contain_edge(pl, edge(eh)); }

	/*! Is the edge parallel to the plane? */
	bool _is_parallel(const Plane& pl, const Face& e) const { return Predicates::is_parallel(pl, line(e)); }

	/*! Is the edge parallel to the plane? */
	bool _is_parallel(const Plane& pl, Face_handle eh) const { return _is_parallel(pl, edge(eh)); }

	/*! Finds an edge incident to the vertex that is not // to the plane. */
	const Face& _find_incident_edge_not_parallel(const Face& v,
			const Plane& pl) const
	{
		const Face* e = 0;

		for (Superfaces_const_iterator it = v.superfaces_begin();
				it != v.superfaces_end(); ++it) {
			const Face_handle eh = *it;

			if (!_is_parallel(pl, eh)) {
				e = &edge(eh);

				break;
			}
		}

		assert(e != 0);

		return *e;
	}

	/*! Finds an edge that intersects the plane. */
	const Face& _find_intersecting_edge(const Plane& pl) const
	{
		assert(number_of_vertices() >= 1);

		const Face* v = &*vertices_begin();
		FT d = _squared_distance(pl, *v);
		const Face* e = &_find_incident_edge_not_parallel(*v, pl);
		Line l = line(*e);

		/* cl e \cap h = \emptyset */
		while (!_do_intersect_edge_cl(pl, *e)) {
			/* Take the vertex of e that is closer to h */
			if (e->is_segment()) {
				const Face* v2 = (v == &vertex(e->subface(0)))
																										? &vertex(e->subface(1))
																												: &vertex(e->subface(0));
				const FT d2 = _squared_distance(pl, *v2);

				if (d2 < d) { /* The other vertex is closer to the plane */
					v = v2;
					d = d2;
				}
			}

			/* Find another edge e' incident to v such as aff e = aff e' */
#ifndef NDEBUG
			const Face* old_e = e;
			const Line old_l = l;
#endif /* NDEBUG */

			for (Superfaces_const_iterator it = v->superfaces_begin();
					it != v->superfaces_end(); ++it) {
				const Face* e2 = &edge(*it);
				const Line l2 = line(*e2);

				if (e2 != e && (l2 == l || l2.opposite() == l)) {
					e = e2;
					l = l2;

					break;
				}
			}

			assert(e != old_e);
		}

		return *e;
	}

	Face_handle _find_incident_facet(const Face& e) const
	{
		assert(e.number_of_superfaces() > 0);

		return *e.superfaces_begin();
	}

	void _mark_intersecting_0_1_faces(const Plane& pl, const Face& e0,
			Face_handle_list* L)
	{
		typedef std::deque<Face_handle> Face_handle_queue;

		const Face_handle fh = _find_incident_facet(e0);

		Face_handle_queue Q;
		Q.push_back(fh);

		/* Mark intersecting 0-faces (vertices) and 1-faces (edges) */
		while (!Q.empty()) {
			const Face_handle rh = Q.front();
			Q.pop_front();

			const Face& r = facet(rh);

			/* Iterate over all incident subfaces */
			for (Subfaces_const_iterator it = r.subfaces_begin();
					it != r.subfaces_end(); ++it) {
				const Face_handle eh = *it;
				Face& e = edge(eh);

				/* Only consider white edges */
				if (e.info() != WHITE)
					continue;

				/* Only consider intersecting edges */
				if (!_do_intersect_edge_cl(pl, e))
					continue;

				// FIXME: redondant computations
				/* Mark each white incident vertex contained in h crimson */
				for (Subfaces_const_iterator it2 = e.subfaces_begin();
						it2 != e.subfaces_end(); ++it2) {
					const Face_handle vh = *it2;
					Face& v = vertex(vh);

					if (v.info() != WHITE)
						continue;

					if (_do_contain_vertex(pl, v)) {
						v.info() = CRIMSON;

						L[0].push_back(vh);
					}
				}

				// FIXME: redondant computations
				/* Mark the edge pink, crimson or red */
				if (!_do_intersect_edge(pl, e)) { /* e \cap h = \emptyset */
					e.info() = PINK;
				} else if (_do_contain_edge(pl, e)) { /* e \subseteq h */
					e.info() = CRIMSON;
				} else {
					e.info() = RED;
				}

				L[1].push_back(eh);

				/*
				 * Mark all white 2-faces incident upon e green and put
				 * them in the queue
				 */
				for (Superfaces_iterator it2 = e.superfaces_begin();
						it2 != e.superfaces_end(); ++it2) {
					const Face_handle fh = *it2;
					Face& f = facet(fh);

					if (f.info() != WHITE)
						continue;

					f.info() = GREEN;

					Q.push_back(fh);
				}
			}
		}
	}

	int _oriented_side(const Plane& pl, const Face& f) const
	{
		return Predicates::oriented_side(pl, point(f));
	}

	bool _has_pink_subfaces_on_both_sides(int k, const Face& f,
			const Plane& pl) const
	{
		bool has_pink_subface_on_positive_side = false;
		bool has_pink_subface_on_negative_side = false;

		for (Subfaces_const_iterator it = f.subfaces_begin();
				it != f.subfaces_end(); ++it) {
			const Face_handle gh = *it;
			const Face& g = face(k - 1, gh);

			if (g.info() != PINK)
				continue;

			const int side = _oriented_side(pl, g);

			if (side == 1)
				has_pink_subface_on_positive_side = true;
			else if (side == -1)
				has_pink_subface_on_negative_side = true;
		}

		return (has_pink_subface_on_positive_side
				&& has_pink_subface_on_negative_side);
	}

	bool _has_a_red_subface(int k, const Face& f) const
	{
		for (Subfaces_const_iterator it = f.subfaces_begin();
				it != f.subfaces_end(); ++it) {
			const Face_handle gh = *it;
			const Face& g = face(k - 1, gh);

			if (g.info() == RED)
				return true;
		}

		return false;
	}

	bool _has_all_subfaces_crimson(int k, const Face& f) const
	{
		for (Subfaces_const_iterator it = f.subfaces_begin();
				it != f.subfaces_end(); ++it) {
			const Face_handle gh = *it;
			const Face& g = face(k - 1, gh);

			if (g.info() != CRIMSON)
				return false;
		}

		return true;
	}

	void _mark_intersecting_2_d_faces(const Plane& pl,
			Face_handle_list* L)
	{
		const int d = dimension();

		/* Mark intersecting k-faces (k >= 2) */
		for (int k = 2; k != d + 1; ++k) {
			const Face_handle_list& l = L[k - 1];

			for (typename Face_handle_list::const_iterator it = l.begin();
					it != l.end(); ++it) {
				const Face_handle fh = *it;
				const Face& f = face(k - 1, fh);

				for (Superfaces_const_iterator it2 = f.superfaces_begin();
						it2 != f.superfaces_end(); ++it2) {
					const Face_handle gh = *it2;
					Face& g = face(k, gh);

					/* Consider only the white or green superfaces of f */
					if (g.info() != WHITE && g.info() != GREEN)
						continue;

					/*
					 * If f is in L_{k - 1}, it can not possibly be
					 * white.
					 */
					assert(f.info() != WHITE);

					switch (f.info()) {
					case PINK:
						g.info() =
								(_has_a_red_subface(k, g)
										|| _has_pink_subfaces_on_both_sides(k, g, pl))
										? RED
												: PINK;
						break;
					case RED:
						g.info() = RED;
						break;
					case CRIMSON:
						g.info() = _has_all_subfaces_crimson(k, g)
						? CRIMSON
								: PINK;
						break;
					}

					L[k].push_back(gh);
				}
			}
		}
	}

	void _mark_intersecting_faces(const Plane& pl, const Face& e0,
			Face_handle_list* L)
	{
		_mark_intersecting_0_1_faces(pl, e0, L);
		_mark_intersecting_2_d_faces(pl, L);
	}

	bool _is_face_in_cell_cl(int k, Face_handle fh, Face_handle ch) const
	{
		assert(k >= 0);
		assert(k <= 3);

		if (k == 3)
			return (fh == ch);

		const Face& f = face(k, fh);

		for (Superfaces_const_iterator it = f.superfaces_begin();
				it != f.superfaces_end(); ++it) {
			const Face_handle gh = *it;

			if (_is_face_in_cell_cl(k + 1, gh, ch))
				return true;
		}

		return false;
	}

	void _update_marked_faces(Plane_handle plh, Face_handle_list* L)
	{
		const int d = dimension();

		for (int k = 0; k != d + 1; ++k) {
			int i = 0;

			for (typename Face_handle_list::iterator it = L[k].begin();
					it != L[k].end(); ++it, ++i) {
				const Face_handle gh = *it;
				Face& g = face(k, gh);

				switch (g.info()) {
				case PINK:
					g.info() = GREY;
					break;
				case CRIMSON:
					g.info() = BLACK;
					break;
				case RED:
					if (!has_bbox())
						_split_g(plh, k, gh, L, it, i);
					else { /* Has a bounding box */
						if (!_split_ch.empty() && !_is_face_splittable(k, gh))
							break;

						/* Only split bounded faces */
						if (k == 1
								&& is_edge_bounded(g)) /* Edges */
							_split_g(plh, k, gh, L, it, i);
						else if (k == 2
								&& is_facet_bounded(g)) /* Facets */
							_split_g(plh, k, gh, L, it, i);
						else if (k == d
								&& is_cell_bounded(g)) /* Cell */
							_split_g(plh, k, gh, L, it, i);
					}
					break;
				}
			}
		}

		_unmark_faces(L);
	}

	void _remove_g(int k, Face_handle gh, Face& g,
			Superface_list& g_superfaces,
			Subface_list& g_subfaces)
	{
		g_superfaces = g.superfaces();
		g.clear_superfaces();

		for (Superfaces_const_iterator it = g_superfaces.begin();
				it != g_superfaces.end(); ++it)
			face(k + 1, *it).remove_subface(gh);

		g_subfaces = g.subfaces();
		g.clear_subfaces();

		for (Subfaces_const_iterator it = g_subfaces.begin();
				it != g_subfaces.end(); ++it)
			face(k - 1, *it).remove_superface(gh);
	}

	Face_handle _replace_g_by_gm_and_gp(int k, Face_handle gh)
	{
		const Face_handle gmh = gh; /* Simply replace g by g- */
		face(k, gmh).info() = GREY;

		_face_list(k).push_back(Face());
		const Face_handle gph = Face_handle(number_of_faces(k) - 1);
		face(k, gph).info() = GREY;

		if (k == dimension() - 1)
			_plane_handles.push_back(_plane_handles[gh]);

		return gph;
	}

	Face_handle _create_f_and_connect_f_with_gm_and_gp(
			int k,
			Face_handle gmh, Face& gm,
			Face_handle gph, Face& gp,
			Plane_handle plh
	)
	{
		_face_list(k - 1).push_back(Face());
		const Face_handle fh = Face_handle(number_of_faces(k - 1) - 1);

		Face& f = face(k - 1, fh);
		f.info() = BLACK;

		if (k /* - 1 */ == dimension() /* - 1 */)
			_plane_handles.push_back(plh);

		f.insert_superface(gmh);
		gm.insert_subface(fh);
		f.insert_superface(gph);
		gp.insert_subface(fh);

		return fh;
	}

	void _connect_g_superfaces_with_gm_and_gp(
			int k,
			const Superface_list& g_superfaces,
			Face_handle gmh, Face& gm,
			Face_handle gph, Face& gp
	)
	{
		for (Superfaces_const_iterator it = g_superfaces.begin();
				it != g_superfaces.end(); ++it) {
			const Face_handle fh = *it;
			Face& f = face(k + 1, fh);

			gm.insert_superface(fh);
			f.insert_subface(gmh);
			gp.insert_superface(fh);
			f.insert_subface(gph);
		}
	}

	void _connect_g_subfaces_with_gm_or_gp(
			int k,
			const Subface_list& g_subfaces,
			const Plane& pl,
			Face_handle gmh, Face& gm,
			Face_handle gph, Face& gp
	)
	{
		for (Subfaces_const_iterator it = g_subfaces.begin();
				it != g_subfaces.end(); ++it) {
			const Face_handle fh = *it;
			Face& f = face(k - 1, fh);

			if (f.info() != WHITE && f.info() != GREY)
				continue;

			const int side = _oriented_side(pl, f);

			assert(side != 0);

			if (side == -1) {
				gm.insert_subface(fh);
				f.insert_superface(gmh);
			} else { /* side == 1 */
				gp.insert_subface(fh);
				f.insert_superface(gph);
			}
		}
	}

	/* When k = 1. */
	void _connect_f_with_empty_face(Face_handle fh, Face& f)
	{
		f.insert_subface(_empty_face_handle);
		_empty_face().insert_superface(fh);
	}

	/* When g is a segment. */
	void _compute_gm_gp_points_segment(const Plane& pl,
			const Point& p,
			const Point& p1,
			const Point& p2,
			Face& gm,
			Face& gp)
	{
		const int p1_side = Predicates::oriented_side(pl, p1);
#ifndef NDEBUG
		const int p2_side = Predicates::oriented_side(pl, p2);
#endif /* NDEBUG */

		assert(p1_side != 0);
		assert(p2_side != 0);
		assert(p2_side != p1_side);

		if (p1_side == -1) {
			gm.point() = p1 + (p - p1) / FT(2);
			gp.point() = p2 + (p - p2) / FT(2);
		} else {
			gm.point() = p2 + (p - p2) / FT(2);
			gp.point() = p1 + (p - p1) / FT(2);
		}
	}

	/* When g is a ray. */
	void _compute_gm_gp_points_ray(const Plane& pl,
			const Point& p,
			const Point& p1,
			const Point& p2,
			Face& gm,
			Face& gp)
	{
		const int p1_side = Predicates::oriented_side(pl, p1);

		assert(p1_side != 0);

		if (p1_side == -1) {
			gm.point() = p1 + (p  - p1) / FT(2);
			gp.point() = p  + (p2 - p1);
		} else {
			gm.point() = p  + (p2 - p1);
			gp.point() = p1 + (p  - p1) / FT(2);
		}
	}

	/* When k = 1. */
	void _compute_f_gm_gp_points(const Plane& pl,
			const Line& g_line,
			const Point& g_point,
			const Subface_list& g_subfaces,
			Face& f,
			Face& gm,
			Face& gp)
	{
		const Point p = Constructions::intersection(pl, g_line);

		f.point() = p;

		const int number_of_g_subfaces = int(g_subfaces.size());

		const Point& p1 = point(g_subfaces[0]);

		if (number_of_g_subfaces == 2) { /* g is a segment */
			const Point& p2 = point(g_subfaces[1]);

			_compute_gm_gp_points_segment(pl, p, p1, p2, gm, gp);
		} else { /* g is a ray */
			assert(number_of_g_subfaces == 1);

			const Point& p2 = g_point;

			_compute_gm_gp_points_ray(pl, p, p1, p2, gm, gp);
		}
	}

	void _connect_f_with_subfaces(int k,
			Face_handle fh,
			Face& f,
			const Subface_list& g_subfaces)
	{
		for (Subfaces_const_iterator it = g_subfaces.begin();
				it != g_subfaces.end(); ++it) {
			const Face& gs = face(k - 1, *it);

			if (gs.info() != GREY)
				continue;

			for (Subfaces_const_iterator it2 = gs.subfaces_begin();
					it2 != gs.subfaces_end(); ++it2) {
				const Face_handle gssh = *it2;
				Face& gss = face(k - 2, gssh);

				if (gss.info() != BLACK)
					continue;

				if (!f.has_subface(gssh)) {
					f.insert_subface(gssh);
					gss.insert_superface(fh);
				}
			}
		}
	}

	/* When f is a ray. */
	void _compute_f_point_ray(Face& f,
			const Plane& pl, Plane_handle plh,
			const Point& g_point,
			const Subface_list& g_subfaces)
	{
		const Point& p = point(f.subface(0));

		const int number_of_g_subfaces = int(g_subfaces.size());

		assert(number_of_g_subfaces > 1);

		/* Find a point != p */
		Point s /* = CGAL::ORIGIN */;

		/* Find a subface of g intersected by pl */
		int i;

		for (i = 0; i != number_of_g_subfaces; ++i) {
			const Line l = line(g_subfaces[i]);

			const Object obj = CGAL::intersection(pl, l);
			const Point* q = CGAL::object_cast<Point>(&obj);

			if (q == 0) /* pl does not intersect l */
				continue;

			s = *q;

			if (s != p) /* Can this ever happen? */
				break;

			/* Find a subface l2 of g not // to l */
			int j;

			for (j = 0; j != number_of_g_subfaces; ++j) {
				if (j == i)
					continue;

				const Line l2 = line(g_subfaces[j]);

				if (l2.direction() == l.direction()
						|| l2.direction() == -l.direction())
					continue;

				/* Translate s along the direction of l2 */
				const Line l3 = l.transform(
						Aff_transformation(CGAL::Translation(), l2.to_vector())
				);

				const Object obj = CGAL::intersection(pl, l3);
				const Point* q = CGAL::object_cast<Point>(&obj);

				assert(q != 0);

				s = *q;

				break;
			}

			if (j != number_of_g_subfaces)
				break;
		}

		assert(i != number_of_g_subfaces);

		assert(Predicates::oriented_side(pl, s) == 0);
		assert(Predicates::oriented_side(pl, p) == 0);

		/*
		 * Now that we have another point on aff e, ensure that is has
		 * the same position vector as g
		 */
		Planes_const_iterator it;

		for (it = planes_begin(); it != planes_begin() + plh; ++it) {
			const Plane& pl = *it;

			const int s_side = Predicates::oriented_side(pl, s);
			const int g_side = Predicates::oriented_side(pl, g_point);

			if (s_side != g_side)
				break;
		}

		if (it != planes_begin() + plh) {
			s = p - (s - p);

#ifndef NDEBUG
			for (it = planes_begin(); it != planes_begin() + plh; ++it) {
				const Plane& pl = *it;

				const int s_side = Predicates::oriented_side(pl, s);
				const int g_side = Predicates::oriented_side(pl, g_point);

				if (s_side != g_side)
					break;
			}
#endif /* NDEBUG */

			assert(it == planes_begin() + plh);
		}

		f.point() = s;
	}

	/*! Splits the g face. */
	void _split_g(Plane_handle plh, int k, Face_handle gh,
			Face_handle_list* L,
			typename Face_handle_list::iterator& it, int i)
	{
		const Plane& pl = _planes[plh];

		Face& g = face(k, gh);

		/* Make a *copy* of the point */
		const Point g_point = g.point();

		/* Only used when k = 1 */
		const Line g_line = (k == 1) ? line(g) : Line(CGAL::ORIGIN,
				CGAL::ORIGIN);

		/*
		 * 0: remove g from the incidence graph but save its superfaces
		 *    and its subfaces.
		 */
		Superface_list g_superfaces;
		Subface_list g_subfaces;
		_remove_g(k, gh, g, g_superfaces, g_subfaces);

		/*
		 * 1: replace g by g- = g \cap h-
		 *             and g+ = g \cap h+
		 */
		const Face_handle gmh = gh;
		const Face_handle gph = _replace_g_by_gm_and_gp(k, gh);
		Face& gm = face(k, gmh);
		Face& gp = face(k, gph);

		//L[k].push_back(gmh); /* g- simply replaces g */
		L[k].push_back(gph);
		//it = L[k].begin() + i - 1; /* g- is grey, so skipped */
		it = L[k].begin() + i;

		/*
		 * 2: Create the face f = g \cap h,
		 *    connect it to g- and g+
		 */
		const Face_handle fh =
				_create_f_and_connect_f_with_gm_and_gp(k, gmh, gm,
						gph, gp,
						plh);
		Face& f = face(k - 1, fh);

		L[k - 1].push_back(fh);

		/* 3: Connect each superface of g with g- and g+ */
		_connect_g_superfaces_with_gm_and_gp(k, g_superfaces, gmh, gm,
				gph, gp);

		/*
		 * 4: Connect each white or grey subface of g
		 *    with g- (if it is in h-)
		 *      or g+ (if it is in h+)
		 */
		_connect_g_subfaces_with_gm_or_gp(k, g_subfaces, pl, gmh, gm,
				gph, gp);

		/*
		 * 5: If k = 1, connect f with the (-1)-face
		 *        else, connect f with the black subfaces
		 *              of the gray subfaces of g
		 */
		if (k == 1) {
			_connect_f_with_empty_face(fh, f);

			_compute_f_gm_gp_points(pl, g_line, g_point, g_subfaces,
					f, gm, gp);

			assert(_oriented_side(pl, f) == 0);
			assert(_oriented_side(pl, gm) < 0);
			assert(_oriented_side(pl, gp) > 0);
		} else { /* k > 1 */
			assert(k > 1);

			_connect_f_with_subfaces(k, fh, f, g_subfaces);

			if (k == 2) { /* f is a 1-face (edge) */
				const int number_of_f_subfaces = f.number_of_subfaces();

				if (number_of_f_subfaces == 2) { /* segment */
					const Point& p1 = point(f.subface(0));
					const Point& p2 = point(f.subface(1));

					f.point() = p1 + (p2 - p1) / FT(2);
				} else { /* ray */
					assert(number_of_f_subfaces == 1);

					_compute_f_point_ray(f, pl, plh,
							g_point, g_subfaces);
				}
			} else
				f.point() = _compute_face_point(k - 1, fh);

			assert(_oriented_side(pl, f) == 0);

			gm.point() = _compute_face_point(k, gmh);

			assert(_oriented_side(pl, gm) < 0);

			gp.point() = _compute_face_point(k, gph);

			assert(_oriented_side(pl, gp) > 0);
		}
	}

	/*! Marks all grey and black faces white and empty the lists. */
	void _unmark_faces(Face_handle_list* L)
	{
		const int d = dimension();

		for (int k = 0; k != d + 1; ++k) {
			for (typename Face_handle_list::iterator it = L[k].begin();
					it != L[k].end(); ++it) {
				Face& f = face(k, *it);

				//if (f.info() == GREY || f.info() == BLACK)
				f.info() = WHITE;
			}

			L[k].clear();
		}
	}

	void _increment_arrangement(Plane_handle plh)
	{
		const Plane& pl = plane(plh);


#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
		std::cout << "Arrangement_3::_increment_arrangement():"
				<< std::endl
				<< "    incrementing arrangement with plane "
				<< (plh + 1) << ": " << pl << std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

#if defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) \
		&& !defined(NDEBUG)
		//std::cout << "    incidence graph:" << std::endl;
		//dump_incidence_graph(std::cout);
#endif /* defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE)
			&& !defined(NDEBUG) */


#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
		std::cout << "    finding intersecting edge" << std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

		const Face& e0 = _find_intersecting_edge(pl);

#if defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) && !defined(NDEBUG)
		std::cout << "    intersecting edge: "
				<< static_cast<const void*>(&e0)
				<< ": (" << line(e0) << ")" << std::endl;
#endif /* defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) && !defined(NDEBUG) */

		const int d = dimension();

#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
		std::cout << "    marking intersecting faces" << std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

		/* Lists of marked faces */
#ifdef _MSC_VER
		Face_handle_list L[_dimension + 1];
#else
		Face_handle_list L[d + 1];
#endif /* _MSC_VER */
		_mark_intersecting_faces(pl, e0, L);

#if defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) \
		&& !defined(NDEBUG)
		//std::cout << "    marked faces:" << std::endl;
		//_dump_marked_faces(L);
#endif /* defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) \
			&& !defined(NDEBUG) */

#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
		std::cout << "    updating marked faces" << std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

		_update_marked_faces(plh, L);

#if defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) \
		&& !defined(NDEBUG)
		//std::cout << "incidence graph:" << std::endl;
		//dump_incidence_graph(std::cout);
#endif /* defined(POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE) \
			&& !defined(NDEBUG) */
	}

	void _dump_marked_faces(const Face_handle_list* L) const
	{
		const int d = dimension();

		for (int k = 0; k != d + 1; ++k) {
			std::cout << "L" << k << ":" << std::endl;

			for (typename Face_handle_list::const_iterator it =
					L[k].begin();
					it != L[k].end(); ++it) {
				const Face& f = face(k, *it);

				std::cout << "    " << static_cast<const void*>(&f)
																											<< std::endl
																											<< "        point: " << f.point() << std::endl
																											<< "        color: "
																											<< _color_to_str(Color(f.info())) << std::endl;
			}
		}
	}

public:
	/*! Checks whether the incidence relations are consistent. */
	bool check_incidence_graph() const
	{
		const int d = dimension();

		if (number_of_faces(-1) != 1) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
			std::cerr << "Arrangement_3::check_incidence_graph(): "
					"no or more than one (-1)-face"
					<< std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

			return false;
		}

		if (faces_begin(-1)->number_of_subfaces() != 0) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
			std::cerr << "Arrangement_3::check_incidence_graph(): "
					"the (-1)-face has subface(s)"
					<< std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

			return false;
		}

		if (number_of_faces(d + 1) != 1) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
			std::cerr << "Arrangement_3::check_incidence_graph(): "
					"no or more than one (d+1)-face"
					<< std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

			return false;
		}

		if (faces_begin(d + 1)->number_of_superfaces() != 0) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
			std::cerr << "Arrangement_3::check_incidence_graph(): "
					"the (d+1)-face has superface(s)"
					<< std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

			return false;
		}

		for (int k = -1; k != d + 1; ++k) {
			for (Faces_const_iterator it = faces_begin(k);
					it != faces_end(k); ++it) {
				const Face& f = *it;
				const Face_handle fh = face_handle(k, f);

				for (Superfaces_const_iterator it2 = it->superfaces_begin();
						it2 != it->superfaces_end(); ++it2) {
					const Face_handle gh = *it2;

					if (!face(k + 1, gh).has_subface(fh)) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
						std::cerr
						<< "Arrangement_3::check_incidence_graph(): "
						"(" << k << ',' << fh
						<< ") is a subface of ("
						<< (k + 1) << ',' << gh
						<< ") but not the other way round"
						<< std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

						return false;
					}
				}
			}
		}

		for (int k = 0; k != d + 2; ++k) {
			for (Faces_const_iterator it = faces_begin(k);
					it != faces_end(k); ++it) {
				const Face& f = *it;
				const Face_handle fh = face_handle(k, f);

				for (Subfaces_const_iterator it2 = it->subfaces_begin();
						it2 != it->subfaces_end(); ++it2) {
					const Face_handle gh = *it2;

					if (!face(k - 1, gh).has_superface(fh)) {
#ifdef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE
						std::cerr
						<< "Arrangement_3::check_incidence_graph(): "
						"(" << k << ',' << fh
						<< ") is a superface of ("
						<< (k - 1) << ',' << gh
						<< ") but not the other way round"
						<< std::endl;
#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_VERBOSE */

						return false;
					}
				}
			}
		}

		return true;
	}

protected:
	/*!
	 *  Finds d hyperplanes whose normals are linearly independent.
	 *  \param[out] plhs an array of d plane handles
	 *  \return whether these d hyperplanes were found
	 */
	bool _choose_initial_planes(Plane_handle* plhs)
	{
		const int n = int(number_of_planes());
		const int d = dimension();

		if (n < d)
			return false;

		plhs[0] = Plane_handle(0);

		int i = 1, j = 1;

		while (i < n && j < d) {
			plhs[j] = Plane_handle(i);

			const int r = _linear_rank(j + 1, plhs);

			if (r == j + 1) /* Intersection is a (d-j-1)-flat */
				j++;

			i++;
		}

		return (j == d);
	}

	/*!
	 *  Computes the reference point of the face with the points
	 *  from the subfaces.
	 */
	Point _compute_face_point(int k, int i) const
	{
		const Face& f = face(k, i);

		Point p = CGAL::ORIGIN;

		const RT n = int(f.number_of_subfaces());

		assert(n >= 2);

		for (Subfaces_const_iterator it = f.subfaces_begin();
				it != f.subfaces_end(); ++it) {
			const Face& g = face(k - 1, *it);

			p = p + (g.point() - CGAL::ORIGIN) / n;
		}

		return p;
	}

protected:
	typedef K_d Kernel_d;
	typedef typename Kernel_d::Vector_d Vector_d;

protected:
	/*! Computes the rank of the normals of the given hyperplanes. */
	int _linear_rank(int n, const Plane_handle* plhs) const
	{
		std::vector<Vector_d> vectors;

		for (int i = 0; i != n; ++i) {
			const Plane& pl = plane(plhs[i]);
			const Vector v = pl.orthogonal_vector();

			vectors.push_back(Vector_d(v.hx(),
					v.hy(),
					v.hz(),
					v.hw()));
		}

		return CGAL::linear_rank(vectors.begin(), vectors.end());
	}
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_3_HPP */
