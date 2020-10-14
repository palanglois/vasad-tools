#ifndef POLYHEDRAL_COMPLEX_3_ARRANGEMENT_FACE_BASE_3_HPP
#define POLYHEDRAL_COMPLEX_3_ARRANGEMENT_FACE_BASE_3_HPP

#include<algorithm>
#include<cassert>
#include<vector>
#include <string>
#include <fstream>

namespace Polyhedral_complex_3 {

template<class K,class Fh>
class Arrangement_face_base_3;

template<class K,class Fh>
std::ostream & operator<<(std::ostream &os, const Arrangement_face_base_3<K,Fh> & face);
template<class K,class Fh>
std::istream & operator>>(std::istream &is,  Arrangement_face_base_3<K,Fh> & face);

template<class K,
class Fh>
class Arrangement_face_base_3 {
public:
	typedef K Kernel;
	typedef typename Kernel::Point_3 Point;
	typedef typename Kernel::Line_3 Line;
	typedef typename Kernel::Ray_3 Ray;
	typedef typename Kernel::Segment_3 Segment;

	typedef Fh Face_handle;

	typedef std::vector<Face_handle> Superface_list;
	typedef typename Superface_list::iterator Superfaces_iterator;
	typedef typename Superface_list::const_iterator
			Superfaces_const_iterator;
	typedef std::vector<Face_handle> Subface_list;
	typedef typename Subface_list::iterator Subfaces_iterator;
	typedef typename Subface_list::const_iterator
			Subfaces_const_iterator;


	//Modifications
public:
	enum{VERTEX, EDGE, FACET, CELL};

	int type;

	bool is_bbox_adj; //boolean for adjacency at the bbox
	bool to_draw;
	std::string ifcType;

	// double facet_nbr_ray;
	// double facet_nbr_pts;
	double facet_nbr_ray_vis_point;
	double facet_nbr_ray_vis_line;
	double facet_nbr_ray_data_point;
	double facet_nbr_ray_data_line;
	
	double cell_value_void_points;
	double cell_value_full_points;
	double cell_value_void_view_points;

	float dist_to_points;


	int cell_type;
	int void_volume_nb; // void volume number of points
	
	double area;
	double solid_angle;
	double density;

	Arrangement_face_base_3(): type(-1), void_volume_nb(0), is_bbox_adj(false), cell_type(-1),
			//MODIF
			ifcType("-"),
			to_draw(false),
			// facet_nbr_ray(0),
			// facet_nbr_pts(0),
			facet_nbr_ray_vis_point(0),
			facet_nbr_ray_vis_line(0),
			facet_nbr_ray_data_point(0),
			facet_nbr_ray_data_line(0),
			cell_value_full_points(0),
			cell_value_void_points(0),
			cell_value_void_view_points(0),
			dist_to_points(1e7), //infinite distance to the points of the primitive
			area(0),
			solid_angle(0),
			density(0),
			_point(Point(0,0,0)), _info(-1){}

public:


	const Point& point() const { return _point; }

	Point& point() { return _point; }

	const int& info() const { return _info; }

	int& info() { return _info; }

	/* Superfaces *************************/

	void insert_superface(Face_handle fh)
	{
		assert(!has_superface(fh));

		_superfaces.push_back(fh);
	}

	void remove_superface(Face_handle fh)
	{
		assert(has_superface(fh));

		const Superfaces_iterator it = _find_superface(fh);
		_superfaces.erase(it);
	}

	Superfaces_const_iterator superfaces_begin() const { return _superfaces.begin(); }

	Superfaces_iterator superfaces_begin() { return _superfaces.begin(); }

	Superfaces_const_iterator superfaces_end() const { return _superfaces.end(); }

	Superfaces_iterator superfaces_end() { return _superfaces.end(); }

	int number_of_superfaces() const { return int(_superfaces.size()); }

	bool has_superface(Face_handle fh) const
	{
		const Superfaces_const_iterator last = superfaces_end();

		return (_find_superface(fh) != last);
	}

	Face_handle superface(int i) const
	{
		assert(i >= 0 && i < number_of_superfaces());

		return _superfaces[i];
	}

	Face_handle& superface(int i)
	{
		assert(i >= 0 && i < number_of_superfaces());

		return _superfaces[i];
	}

	const Superface_list& superfaces() const { return _superfaces; }

	Superface_list& superfaces() { return _superfaces; }

	void clear_superfaces() { _superfaces.clear(); }

	/* Subfaces ***************************/

	void insert_subface(Face_handle fh)
	{
		assert(!has_subface(fh));

		_subfaces.push_back(fh);
	}

	void remove_subface(Face_handle fh)
	{
		assert(has_subface(fh));

		const Subfaces_iterator it = _find_subface(fh);
		_subfaces.erase(it);
	}

	Subfaces_const_iterator subfaces_begin() const { return _subfaces.begin(); }

	Subfaces_iterator subfaces_begin() { return _subfaces.begin(); }

	Subfaces_const_iterator subfaces_end() const { return _subfaces.end(); }

	Subfaces_iterator subfaces_end() { return _subfaces.end(); }

	int number_of_subfaces() const { return int(_subfaces.size()); }

	bool has_subface(Face_handle fh) const
	{
		const Subfaces_const_iterator last = subfaces_end();

		return (_find_subface(fh) != last);
	}

	Face_handle subface(int i) const
	{
		assert(i >= 0 && i < number_of_subfaces());

		return _subfaces[i];
	}

	Face_handle& subface(int i)
	{
		assert(i >= 0 && i < number_of_subfaces());

		return _subfaces[i];
	}

	const Subface_list& subfaces() const { return _subfaces; }

	Subface_list& subfaces() { return _subfaces; }

	void clear_subfaces() { _subfaces.clear(); }

	/**************************************/

	/*! Whether the (1-)face is a segment. */
	bool is_segment() const { return (number_of_subfaces() == 2); }

	/*! Whether the (1-)face is a ray. */
	bool is_ray() const { return (number_of_subfaces() == 1); }

protected:
	Superfaces_const_iterator _find_superface(Face_handle fh) const
	{
		const Superfaces_const_iterator first = superfaces_begin();
		const Superfaces_const_iterator last = superfaces_end();

		return std::find(first, last, fh);
	}

	Superfaces_iterator _find_superface(Face_handle fh)
	{
		const Superfaces_iterator first = superfaces_begin();
		const Superfaces_iterator last = superfaces_end();

		return std::find(first, last, fh);
	}

	Subfaces_const_iterator _find_subface(Face_handle fh) const
	{
		const Subfaces_const_iterator first = subfaces_begin();
		const Subfaces_const_iterator last = subfaces_end();

		return std::find(first, last, fh);
	}

	Subfaces_iterator _find_subface(Face_handle fh)
	{
		const Subfaces_iterator first = subfaces_begin();
		const Subfaces_iterator last = subfaces_end();

		return std::find(first, last, fh);
	}

protected:
	/* Incidence information */
	Subface_list _subfaces;
	Superface_list _superfaces;

	/* Auxiliary information */
	Point _point;

	//MODIF
public:
    int _info; /* Store the color during the arrangement construction */

	friend std::ostream& operator<< <K,Fh>(std::ostream &os, const Arrangement_face_base_3<K,Fh> & face);
	friend std::istream& operator>> <K,Fh>(std::istream &is, Arrangement_face_base_3<K,Fh> & face);

#if 0
	std::ostream & operator<<(std::ostream &os){
		os << type << ' ';
		os << points.size()<<' ';
		for(int i=0; i<points.size(); i++){
			os << CGAL::to_double(points[i].x()) << ' ';
			os << CGAL::to_double(points[i].y()) << ' ';
			os << CGAL::to_double(points[i].z()) << ' ';
		}
		os << scores.size() << ' ';
		for(int i=0; i<scores.size(); i++){
			os << scores[i] << ' ';
		}
		os << is_bbox_adj << ' ';
		os << to_draw << ' ';
		os << ifcType << ' ';
		os << facet_score << ' ';
		os << cell_type << ' ';
		os << void_volume_nb << ' ';
		os << _subfaces.size() << ' ';
		for(int i=0; i<_subfaces.size(); i++){
			os << (int)_subfaces[i] << ' ';
		}
		os << _superfaces.size() << ' ';
		for(int i=0; i<_superfaces.size(); i++){
			os << (int) _superfaces[i] << ' ';
		}
		os << CGAL::to_double(_point.x()) << ' ';
		os << CGAL::to_double(_point.y()) << ' ';
		os << CGAL::to_double(_point.z()) << ' ';
		os << _info << std::endl;
		return os;
	}
#endif
#if 0
	std::istream & operator>>(std::istream &is){
		is >> type;
		int temp_i;
		is >> temp_i;
		points.resize(temp_i);
		for(int i=0; i<points.size(); i++){
			double tx,ty,tz;
			is >> tx >> ty >> tz;
			points[i] = Point(tx,ty,tz);
		}
		is >> temp_i;
		scores.resize(temp_i);
		for(int i=0; i<scores.size(); i++){
			is >> scores[i];
		}
		is>> is_bbox_adj;
		is>> to_draw;
		is>> ifcType;
		is>> facet_score;
		is>> cell_type;
		is>> void_volume_nb;

		is >> temp_i;
		_subfaces.resize(temp_i);
		for(int i=0; i<_subfaces.size(); i++){
			is >> _subfaces[i];
		}
		is >> temp_i;
		_superfaces.resize(temp_i);
		for(int i=0; i<_superfaces.size(); i++){
			is >> _superfaces[i];
		}
		double tx, ty,tz;
		is >> tx >> ty >> tz;
		_point = Point(tx,ty,tz);
		is >>_info;
		return is;
	}
#endif
	//END MODIF
};


template<class K,class Fh>
std::ostream & operator<<(std::ostream &os, const Arrangement_face_base_3<K,Fh> & face){

	os << face.type << ' ';
	os << face.is_bbox_adj << ' ';
	os << face.to_draw << ' ';
	os << face.ifcType << ' ';
	os << face.dist_to_points << ' ';
	os << face.facet_nbr_ray << ' ';
	os << face.facet_nbr_pts << ' ';
	os << face.cell_type << ' ';
	os << face.void_volume_nb << ' ';

	os << face._subfaces.size() << ' ';
	for(int i=0; i<face._subfaces.size(); i++){
		os << (int)face._subfaces[i] << ' ';
	}
	os << face._superfaces.size() << ' ';
	for(int i=0; i<face._superfaces.size(); i++){
		os << (int) face._superfaces[i] << ' ';
	}
	os << CGAL::to_double(face._point.x()) << ' ';
	os << CGAL::to_double(face._point.y()) << ' ';
	os << CGAL::to_double(face._point.z()) << ' ';
	os << face._info;/**/
	os << std::endl;
	return os;
}


template<class K,class Fh>
std::istream & operator>>(std::istream &is, Arrangement_face_base_3<K,Fh> & face){

	is >> face.type;
	int temp_i;
	is>> face.is_bbox_adj;
	is>> face.to_draw;
	is>> face.ifcType;
	is>> face.dist_to_points;
	is>> face.facet_nbr_ray;
	is>> face.facet_nbr_pts;
	is>> face.cell_type;
	is>> face.void_volume_nb;

	is >> temp_i;
	face._subfaces.clear();
	for(int i=0; i<temp_i; i++){
		int temp;
		is >> temp;
		face._subfaces.push_back(temp);
	}
	is >> temp_i;
	face._superfaces.clear();
	for(int i=0; i<temp_i; i++){
		int temp;
		is >> temp;
		face._superfaces.push_back(temp);
	}
	double tx, ty,tz;
	is >> tx >> ty >> tz;
	face._point = typename K::Point_3(tx,ty,tz);
	is >>face._info;/**/
	return is;
}

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_ARRANGEMENT_FACE_BASE_3_HPP */
