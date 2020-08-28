#ifndef POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_BGL_HPP
#define POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_BGL_HPP

#include<algorithm>
#include<cstddef>
#include<iterator>
#include<utility>

//#ifndef NDEBUG
# include<iostream>
using std::cout;
using std::cerr;
using std::endl;
//#endif /* NDEBUG */

#include "boost/graph/graph_traits_Polyhedral_complex_cells_binary_labeler_3_mincut_BGL.hpp"

#include <boost/version.hpp>
#if BOOST_VERSION >= 104400
/* simple macro to accept multiple version of boost */
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#else
#include <boost/graph/kolmogorov_max_flow.hpp>
#endif

#include <Polyhedral_complex_3/Polyhedral_complex_cells_binary_labeler_3.hpp>




namespace Polyhedral_complex_3 {

template<class Comp,
//class CostT = float,
class CostT = double,
class CapacityT = int>
//class CapacityT = long long>
class Polyhedral_complex_cells_binary_labeler_3_mincut_BGL :
		public Polyhedral_complex_cells_binary_labeler_3<Comp,
		CostT,
		CapacityT> {
		public:
	typedef Polyhedral_complex_cells_binary_labeler_3<Comp,
			CostT,
			CapacityT> Base;
	typedef typename Base::cost_type cost_type;
	typedef typename Base::capacity_type capacity_type;
	typedef typename Base::Face Face;
	typedef typename Base::Face_handle Face_handle;
	typedef typename Base::Labeler_category Labeler_category;

	typedef Polyhedral_complex_cells_binary_labeler_3_mincut_BGL<
			Comp,
			CostT,
			CapacityT
			> Self;

		protected:
	typedef typename Base::Faces_const_iterator Faces_const_iterator;

	typedef typename Comp::Subfaces_const_iterator Subfaces_const_iterator;

		public:
	enum {
		source_vertex = -2,
		sink_vertex   = -1,
	};

		public:
	Polyhedral_complex_cells_binary_labeler_3_mincut_BGL(const Comp& comp) :
		Base(comp),
		_cell_colors(0)
		{
		for (int i = 0; i != 6; ++i)
			_residual_edge_capacities[i] = 0;

		reserve();
		}

	void clear()
	{
		_clear_cell_colors();
		Base::clear();
	}

	void reserve()
	{
		Base::reserve();
		_reserve_cell_colors();
	}

	/**********************************************************/

	bool is_cell_inside(Face_handle ch) const { return (color(ch) == color(sink_vertex)); }

	bool is_cell_inside(const Face& c) const { return is_cell_inside(this->_comp.cell_handle(c)); }

	void label_cell(Face_handle ch, bool inside) { color(ch) = color(inside ? sink_vertex : source_vertex); }

	void label_cell(const Face& c, bool inside) { label_cell(this->_comp.cell_handle(c), inside); }

	/**********************************************************/

	void compute_labeling(bool free_costs = false)
	{
		assert(this->_cell_costs);
		assert(this->_facet_costs[0]);
		assert(this->_facet_costs[1]);
		assert(_cell_colors);

#ifndef NDEBUG
		const int number_of_vertices = this->_comp.number_of_cells();
		const int number_of_edges = this->_comp.number_of_facets();

		std::cout << "Polyhedral_complex_cells_binary_labeler_3_mincut_BGL:"
				<< std::endl
				<< "    # of vertices: "
				<< number_of_vertices << std::endl
				<< "    # of    edges: "
				<< number_of_edges << std::endl
				<< "    computing minimum s-t cut" << std::endl;
#endif /* NDEBUG */

		_reserve_residual_edge_capacities();

#if BOOST_VERSION >= 104400

		boykov_kolmogorov_max_flow(*this,
				edge_capacity_map(*this),
				residual_edge_capacity_map(*this),
				reverse_edge_map(*this),
				color_map(*this),
				index_map(),
				source_vertex,
				sink_vertex);
#else
		kolmogorov_max_flow(*this,
				edge_capacity_map(*this),
				residual_edge_capacity_map(*this),
				reverse_edge_map(*this),
				color_map(*this), index_map(),
				source_vertex, sink_vertex);
#endif

		_clear_residual_edge_capacities();

		/* Deallocate the cell and facet costs */
		if (free_costs) {
			this->_clear_facet_costs();
			this->_clear_cell_costs();
		}

#ifndef NDEBUG
		const int number_of_inside_cells = this->_count_inside_cells();
		const int number_of_outside_cells = number_of_vertices
				- this->_count_inside_cells();

		std::cout << "    # of  inside cells: "
				<< number_of_inside_cells << std::endl
				<< "    # of outside cells: "
				<< number_of_outside_cells << std::endl;
#endif /* NDEBUG */
	}

	/**********************************************************/

	bool operator()(Face_handle ch) const { return is_cell_inside(ch); }

	bool operator()(const Face& c) const { return is_cell_inside(c); }

		protected:
	boost::default_color_type* _cell_colors;
	capacity_type* _residual_edge_capacities[6];

		protected:
	void _clear_cell_colors()
	{
		delete[] _cell_colors;
		_cell_colors = 0;
	}

	void _clear_residual_edge_capacities()
	{
		for (int i = 0; i != 6; ++i) {
			delete[] _residual_edge_capacities[i];
			_residual_edge_capacities[i] = 0;
		}
	}

	void _reserve_cell_colors()
	{
		_clear_cell_colors(); /* Safeguard */

		const int number_of_cells = this->_comp.number_of_cells();

		assert(_cell_colors == 0);
		_cell_colors = new boost::default_color_type[number_of_cells + 2];

		color(source_vertex) =
				boost::color_traits<boost::default_color_type>::white();
		color(sink_vertex) =
				boost::color_traits<boost::default_color_type>::black();

		std::fill(_cell_colors + 2, _cell_colors + 2 + number_of_cells,
				color(source_vertex));
	}

	void _reserve_residual_edge_capacities()
	{
		_clear_residual_edge_capacities(); /* Safeguard */

		const int number_of_cells = this->_comp.number_of_cells();

		for (int i = 0; i != 4; ++i)
			_residual_edge_capacities[i] =
					new capacity_type[number_of_cells];

		const int number_of_facets = this->_comp.number_of_facets();

		_residual_edge_capacities[4] = new capacity_type[number_of_facets];
		_residual_edge_capacities[5] = new capacity_type[number_of_facets];
	}

		public:
	/* Interface with Boost Graph Library *********************/

	/* Vertices ***************************/

	typedef int vertex_descriptor; /* XXX: should be a Face_handle? */

	typedef std::size_t vertices_size_type;

	vertices_size_type num_vertices() const { return (this->_comp.number_of_cells() + 2); }

	class vertex_iterator {
	public:
		typedef std::input_iterator_tag iterator_category;
		typedef vertex_descriptor value_type;
		typedef int difference_type;
		typedef vertex_descriptor pointer;
		typedef vertex_descriptor reference;

	public:
		vertex_iterator() { }
		vertex_iterator(vertex_descriptor v) : _v(v) { }
		vertex_iterator(const vertex_iterator& it) : _v(it) { }
		operator vertex_descriptor() const { return _v; }
		bool operator ==(vertex_iterator it) const { return _v == it._v; }
		bool operator !=(vertex_iterator it) const { return _v != it._v; }
		vertex_iterator operator++() { return ++_v; }
		vertex_iterator operator++(int) { return _v++; }
		vertex_descriptor operator*() const { return _v; }

	protected:
		vertex_descriptor _v;
	};

	std::pair<vertex_iterator,
	vertex_iterator> vertices() const
	{
		return std::make_pair(vertex_iterator(source_vertex),
				vertex_iterator(this->_comp.number_of_cells()));
	}

	/* Edges ******************************/

	/*
	 * Each cell has several directed edges:
	 *     0 : from source
	 *     1 : from sink
	 *     2 :   to source
	 *     3 :   to sink
	 *     >3:   to neighbors (# - 4 is the cell subface number)
	 */
	enum {
		EDGE_FROM_SOURCE = 0,
		EDGE_FROM_SINK   = 1,
		/* */
		EDGE_TO_SOURCE = 2,
		EDGE_TO_SINK   = 3,
		/* */
		FIRST_EDGE_TO_NEIGHBOR = 4,
	};

	/*
	 * Redefine std::pair in this namespace to avoid Koenig lookup
	 * of source() and target() functions
	 */
	template<class T1, class T2>
	struct pair {
		typedef T1 first_type;
		typedef T2 second_type;

		T1 first;
		T2 second;

		pair() : first(), second() { }

		pair(const T1& first, const T2& second) : first(first), second(second) { }

		pair(const pair& p) : first(p.first), second(p.second) { }

		pair& operator=(const pair& p) { first = p.first; second = p.second; return *this; }

		bool operator==(const pair& p) const { return (first == p.first && second == p.second); }

		bool operator!=(const pair& p) const { return (first != p.first || second != p.second); }

		bool operator<(const pair& p) { return (first < p.first || (!(p.first < second) && second < p.second)); }
	};

	template<class T1, class T2>
	pair<T1, T2> make_pair(const T1& first, const T2& second) { return pair<T1, T2>(first, second); }

	typedef pair<vertex_descriptor,
			int> edge_descriptor;

	vertex_descriptor source(edge_descriptor e) const
	{
		if (e.second == EDGE_FROM_SOURCE)
			return source_vertex;

		if (e.second == EDGE_FROM_SINK)
			return sink_vertex;

		return e.first;
	}

	vertex_descriptor target(edge_descriptor e) const
	{
		if (e.second == EDGE_TO_SOURCE)
			return source_vertex;

		if (e.second == EDGE_TO_SINK)
			return sink_vertex;

		const Face_handle ch = e.first;

		if (e.second == EDGE_FROM_SOURCE
				|| e.second == EDGE_FROM_SINK)
			return ch;

		const int i = e.second - FIRST_EDGE_TO_NEIGHBOR;

		return this->_comp.cell_neighbor(ch, i);
	}

	typedef std::size_t edges_size_type;

	vertices_size_type num_edges() const
	{
		return (2 * this->_comp.number_of_facets()
				+ 4 * this->_comp.number_of_cells());
	}

	class edge_iterator : public edge_descriptor {
	public:
		typedef std::input_iterator_tag iterator_category;
		typedef edge_descriptor value_type;
		typedef int difference_type;
		typedef edge_descriptor pointer;
		typedef edge_descriptor reference;

	public:
		edge_iterator() : _comp(0) { }
		edge_iterator(const edge_iterator& it) : edge_descriptor(it), _comp(it._comp) { }
		edge_iterator(const Comp& comp, vertex_descriptor v, int i) : edge_descriptor(v, i), _comp(&comp) { }

		edge_iterator& operator++()
                				{
			this->second++;

			assert(_comp);

			const Face& c = _comp->cell(this->first);

			if (this->second < FIRST_EDGE_TO_NEIGHBOR
					+ c.number_of_subfaces())
				return *this;

			this->second = 0;
			this->first++;

			return *this;
                				}

		edge_iterator operator++(int)
                				{
			const edge_iterator it = *this;
			++*this;

			return it;
                				}

		edge_descriptor operator*() const { return *this; }

	protected:
		const Comp* _comp;
	};

	std::pair<edge_iterator,
	edge_iterator> edges() const
	{
		const Comp& comp = this->_comp;

		return std::make_pair(edge_iterator(comp, 0, 0),
				edge_iterator(comp, comp.number_of_cells(), 0));
	}

	/* Incidence **************************/

	typedef std::size_t degree_size_type;

	degree_size_type out_degree(vertex_descriptor v) const
	{
		if (v == source_vertex
				|| v == sink_vertex)
			return this->_comp.number_of_cells();

		return this->_comp.cell(v).number_of_subfaces();
	}

	class out_edge_iterator : public edge_descriptor {
	public:
		typedef std::input_iterator_tag iterator_category;
		typedef edge_descriptor value_type;
		typedef int difference_type;
		typedef edge_descriptor pointer;
		typedef edge_descriptor reference;

	public:
		out_edge_iterator() { }
		out_edge_iterator(const out_edge_iterator& it) : edge_descriptor(it) { }
		out_edge_iterator(vertex_descriptor v, int i) : edge_descriptor(v, i) { }

		out_edge_iterator& operator++()
                				{
			/* Out edges of source vertex */
			if (this->second == EDGE_FROM_SOURCE) {
				this->first++;

				return *this;
			}

			/* Out edges of sink vertex */
			if (this->second == EDGE_FROM_SINK) {
				this->first++;

				return *this;
			}

			/* Out egdes of other vertices */
			this->second++;

			return *this;
                				}

		out_edge_iterator operator++(int)
                				{
			out_edge_iterator it = *this;
			++*this;

			return it;
                				}

		edge_descriptor operator*() const { return *this; }
	};

	std::pair<out_edge_iterator,
	out_edge_iterator> out_edges(vertex_descriptor v) const
	{
		const Comp& comp = this->_comp;

		if (v == source_vertex)
			return std::make_pair(out_edge_iterator(0,
					EDGE_FROM_SOURCE),
					out_edge_iterator(comp.number_of_cells(),
							EDGE_FROM_SOURCE));

		if (v == sink_vertex)
			return std::make_pair(out_edge_iterator(0,
					EDGE_FROM_SINK),
					out_edge_iterator(comp.number_of_cells(),
							EDGE_FROM_SINK));

		const int num_neighbors = comp.cell(v).number_of_subfaces();

		return std::make_pair(out_edge_iterator(v,
				EDGE_TO_SOURCE),
				out_edge_iterator(v,
						FIRST_EDGE_TO_NEIGHBOR
						+ num_neighbors));
	}

	std::pair<edge_descriptor,
	bool> edge(vertex_descriptor u,
			vertex_descriptor v) const
			{
		if (u == source_vertex) {
			if (v != sink_vertex && v != source_vertex)
				return std::make_pair(edge_descriptor(v, EDGE_FROM_SOURCE),
						true);

			return std::make_pair(edge_descriptor(),
					false);
		}

		if (u == sink_vertex) {
			if (v != sink_vertex && v != source_vertex)
				return std::make_pair(edge_descriptor(v, EDGE_FROM_SINK),
						true);

			return std::make_pair(edge_descriptor(),
					false);
		}

		/* u is a regular vertex, i.e. a cell */
		if (v == source_vertex)
			return std::make_pair(edge_descriptor(u, EDGE_TO_SOURCE),
					true);

		if (v == sink_vertex)
			return std::make_pair(edge_descriptor(u, EDGE_TO_SINK),
					true);

		/* Not used by boykov_kolmogorov_max_flow() */
		cout << "Attention" << endl;
		return std::make_pair(edge_descriptor(),
				false);
			}

	/* Property maps **********************/

	class index_map : public boost::put_get_helper<int,
	index_map> {
	public:
		typedef boost::lvalue_property_map_tag category;
		typedef int value_type;
		typedef const int& reference;
		typedef vertex_descriptor key_type;

	public:
		index_map() { }
		value_type operator[](key_type v) const { return (v + 2); }
	};

	typedef capacity_type edge_capacity_type;

	class edge_capacity_map :
			public boost::put_get_helper<edge_capacity_type,
			edge_capacity_map> {
			public:
		typedef boost::lvalue_property_map_tag category;
		typedef edge_capacity_type value_type;
		typedef const edge_capacity_type& reference;
		typedef edge_descriptor key_type;

			public:
		edge_capacity_map(const Self& labeler) : _labeler(labeler) { }
		value_type operator[](key_type e) const { return _labeler.edge_capacity(e); }

			protected:
		const Self& _labeler;
	};

	edge_capacity_type edge_capacity(edge_descriptor e) const
	{
		if (e.second == EDGE_TO_SOURCE
				|| e.second == EDGE_FROM_SINK)
			return 0;

		const Face_handle ch = e.first;

		if (e.second == EDGE_FROM_SOURCE)
			return std::max(-this->_map_cost_to_capacity(this->cell_cost(ch)),
					edge_capacity_type(0));

		if (e.second == EDGE_TO_SINK)
			return std::max( this->_map_cost_to_capacity(this->cell_cost(ch)),
					edge_capacity_type(0));

		const int i = e.second - FIRST_EDGE_TO_NEIGHBOR;
		const Face_handle fh = this->_comp.cell(e.first).subface(i);
		const int index = (this->_comp.facet(fh).superface(0) == ch ? 0 : 1);

		return this->_map_cost_to_capacity(this->facet_cost(fh, index));
	}

	class residual_edge_capacity_map :
			public boost::put_get_helper<edge_capacity_type,
			residual_edge_capacity_map> {
			public:
		typedef boost::lvalue_property_map_tag category;
		typedef edge_capacity_type value_type;
		typedef edge_capacity_type& reference;
		typedef edge_descriptor key_type;

			public:
		residual_edge_capacity_map(Self& labeler) : _labeler(labeler) { }
		reference operator[](key_type e) const { return _labeler.residual_edge_capacity(e); }

			protected:
		Self& _labeler;
	};

	edge_capacity_type& residual_edge_capacity(edge_descriptor e)
	{
		if (e.second >= FIRST_EDGE_TO_NEIGHBOR) {
			const Face_handle ch = e.first;
			const int i = e.second - FIRST_EDGE_TO_NEIGHBOR;
			const Face_handle fh = this->_comp.cell(ch).subface(i);
			const int index = (this->_comp.facet(fh).superface(0) == ch ? 0 : 1);

			const int j = FIRST_EDGE_TO_NEIGHBOR + index;

			return _residual_edge_capacities[j][fh];
		}

		return _residual_edge_capacities[e.second][e.first];
	}

	class reverse_edge_map :
			public boost::put_get_helper<edge_descriptor,
			reverse_edge_map> {
			public:
		typedef boost::lvalue_property_map_tag category;
		typedef edge_descriptor value_type;
		typedef const edge_descriptor& reference;
		typedef edge_descriptor key_type;

			public:
		reverse_edge_map(const Self& labeler) : _labeler(labeler) { }
		value_type operator [](key_type e) const { return _labeler.reverse_edge(e); }

			protected:
		const Self& _labeler;
	};

	edge_descriptor reverse_edge(edge_descriptor e) const
	{
		if (e.second >= FIRST_EDGE_TO_NEIGHBOR) {
			const Face_handle ch = e.first;
			const int i = e.second - FIRST_EDGE_TO_NEIGHBOR;
			const Face_handle fh = this->_comp.cell(ch).subface(i);

			const Face_handle ch1 = this->_comp.cell_neighbor(ch, i);
			const Face& c1 = this->_comp.cell(ch1);
			const Subfaces_const_iterator first = c1.subfaces_begin();
			const Subfaces_const_iterator last = c1.subfaces_end();
			const Subfaces_const_iterator it = std::find(first, last, fh);
			assert(it != last);

			const int j = it - first + FIRST_EDGE_TO_NEIGHBOR;

			return edge_descriptor(ch1, j);
		}

		if (e.second == EDGE_FROM_SOURCE)
			return edge_descriptor(e.first, EDGE_TO_SOURCE);

		if (e.second == EDGE_FROM_SINK)
			return edge_descriptor(e.first, EDGE_TO_SINK);

		if (e.second == EDGE_TO_SOURCE)
			return edge_descriptor(e.first, EDGE_FROM_SOURCE);

		assert(e.second == EDGE_TO_SINK);

		/* if (e.second == EDGE_TO_SINK) */
				return edge_descriptor(e.first, EDGE_FROM_SINK);
	}

	class color_map :
			public boost::put_get_helper<boost::default_color_type,
			color_map> {
			public:
		typedef boost::lvalue_property_map_tag category;
		typedef boost::default_color_type value_type;
		typedef boost::default_color_type& reference;
		typedef vertex_descriptor key_type;

			public:
		color_map(Self& labeler) : _labeler(labeler) { }
		reference operator[](key_type v) const { return _labeler.color(v); }

			protected:
		Self& _labeler;
	};

	const boost::default_color_type& color(vertex_descriptor v) const { return _cell_colors[v + 2]; }

	boost::default_color_type& color(vertex_descriptor v) { return _cell_colors[v + 2]; }
};

} /* Polyhedral_complex_3 */

#endif /* POLYHEDRAL_COMPLEX_3_POLYHEDRAL_COMPLEX_CELLS_BINARY_LABELER_3_MINCUT_BGL_HPP */
