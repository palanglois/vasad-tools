#ifndef BIM_DATA_BIMOBJ_H
#define BIM_DATA_BIMOBJ_H

// STD
#include <map>

// CGAL
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/boost/graph/properties.h>

// Own
#include "meshProcessing.h"

// Typedefs
class SemPolyhedron : public Polyhedron {
public:
    typedef Polyhedron Base;

    SemPolyhedron(std::string _name, std::string _category, colorTuple _color);

    bool isClosed() const;
    std::string getName() const;

    Base* toCgal();


private:
    const std::string name;
    const std::string category;
    const colorTuple color;

};

/* Weird stuff you need to do to make the SemPolyhedron class work -_- (thank you CGAL) */
namespace boost {
    template<>
    struct graph_traits<SemPolyhedron>
            : public boost::graph_traits<SemPolyhedron::Base> {
    };
    template<typename T>
    struct property_map<SemPolyhedron, T>
            : public boost::property_map<SemPolyhedron::Base, T> {
    };

    template <class T>
    struct property_map<SemPolyhedron, CGAL::dynamic_vertex_property_t<T> >
            : public property_map<SemPolyhedron::Base, CGAL::dynamic_vertex_property_t<T> >
    {};

    template <class T>
    struct property_map<SemPolyhedron, CGAL::dynamic_halfedge_property_t<T> >
            : public property_map<SemPolyhedron::Base, CGAL::dynamic_halfedge_property_t<T> >
    {};

    template <class T>
    struct property_map<SemPolyhedron, CGAL::dynamic_edge_property_t<T> >
            : public property_map<SemPolyhedron::Base, CGAL::dynamic_edge_property_t<T> >
    {};

}

namespace CGAL {

    template<class Tag>
    struct graph_has_property<SemPolyhedron, Tag>
            : public graph_has_property<SemPolyhedron::Base, Tag> {
    };

    template<typename T>
    typename boost::property_map<SemPolyhedron, CGAL::dynamic_vertex_property_t<T> >::const_type
    get(const CGAL::dynamic_vertex_property_t<T> &tag, SemPolyhedron &dm) {
        return get(tag, static_cast<SemPolyhedron::Base &>(dm));
    }

    template<typename T>
    typename boost::property_map<SemPolyhedron, CGAL::dynamic_halfedge_property_t<T> >::const_type
    get(const CGAL::dynamic_halfedge_property_t<T> &tag, SemPolyhedron &dm) {
        return get(tag, static_cast<SemPolyhedron::Base &>(dm));
    }

    template<typename T>
    typename boost::property_map<SemPolyhedron, CGAL::dynamic_edge_property_t<T> >::const_type
    get(const CGAL::dynamic_edge_property_t<T> &tag, SemPolyhedron &dm) {
        return get(tag, static_cast<SemPolyhedron::Base &>(dm));
    }
}

class BimObj {
public:
    void loadFromObj(std::string path, const std::vector<classKeywordsColor> &classes);

    std::vector<SemPolyhedron>::iterator begin();
    std::vector<SemPolyhedron>::iterator end();

private:
    std::vector<SemPolyhedron> allMeshes;
};


#endif //BIM_DATA_BIMOBJ_H
