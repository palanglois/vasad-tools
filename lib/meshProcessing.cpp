#include "meshProcessing.h"

using namespace std;


void repairMesh(Polyhedron &mesh, bool verbose)
{
    // Incrementally fill the holes
    unsigned int nb_holes = 0;
    for(Halfedge_handle h : halfedges(mesh))
    {
        if(h->is_border())
        {
            vector<Facet_handle>  patch_facets;
            vector<Vertex_handle> patch_vertices;
            bool success = get<0>(
                    CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                            mesh,
                            h,
                            back_inserter(patch_facets),
                            back_inserter(patch_vertices),
                            CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)).
                                    geom_traits(Kernel())));
            if (verbose) {
                cout << " Number of facets in constructed patch: " << patch_facets.size() << endl;
                cout << " Number of vertices in constructed patch: " << patch_vertices.size() << endl;
                cout << " Fairing : " << (success ? "succeeded" : "failed") << endl;
                ++nb_holes;
            }
        }
    }
    if(verbose) {
        cout << endl;
        cout << nb_holes << " holes have been filled" << endl;
    }
}