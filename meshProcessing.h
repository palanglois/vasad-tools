#ifndef BIM_DATA_MESHPROCESSING_H
#define BIM_DATA_MESHPROCESSING_H

// STD
#include <iostream>
#include <vector>

// Own
#include "iogeometry.h"

// CGAL
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

void repairMesh(Polyhedron &mesh, bool verbose=false);


#endif //BIM_DATA_MESHPROCESSING_H
