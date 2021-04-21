#include "ImplicitRepresentation.h"

using namespace std;

vector<Point> sampleInBbox(const CGAL::Bbox_3 &bbox, int nbSamples)
{

    // Draw points in the arrangement
    vector<Point> sampledPoints(nbSamples);
    default_random_engine generator;
    uniform_real_distribution<double> xDist(bbox.xmin(), bbox.xmax());
    uniform_real_distribution<double> yDist(bbox.ymin(), bbox.ymax());
    uniform_real_distribution<double> zDist(bbox.zmin(), bbox.zmax());
#pragma omp parallel for
    for(int i=0; i < nbSamples; i++)
        sampledPoints[i] = Point(xDist(generator), yDist(generator), zDist(generator));
    return sampledPoints;
}

ImplicitRepresentation::ImplicitRepresentation(const CGAL::Bbox_3 &inBbox) : bbox(inBbox) {

}

void ImplicitRepresentation::computeSurfacicFromPointCloud(const vector<Point> &pointCloud,
                                                           const vector<Vector> &normals) {

    for(int i=0; i < pointCloud.size(); i++)
    {
        if(CGAL::do_overlap(pointCloud[i].bbox(), bbox))
        {
            surfacicPoints.push_back({pointCloud[i].x(), pointCloud[i].y(), pointCloud[i].z()});
            surfacicNormals.push_back({normals[i].x(), normals[i].y(), normals[i].z()});
        }
    }

}

void ImplicitRepresentation::computeVolumicPoints(vector<facesLabelName> &labeledShapes, int nbClasses,
                                                  const vector<Point> &sampledPoints, bool verbose) {

    // Label the points
    vector<int> labels = assignLabelToPoints(sampledPoints, labeledShapes, nbClasses, bbox);

    // Concatenate the new data to the corresponding attributes
    for(int i=0; i < sampledPoints.size(); i++)
        volumicPoints.push_back({sampledPoints[i].x(), sampledPoints[i].y(), sampledPoints[i].z()});
    occupancies.insert(occupancies.end(), labels.begin(), labels.end());

}

const std::vector<std::vector<double>> &ImplicitRepresentation::getSurfacicPoints() const {
    return surfacicPoints;
}

const std::vector<std::vector<double>> &ImplicitRepresentation::getSurfacicNormals() const {
    return surfacicNormals;
}

const std::vector<std::vector<double>> &ImplicitRepresentation::getVolumicPoints() const {
    return volumicPoints;
}

const std::vector<int> &ImplicitRepresentation::getOccupancies() const {
    return occupancies;
}
