#ifndef BIM_DATA_VOXELARRANGEMENT_H
#define BIM_DATA_VOXELARRANGEMENT_H

#include "graphStats.h"


class VoxelArrangement {
public:

    typedef std::tuple<int, int, int> triplet;
    typedef std::vector<std::vector<std::vector<std::vector<double>>>> FeatTensor;
    typedef std::vector<std::vector<std::vector<int>>> LabelTensor;

    // Constructors
    explicit VoxelArrangement(const std::string& name);
    VoxelArrangement(const CGAL::Bbox_3 &inBbox, double inVoxelSide);

    // Internal functions
    void computePlanes();
    void buildArrangement();

    // Finding the closest facet to a query point
    int closestFacet(const Arrangement::Point &query);

    // Labeling the voxels
    void assignLabel(std::vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose);

    // Computing the features from labeled points with their point of views
    void computeFeatures(const std::vector<Point> &points, const std::vector<Point> &pointOfViews,
                         const std::vector<int> &labels, int nbClasses, bool verbose);

    // Getters
    [[nodiscard]] const std::vector<Plane> &planes() const;
    [[nodiscard]] const Arrangement::Plane &planeFromFacetHandle(int handle) const;
    [[nodiscard]] const LabelTensor &labels() const;
    [[nodiscard]] const FeatTensor &features() const;
private:
    Arrangement _arr;
    CGAL::Bbox_3 _bbox;
    double _voxelSide;
    std::map<int, triplet> _node2index;
    std::map<triplet, int> _index2node;
    std::map<int, std::vector<int>> _node2facets;
    FeatTensor _features;
    LabelTensor _labels;
    std::vector<Plane> _planes;
    bool isArrangementComputed;
    int _width;
    int _height;
    int _depth;
};


#endif //BIM_DATA_VOXELARRANGEMENT_H
