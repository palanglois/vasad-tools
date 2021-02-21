#ifndef BIM_DATA_VOXELARRANGEMENT_H
#define BIM_DATA_VOXELARRANGEMENT_H

#include "highfive/H5Easy.hpp"

#include "graphStats.h"
#include <cmath>


class VoxelArrangement {
public:

    typedef std::tuple<int, int, int> triplet;
    typedef std::vector<std::vector<std::vector<std::vector<double>>>> FeatTensor;
    typedef std::vector<std::vector<std::vector<int>>> LabelTensor;

    // Constructors
    explicit VoxelArrangement(const std::string& name);
    VoxelArrangement(const CGAL::Bbox_3 &inBbox, double inVoxelSide);

    // Internal functions
    void loadJson(const std::string& name);
    void loadHdf(const std::string& name);
    void computePlanes();
    void buildArrangement();
    void normalizeFeatures();
    bool isLabelEmpty() const;

    // Count the number of cells
    int numberOfCells();

    // Finding the cell in which a point is
    triplet findVoxel(const Point &query);

    // Finding the closest facet to a query point
    int closestFacet(const Point &query);

    // Labeling the voxels
    void assignLabel(std::vector<facesLabelName> &labeledShapes, int nbClasses, bool verbose);

    // Computing the features from labeled points with their point of views (no point snapping on the facets)
    void computeFeaturesRegular(const std::vector<Point> &points, const std::vector<Point> &pointOfViews,
                                const std::vector<int> &labels, int nbClasses, bool verbose);

    // Computing the features from labeled points with their point of views (with snapping)
    void computeFeatures(const std::vector<Point> &points, const std::vector<Point> &pointOfViews,
                         const std::vector<int> &labels, int nbClasses, bool verbose);

    // Save as json
    void saveAsJson(const std::string &path);

    // Save as hdf5
    void saveAsHdf(const std::string &path);

    // Save as ply (based on ground truth labels)
    void saveAsPly(const std::string &path, const std::vector<classKeywordsColor> &classesWithColor);

    // Save a representation of the features
    void saveFeaturesAsPly(const std::string &path, const std::vector<classKeywordsColor> &classesWithColor);

    // Save raw arrangement (for debug purpose)
    void saveArrangementAsPly(const std::string &path);

    // Getters
    [[nodiscard]] const std::vector<Plane> &planes() const;
    [[nodiscard]] const std::vector<Point> &pointCloud() const;
    [[nodiscard]] const Arrangement::Plane &planeFromFacetHandle(int handle);
    [[nodiscard]] const LabelTensor &labels() const;
    [[nodiscard]] const FeatTensor &features() const;
    [[nodiscard]] double width() const;
    [[nodiscard]] double height() const;
    [[nodiscard]] double depth() const;
    [[nodiscard]] CGAL::Bbox_3 bbox() const;
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
    std::vector<Point> _pointCloud;
    bool isArrangementComputed;
    int _width;
    int _height;
    int _depth;
};


std::vector<CGAL::Bbox_3> splitBigBbox(const CGAL::Bbox_3 &bigBbox, int nbVoxelsAlongAxis, double voxelSide);

int splitArrangementInVoxels(std::vector<facesLabelName> &labeledShapes,
                             const std::vector<Point> &pointOfViews,
                             const std::vector<Point> &pointCloud,
                             const std::vector<int> &pointCloudLabels,
                             double voxelSide,
                             int nbClasses, const std::string &path, int maxNodes, bool verbose);

int splitArrangementInVoxelsRegular(std::vector<facesLabelName> &labeledShapes,
                                    const std::vector<Point> &pointOfViews,
                                    const std::vector<Point> &pointCloud,
                                    const std::vector<int> &pointCloudLabels,
                                    double voxelSide,
                                    int nbClasses, const std::string &path, int nbVoxelsAlongAxis, bool verbose);


#endif //BIM_DATA_VOXELARRANGEMENT_H
