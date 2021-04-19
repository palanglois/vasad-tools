#ifndef BIM_DATA_VOXELARRANGEMENTINLIER_H
#define BIM_DATA_VOXELARRANGEMENTINLIER_H

#include "VoxelArrangement.h"

using namespace std;

inline void updateFeatures(vector<double>& feature, int label)
{
    assert(label >= 0);
    assert(label < feature.size());
    feature[label]++;
}

inline void updateFeatures(vector<double>& feature, const vector<double>& richLabel)
{
    assert(feature.size() - 1 == richLabel.size());
    for(int i=0; i < richLabel.size(); i++)
        feature[i] += richLabel.at(i);
}

template <typename T>
int splitArrangementInVoxelsRegular(vector<facesLabelName> &labeledShapes,
                                    const vector<Point> &pointOfViews,
                                    const vector<Point> &pointCloud,
                                    const vector<T> &pointCloudLabels,
                                    double voxelSide,
                                    int nbClasses, const string &path, int nbVoxelsAlongAxis,
                                    bool withRichFeatures, bool verbose)
{

    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for(const auto& shape: labeledShapes)
        for(const auto& triangle: get<0>(shape))
            initialBbox += triangle.bbox();

    // Split it
    vector<CGAL::Bbox_3> bboxes = splitBigBbox(initialBbox, nbVoxelsAlongAxis * voxelSide);

    // Generate the chunks
    for(int i=0; i < bboxes.size(); i++)
    {
        const CGAL::Bbox_3 &curBbox = bboxes[i];
        if (verbose) {
            cout << endl << "Bbox \033[1;31m" << i << "\033[0m out of " << bboxes.size() << endl;
            cout << "Current bbox: " << curBbox << endl;
        }

        // We build the arrangement corresponding to the current bounding box
        auto fullArrangement = VoxelArrangement(curBbox, voxelSide);

        if(withRichFeatures)
        {
            // We compute the rich features
            fullArrangement.computeRichFeatures(labeledShapes, nbClasses, verbose);

            // If we have an empty chunk, we discard it
            if(fullArrangement.areRichFeaturesEmpty()) continue;
        }
        else {

            // We compute the labels
            fullArrangement.assignLabel(labeledShapes, nbClasses, verbose);

            // If we have an empty chunk, we discard it
            if (fullArrangement.isLabelEmpty()) continue;
        }

        // We compute the features
        fullArrangement.computeFeaturesRegular(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // We save the current chunk
        string outPath(path + padTo(to_string(i), 5) + ".h5");
        fullArrangement.saveAsHdf(outPath, withRichFeatures);
    }
    return bboxes.size();
}


template <typename T>
void VoxelArrangement::computeFeaturesRegular(const std::vector<Point> &points, const std::vector<Point> &pointOfViews,
                                              const std::vector<T> &labels, int nbClasses, bool verbose) {
    Simple_to_Epeck s2e;
    Epeck_to_Simple e2s;
    // Make sure that the bbox planes have been built
    computeBboxPlanes();
    // Initialize the features
    _features = vector<vector<vector<vector<double>>>>(_width,
                 vector<vector<vector<double>>>(_height,
                         vector<vector<double>>(_depth,
                                 vector<double>(nbClasses + 1, 0.))));
    vector<vector<vector<int>>> nbHits = vector<vector<vector<int>>>(_width,
                                                 vector<vector<int>>(_height,
                                                         vector<int>(_depth, 0)));

    // Histograms
    for(int i=0; i < points.size(); i++) {
        const Point &point = points[i];
        const Point &pov = pointOfViews[i];
        const T &label = labels[i];

        // Use point only if it is in the current bounding box
        if (!CGAL::do_overlap(_bbox, point.bbox())) continue;

        triplet cellIdx = findVoxel(point);
        updateFeatures(_features[get<0>(cellIdx)][get<1>(cellIdx)][get<2>(cellIdx)], label);
        nbHits[get<0>(cellIdx)][get<1>(cellIdx)][get<2>(cellIdx)]++;
    }


    // Select the visibility segments that lie in the bounding box
    vector<Point> beginPoints;
    vector<Point> endPoints;
    vector<int> validIdx = addSegmentIfInBbox(pointOfViews, points, back_inserter(beginPoints), back_inserter(endPoints), _bbox);

    // Visibility
#pragma omp parallel for
    for(int i=0; i < validIdx.size(); i++) {
        const Point &point = endPoints[i];
        const Point &pov = beginPoints[i];

        bool isPointInBbox = CGAL::do_overlap(point.bbox(), _bbox);
        vector<triplet> intersectedVoxels = intersectSegment(pov, point);
        triplet pointVoxel = make_tuple(-1, -1, -1);
        if(isPointInBbox) pointVoxel = findVoxel(point);
        for(const auto& voxel: intersectedVoxels) {
            if (voxel != pointVoxel)
#pragma omp critical
            {
                _features[get<0>(voxel)][get<1>(voxel)][get<2>(voxel)][nbClasses]++;
                nbHits[get<0>(voxel)][get<1>(voxel)][get<2>(voxel)]++;
            }
        }
    }
    if (typeid(T) == typeid(int))
        normalizeFeatures();
    else
        normalizeFeatures(nbHits);
}


template <typename T>
int splitLabeledPointCloud(const std::vector<Point> &pointOfViews,
                           const std::vector<Point> &pointCloud,
                           const std::vector<T> &pointCloudLabels,
                           double voxelSide, int nbClasses, const std::string &path, int nbVoxelsAlongAxis,
                           bool verbose)
{
    // Compute initial bounding box
    CGAL::Bbox_3 initialBbox;
    for (const auto &point: pointCloud)
        initialBbox += point.bbox();

    // Split it
    vector<CGAL::Bbox_3> bboxes = splitBigBbox(initialBbox, nbVoxelsAlongAxis * voxelSide);

    // Generate the chunks
    for(int i=0; i < bboxes.size(); i++) {
        const CGAL::Bbox_3 &curBbox = bboxes[i];
        if (verbose) {
            cout << endl << "Bbox \033[1;31m" << i << "\033[0m out of " << bboxes.size() << endl;
            cout << "Current bbox: " << curBbox << endl;
        }

        // We build the arrangement corresponding to the current bounding box
        auto fullArrangement = VoxelArrangement(curBbox, voxelSide);

        // We compute the features
        fullArrangement.computeFeaturesRegular(pointCloud, pointOfViews, pointCloudLabels, nbClasses, verbose);

        // If we have an empty chunk, we discard it
        if(fullArrangement.areFeaturesEmpty()) continue;

        // We save the current chunk
        string outPath(path + padTo(to_string(i), 4) + ".h5");
        fullArrangement.saveAsHdf(outPath);
    }
    return bboxes.size();
}


#endif //BIM_DATA_VOXELARRANGEMENTINLIER_H
