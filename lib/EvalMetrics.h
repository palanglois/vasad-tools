#ifndef BIM_DATA_EVALMETRICS_H
#define BIM_DATA_EVALMETRICS_H

#include "graphStats.h"


class EvalMetrics {
public:
    EvalMetrics(std::vector<facesLabelName> &predShapes,
                std::vector<facesLabelName> &gtShapes,
                std::vector<classKeywordsColor> &_classes,
                int nbSamplesVolumic, int nbSamplesSurfacic);

    // Volumic
    [[nodiscard]] double getIoU() const;

    [[nodiscard]] double getIoUGeometric() const;

    [[nodiscard]] std::vector<std::vector<int>> getConfusionMatrix() const;

    // Surfacic
    [[nodiscard]] double meanSurfacicDistance() const;
    [[nodiscard]] double maxSurfacicDistance() const;
    [[nodiscard]] double precision(double threshold) const;
    [[nodiscard]] double recall(double threshold) const;

    // Save
    void saveAsJson(const std::string &path, double threshold) const;

private:
    std::vector<classKeywordsColor> classes;
    CGAL::Bbox_3 globalBbox;

    // Volumic
    std::vector<Point> points;
    std::vector<int> predLabels;
    std::vector<int> gtLabels;

    // Surfacic
    std::vector<Point> predPoints;
    std::vector<Point> gtPoints;
    std::multiset<double> nnDistancesGt;
    std::multiset<double> nnDistancesPred;
};

std::vector<Point> samplePointsOnMesh(const std::vector<Triangle>& mesh, int nbSamples);
std::multiset<double> findPcDistance(const std::vector<Point>& refPointCloud, const std::vector<Point>& queryPointCloud);


#endif //BIM_DATA_EVALMETRICS_H
