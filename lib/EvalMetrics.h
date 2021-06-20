#ifndef BIM_DATA_EVALMETRICS_H
#define BIM_DATA_EVALMETRICS_H

#include "graphStats.h"


class EvalMetrics {
public:
    EvalMetrics(std::vector<facesLabelName> &predShapes,
                std::vector<facesLabelName> &gtShapes,
                std::vector<classKeywordsColor> &_classes,
                int nbSamples);

    [[nodiscard]] double getIoU() const;

    [[nodiscard]] std::vector<std::vector<int>> getConfusionMatrix() const;

private:
    std::vector<classKeywordsColor> classes;
    CGAL::Bbox_3 globalBbox;
    std::vector<Point> points;
    std::vector<int> predLabels;
    std::vector<int> gtLabels;
};


#endif //BIM_DATA_EVALMETRICS_H
