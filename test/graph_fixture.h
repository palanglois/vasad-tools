#ifndef BIM_DATA_GRAPH_FIXTURE_H
#define BIM_DATA_GRAPH_FIXTURE_H

#include "iogeometry.h"
#include "graphStats.h"

class PlaneArrangementFixture: public ::testing::Test
{
public:
    PlaneArrangementFixture();
    ~PlaneArrangementFixture() override;
    void SetUp() override;
    void TearDown() override;
protected:
    PlaneArrangement planeArrangement;
    std::map<int, int> label2cell;
    std::map<int, int> cell2label;
    std::map<int, int> label2facet;
    CGAL::Bbox_3 bbox;

};

#endif //BIM_DATA_GRAPH_FIXTURE_H
