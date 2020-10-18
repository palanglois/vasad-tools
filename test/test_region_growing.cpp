#include <gtest/gtest.h>

#include "RegionGrowing.h"

using namespace std;

TEST(RegionGrowing, PlaneClass)
{
    PlaneClass planeClass;
    Triangle triangle;
    triangle.row(0) = Point(1., 0., 0.);
    triangle.row(1) = Point(0., 1., 0.);
    triangle.row(2) = Point(1., 1., 0.);
    Point normal(0., 0., 1.);
    Point average = triangle.colwise().mean();
    int idx=0;
    planeClass.addTriangle(triangle, idx, normal, average);
    ASSERT_EQ(planeClass.getNormal(), normal);
    ASSERT_EQ(planeClass.getRefPoint(), average);
    ASSERT_EQ(planeClass.getArea(), 0.5);
}

TEST(RegionGrowing, PlaneClassFusion)
{
    //Class 1
    PlaneClass planeClassOne;
    Triangle triangle;
    triangle.row(0) = Point(1., 0., 0.);
    triangle.row(1) = Point(0., 1., 0.);
    triangle.row(2) = Point(1., 1., 0.);
    Point normal(0., 0., 1.);
    Point average = triangle.colwise().mean();
    int idx=0;
    planeClassOne.addTriangle(triangle, idx, normal, average);

    //Class 2
    PlaneClass planeClassTwo;
    Triangle triangleTwo;
    triangleTwo.row(0) = Point(1., 0., 0.);
    triangleTwo.row(1) = Point(0., 1., 0.);
    triangleTwo.row(2) = Point(0., 0., 0.);
    Point normalTwo(0., 0., 1.);
    Point averageTwo = triangleTwo.colwise().mean();
    int Two=1;
    planeClassTwo.addTriangle(triangleTwo, Two, normalTwo, averageTwo);

    // Test Merging
    planeClassOne.mergeWith(planeClassTwo);
    ASSERT_EQ(planeClassOne.getArea(), 1);
    ASSERT_EQ(planeClassOne.getNormal(), Point(0., 0., 1.));
    ASSERT_EQ(planeClassOne.getRefPoint(), Point(0.5, 0.5, 0.));
}

TEST(RegionGrowing, RegionGrowing)
{
    double epsilonPoint = 0.01;
    double epsilonNormal = 0.005;
    double sigmaPoint = 0.04;
    double sigmaNormal = 0.001;
    RegionGrowing rg((string) TEST_DIR + "cube.obj", epsilonPoint, epsilonNormal, sigmaPoint, sigmaNormal, false);
    vector<int> permutation = rg.computeTriangleOrder();
    Faces faces = rg.getFaces();
    PointCloud points = rg.getPointCloud();
    vector<double> areas;
    for(auto idx: permutation)
    {
        auto curFace = faces.row(idx);
        auto ptA = points.row(curFace(0));
        auto ptB = points.row(curFace(1));
        auto ptC = points.row(curFace(2));
        areas.push_back(0.5*(ptB - ptA).cross(ptC - ptA).norm());
    }
    for(int i=0; i < areas.size() - 1; i++)
        ASSERT_GE(areas[i], areas[i + 1]);
    rg.run();
}