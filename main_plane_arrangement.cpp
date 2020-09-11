#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

// External
#include "OptionParser/option_parser.h"

// CGAL
#include <boost/optional/optional_io.hpp>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Timer.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

//Polyhedral complex
#include <Polyhedral_complex_3/Arrangement_3.hpp>
#include <Polyhedral_complex_3/Mesh_3.hpp>
#include <Polyhedral_complex_3/Mesh_extractor_3.hpp>
#include <Polyhedral_complex_3/print_PLY.hpp>
#include "Polyhedral_complex_3/Polyhedral_complex_queries_3.hpp"

// Ours
#include "iogeometry.h"

using namespace std;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

typedef std::pair<Point, Vector>         Point_with_normal;
typedef std::vector<Point_with_normal>                       Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits
        <Kernel, Pwn_vector, Point_map, Normal_map>             Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>            Plane;

typedef vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<Kernel, Iterator> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
typedef Polyhedral_complex_3::Arrangement_3<Kernel2> Arrangement;

struct Timeout_callback {
    mutable int nb;
    mutable CGAL::Timer timer;
    const double limit;
    explicit Timeout_callback(double limit) :
            nb(0), limit(limit) {
        timer.start();
    }
    bool operator()(double advancement) const {
        // Avoid calling time() at every single iteration, which could
        // impact performances very badly.
        ++nb;
        if (nb % 1000 != 0)
            return true;
        // If the limit is reached, interrupt the algorithm.
        if (timer.time() > limit) {
            std::cerr << "Algorithm takes too long, exiting ("
                      << 100.0 * advancement << "% done)" << std::endl;
            return false;
        }
        return true;
    }
};

void saveArrangementAsPly(const string& fileName, Polyhedral_complex_3::Arrangement_3<Kernel2> &arr, bool verbose=true) {
    // Mark all facet to be drawn
    for (auto itf = arr.facets_begin(); itf != arr.facets_end(); itf++) {
        Polyhedral_complex_3::Arrangement_3<Kernel2>::Face &f = *itf;
        f.to_draw = false;
        if (!arr.is_facet_bounded(f)) { continue; }
        f.to_draw = true;
    }

    // Save the drawn faces
    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Polyhedral_complex_3::Arrangement_3<Kernel2>, Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(arr);
    extractorGC.extract(meshGC, false);
    {
        std::ofstream stream(fileName.c_str());
        if (!stream.is_open())
            return;
        Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }

    if (verbose)
        cout << "Saved the plane arrangement at path : " << fileName << endl;
}

void sort_intersections(vector<Point> &intersectPoints)
{
    if(intersectPoints.size() <= 1) return;
    Vector refSegment(intersectPoints[0], intersectPoints[1]);
    vector<pair<size_t, double>> allDistances(intersectPoints.size());
    for(int i=0; i < intersectPoints.size(); i++)
        allDistances[i] = make_pair(i, refSegment * Vector(intersectPoints[0], intersectPoints[i]));
    struct ordering {
        bool operator ()(pair<size_t, double> const& a, pair<size_t, double> const& b) {
            return (a.second) < (b.second);
        }
    };
    sort(allDistances.begin(), allDistances.end(), ordering());

    vector<Point> ret(intersectPoints.size());
    size_t const size = intersectPoints.size();
    for (size_t i = 0; i < size; ++i)
        ret[i] = intersectPoints[allDistances[i].first];
    intersectPoints = ret;
}

void savePlyFromLabel(const string &filename, Arrangement &arr, map<Arrangement::Face_handle, int> &fh_to_node, const vector<bool> &labels)
{

    for(auto itf = arr.facets_begin(); itf != arr.facets_end(); itf++){
        Arrangement::Face& f = *itf;
        itf->to_draw = false;
        if(! arr.is_facet_bounded(f)){continue;}
        Arrangement::Face_handle ch0 = f.superface(0), ch1 = f.superface(1);
        //if(!(is_cell_bounded(ch0) && is_cell_bounded(ch1))){continue;}
        if(fh_to_node.count((int)ch0) ==0 || fh_to_node.count((int)ch1) == 0){continue;}
        if(labels[fh_to_node[int(ch0)]] != labels[fh_to_node[int(ch1)]]){
            f.to_draw = true;
        }
    }

    typedef Polyhedral_complex_3::Mesh_3<> Mesh;
    typedef Polyhedral_complex_3::Mesh_extractor_3<Arrangement,Mesh> Extractor;
    Mesh meshGC;
    Extractor extractorGC(arr);
    extractorGC.extract(meshGC,false);
    {
        std::ofstream stream(filename.c_str());
        if (!stream.is_open())
            return ;
        Polyhedral_complex_3::print_mesh_PLY(stream, meshGC);
        stream.close();
    }
}


int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input obj point cloud", "");
    opt.add_option("-pov", "--pointofview", "Path to the pov obj file", "");
    opt.add_option("-t", "--timout", "Timeout for RANSAC", "60");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if(opt["-i"].empty())
    {
        cerr << "Input file (-i) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string povPath = opt["-pov"];
    double timout = op::str2double(opt["-t"]);
    const int maxNumberOfPlanes = 160;

    // Load file
    cout << "Loading point cloud..." << endl;
    vector<Point> pointCloud = loadPointCloudObj(inputPath);
    vector<Point> pov = loadPointCloudObj(povPath);
    Pwn_vector pwnCloud;
    const int debugLimit = 100000;
    int iter = 0;
    for(auto point: pointCloud)
    {
#ifndef NDEBUG
        // debug code
        iter++;
        if(iter >= debugLimit)
            break;
#endif
        pwnCloud.emplace_back(Point_with_normal(point, Vector(0., 0., 0.)));
    }
    cout << "Loaded " << pwnCloud.size() << " points." << endl;

    cout << "Estimating normals..." << endl;
    const int nb_neighbors = 10;
    CGAL::pca_estimate_normals<Concurrency_tag>(pwnCloud, nb_neighbors,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Point_with_normal>()).
        normal_map(CGAL::Second_of_pair_property_map<Point_with_normal>()));
    cout << "Normals estimated." << endl;

    // Shape detection engine.
    cout << "Running RANSAC..." << endl;
    Efficient_ransac ransac;
    ransac.set_input(pwnCloud);
    ransac.add_shape_factory<Plane>();
    Timeout_callback timeout_callback(timout);
    Efficient_ransac::Parameters parameters;
    parameters.min_points = 10;
    parameters.epsilon = 0.08;
    ransac.detect(parameters, timeout_callback);
    cout << ransac.shapes().end() - ransac.shapes().begin()
         << " shapes detected." << endl;

    cout << "Building plane arrangement..." << endl;
    Arrangement arrangement;
    CGAL::Bbox_3 bbox = CGAL::bbox_3(pointCloud.begin(), pointCloud.end());
    bbox += CGAL::bbox_3(pov.begin(), pov.end());

    arrangement.set_bbox(bbox);
    vector<Plane*> shapes;
    for(const auto & planeIt : ransac.shapes())
        shapes.push_back(dynamic_cast<Plane*>(planeIt.get()));
    auto it = shapes.begin();
    Simple_to_Epeck s2e;
    int planeIter = 0;
    sort(shapes.begin(), shapes.end(),
         [](auto & a, auto & b) -> bool
         {
             return a->indices_of_assigned_points().size() > b->indices_of_assigned_points().size();
         });
    while (it != shapes.end()) {
            arrangement.insert(s2e(Kernel::Plane_3(**it)));
            if(planeIter++ == maxNumberOfPlanes) break;
        it++;
    }
    cout << "Plane arrangement built !" << endl;
    cout << "Saving arrangement..." << endl;
    saveArrangementAsPly("arrangement.ply", arrangement);

    // Prepare labels and mapping
    vector<bool> labels(arrangement.number_of_cells(), true);
    map<Arrangement::Face_handle, int> cell2label;
    int cellIter = 0;
    for(auto cellIt = arrangement.cells_begin(); cellIt != arrangement.cells_end(); cellIt++)
        cell2label[arrangement.cell_handle(*cellIt)] = cellIter++;

    //Prepare intersection tree
    cout << "Building intersection tree..." << endl;
    std::vector<Triangle> triangles;
    for (auto itf = arrangement.facets_begin(); itf != arrangement.facets_end(); itf++) {

        Arrangement::Face &f = *itf;
        Arrangement::Face_handle fh = arrangement.facet_handle(f);

        // test if facet on the bounding box
        if (!arrangement.is_facet_bounded(fh)) { continue; }

        // get the vertices of the facet
        std::vector<Arrangement::Face_handle> vertices;
        arrangement.facet_to_polygon(f, std::back_inserter(vertices));
        int num_hs = vertices.size();

        // be sur it is a triangle
        if (num_hs < 3) { continue; }

        // create the triangles
        // here there is a strong assumption that f is convex
        std::vector<Arrangement::Face_handle>::const_iterator vhi = vertices.begin();
        const Arrangement::Point& first = arrangement.point(*vhi);
        ++vhi;
        Arrangement::Point second = arrangement.point(*vhi);
        ++vhi;
        while (vhi != vertices.end()) {
            const Arrangement::Point& third = arrangement.point(*vhi);
            ++vhi;

            Point a(CGAL::to_double(first.x()), CGAL::to_double(first.y()), CGAL::to_double(first.z()));
            Point b(CGAL::to_double(second.x()), CGAL::to_double(second.y()),
                                   CGAL::to_double(second.z()));
            Point c(CGAL::to_double(third.x()), CGAL::to_double(third.y()), CGAL::to_double(third.z()));

            second = third;
            if (CGAL::collinear(a, b, c)) {
                continue;
            }

            Triangle tr(a, b, c);
            triangles.push_back(tr);
        }
    }
    Tree tree(triangles.begin(),triangles.end());
    cout << "Intersection tree build." << endl;

    cout << "Computing intersections and labeling corresponding cells..." << endl;
    // using a random subset of size 400k (30min computation)
    vector<int> indices(pointCloud.size());
    iota(indices.begin(), indices.end(), 0);
    const int seed = 0;
    const int limit = 400000;
    shuffle(indices.begin(), indices.end(), default_random_engine(seed));
    for(int k = 0; k < limit; k++)
    {
        int i = indices[k];
        if(k % 1000 == 0)
            cout << "Computed " << k << " out of " << limit << " rays." << endl;
        Vector dir = pov[i] - pointCloud[i];
        dir /= sqrt(dir.squared_length());
        Point endPoint = pov[i] - 0.01*dir;
        Segment query(pointCloud[i], endPoint);
        vector<Segment_intersection> intersections;
        vector<Point> intersectPoints;
        tree.all_intersections(query, back_inserter(intersections));
        for(auto &intersection: intersections)
            if(intersection)
                if(boost::get<Point>(&(intersection->first)))
                    intersectPoints.push_back(*boost::get<Point>(&(intersection->first)));
        sort_intersections(intersectPoints);
        for(int j = 0; j < int(intersectPoints.size()) - 1; j++)
        {
            Point queryPoint = intersectPoints[j] + (intersectPoints[j + 1] - intersectPoints[j]) / 2.;
            Arrangement::Face_handle cellHandle = find_containing_cell(arrangement, s2e(queryPoint));
            labels[cell2label[cellHandle]] = false;
        }
    }
    cout << "Labeling done !" << endl;
    int nbTrue = 0;
    for(auto label: labels)
        nbTrue += label;
    cout << "Number of true labels: " << nbTrue << " out of " << labels.size() << endl;

    cout << "Saving reconstruction..." << endl;
    savePlyFromLabel("filtered_plane_arrangement.ply", arrangement, cell2label, labels);
    cout << "Reconstruction saved." << endl;

    return 0;
}
