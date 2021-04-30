// External
#include "OptionParser/option_parser.h"

// Own
#include "ImplicitRepresentation.h"

using namespace std;

int main(int argc, char *argv[]) {
    op::OptionParser opt;
    opt.add_option("-h", "--help", "show option help");
    opt.add_option("-i", "--input", "Path to the input npz file", "");
    opt.add_option("-o", "--output", "Path to the output obj file", "");
    opt.add_option("-v", "--verbose", "Verbosity trigger");

    //Parsing options
    bool correctParsing = opt.parse_options(argc, argv);
    if (!correctParsing)
        return EXIT_FAILURE;

    if (op::str2bool(opt["-h"])) {
        opt.show_help();
        return 0;
    }

    if (opt["-i"].empty()) {
        cerr << "Input file (-i) required !" << endl;
        return EXIT_FAILURE;
    }

    if (opt["-o"].empty()) {
        cerr << "Output file (-o) required !" << endl;
        return EXIT_FAILURE;
    }

    const string inputPath = opt["-i"];
    const string outputPath = opt["-o"];
    bool verbose = op::str2bool(opt["-v"]);

    // Load semantic_classes
    vector<classKeywordsColor> classesWithColor = loadSemanticClasses((string) TEST_DIR + "semantic_classes.json");
    if (verbose)
        for (int i = 0; i < classesWithColor.size(); i++)
            cout << "Class " << i << " is " << get<0>(classesWithColor[i]) << endl;

    // Load the npz file
    cnpy::npz_t fullNpz = cnpy::npz_load(inputPath);
    cnpy::NpyArray arrBoxes = fullNpz["boxes"];
    double *boxes = arrBoxes.data<double>();
    cnpy::NpyArray arrSemantic = fullNpz["semantic"];
    double *semantic = arrSemantic.data<double>();

    // Make original bounding box
    CGAL::Bbox_3 bbox(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);
    vector<Eigen::Vector3f> bboxPoints = {
            Eigen::Vector3f(bbox.xmin(), bbox.ymin(), bbox.zmin()),
            Eigen::Vector3f(bbox.xmin(), bbox.ymin(), bbox.zmax()),
            Eigen::Vector3f(bbox.xmin(), bbox.ymax(), bbox.zmin()),
            Eigen::Vector3f(bbox.xmin(), bbox.ymax(), bbox.zmax()),
            Eigen::Vector3f(bbox.xmax(), bbox.ymin(), bbox.zmin()),
            Eigen::Vector3f(bbox.xmax(), bbox.ymin(), bbox.zmax()),
            Eigen::Vector3f(bbox.xmax(), bbox.ymax(), bbox.zmin()),
            Eigen::Vector3f(bbox.xmax(), bbox.ymax(), bbox.zmax()),
    };

    // Make the boxes
    vector<Triangle> triangles;
    TriangleClassMap triangleClasses;
    for (int i = 0; i < arrBoxes.shape[0]; i++) {
        vector<double> curBox;
        bool isNan = false;
        for (int j = 0; j < arrBoxes.shape[1]; j++) {
            double curElem = boxes[i * arrBoxes.shape[1] + j];
            if(curElem != curElem) isNan = true;
            curBox.push_back(boxes[i * arrBoxes.shape[1] + j]);
        }
        if(isNan) continue;
        if (curBox[0] == -1) continue;
        Eigen::Quaternionf q(curBox[3], curBox[4], curBox[5], curBox[6]);
        Eigen::Matrix3f mat = q.normalized().toRotationMatrix();
        Eigen::Vector3f scale(curBox[0], curBox[1], curBox[2]);
        Eigen::Vector3f translation(curBox[7], curBox[8], curBox[9]);

        cout << "Mat: " << mat << endl;
        cout << "Scale: " << scale << endl;
        cout << "Translation: " << translation << endl;

        vector<Point> targetBoxPoints;
        for (const auto &point: bboxPoints) {
            Eigen::Vector3f targetPoint = mat * scale.cwiseProduct(point) + translation;
            if(targetPoint.norm() > 10) {
                cout << "Target point: " << targetPoint << endl;
                isNan = true;
            }
            targetBoxPoints.emplace_back(targetPoint(0), targetPoint(1), targetPoint(2));
        }
        if(isNan) continue;
        vector<Triangle> bboxTriangles = {
                Triangle(targetBoxPoints[0], targetBoxPoints[1], targetBoxPoints[5]),
                Triangle(targetBoxPoints[0], targetBoxPoints[5], targetBoxPoints[4]),
                Triangle(targetBoxPoints[4], targetBoxPoints[5], targetBoxPoints[6]),
                Triangle(targetBoxPoints[6], targetBoxPoints[5], targetBoxPoints[7]),
                Triangle(targetBoxPoints[1], targetBoxPoints[3], targetBoxPoints[5]),
                Triangle(targetBoxPoints[5], targetBoxPoints[3], targetBoxPoints[7]),
                Triangle(targetBoxPoints[0], targetBoxPoints[4], targetBoxPoints[2]),
                Triangle(targetBoxPoints[2], targetBoxPoints[4], targetBoxPoints[6]),
                Triangle(targetBoxPoints[0], targetBoxPoints[2], targetBoxPoints[1]),
                Triangle(targetBoxPoints[1], targetBoxPoints[2], targetBoxPoints[3]),
                Triangle(targetBoxPoints[2], targetBoxPoints[7], targetBoxPoints[3]),
                Triangle(targetBoxPoints[2], targetBoxPoints[6], targetBoxPoints[7]),
        };

        for(const auto& triangle: bboxTriangles)
        {
            triangles.push_back(triangle);
            triangleClasses[triangle] = semantic[i];
        }

    }
    saveTrianglesAsObj(triangles, outputPath, triangleClasses, classesWithColor);

    return 0;
}