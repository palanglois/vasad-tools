import argparse
from os.path import join
import ujson as json
import numpy as np

"""
Cuts a graph into chunks with step "step" along x and y axis (z is supposed to be the vertical direction)
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input json arrangement file")
    parser.add_argument("--output", required=True, help="Output folder")
    parser.add_argument("--step", type=float, default=4., help="Side size of the pillars")

    args = parser.parse_args()

    # Loading current big graph
    data = json.load(open(args.input))
    node_features = np.array(data['NodeFeatures'])
    edge_features = np.array(data['EdgeFeatures'])
    edges = np.array([x[0] for x in edge_features])
    all_points = np.array(data["NodePoints"])
    gt_labels = np.array(data["gtLabels"])
    mapping = np.array(sorted(list(data["map"].items()), key=lambda x: x[1])).astype(np.int)

    # Computing the bounding boxes
    min_pt = all_points.min(0)
    max_pt = all_points.max(0)
    x_range = [min_pt[0] + i * 4 for i in range(int((max_pt[0] - min_pt[0]) / 4) + 1)] + [max_pt[0]]
    y_range = [min_pt[1] + i * 4 for i in range(int((max_pt[1] - min_pt[1]) / 4) + 1)] + [max_pt[1]]
    bboxes = np.array([[x_range[i], y_range[j], min_pt[2], x_range[i + 1], y_range[j + 1], max_pt[2]]
                       for i in range(len(x_range) - 1) for j in range(len(y_range) - 1)])
    for i, bbox in enumerate(bboxes):
        print(f"Computed {i} bboxes out of {len(bboxes)}")
        new_data = {}
        # Retrieve the point indice which are in the current bounding box
        filter_xmin = all_points[:, 0] > bbox[0]
        filter_xmax = all_points[:, 0] < bbox[3]
        filter_ymin = all_points[:, 1] > bbox[1]
        filter_ymax = all_points[:, 1] < bbox[4]
        filter_zmin = all_points[:, 2] > bbox[2]
        filter_zmax = all_points[:, 2] < bbox[5]
        filter = filter_xmin & filter_xmax & filter_ymin & filter_ymax & filter_zmin & filter_zmax
        good_idx = np.argwhere(filter)
        old2newIdx = np.stack((np.arange(len(filter)), np.zeros(len(filter)))).T
        old2newIdx[filter, 1] = np.arange(len(good_idx))

        # Update all the attributes of the graph
        new_data["NodeFeatures"] = node_features[good_idx.reshape((-1,))].tolist()
        new_data["EdgeFeatures"] = edge_features[np.all(filter[edges], 1)].tolist()
        new_data["NodePoints"] = all_points[filter].tolist()
        new_data["Adjacency"] = old2newIdx[np.array([x[0] for x in edge_features[np.all(filter[edges], 1)]]).reshape((-1)), 1].reshape((-1, 2)).astype(np.long).tolist()
        new_data["gtLabels"] = gt_labels[filter].tolist()
        new_data["map"] = {str(x[0]): int(x[1]) for x in np.stack((mapping.T[0][filter], np.arange(len(good_idx)))).T}

        # The planes remain the same in order to properly reconstruct the meshes from the graph
        new_data["nbPlanes"] = data["nbPlanes"]
        new_data["bbox"] = data["bbox"]
        new_data["planes"] = data["planes"]

        # Output the chunk
        with open(join(args.output, f"{str(i).zfill(5)}.json"), "w") as out_file:
            json.dump(new_data, out_file)


if __name__ == "__main__":
    main()
