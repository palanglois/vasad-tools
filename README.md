# Useful lines: 

* `for infile in *.json; do ~/Documents/Projects/scene-semantic-recons/bim-data/cmake-build-release/graph2ply -i $infile -o "${infile%.json}.ply"; done`