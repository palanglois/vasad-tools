if [[ -f "./cmake-build-release/voxels2ply" ]]
then
	exec_path="./cmake-build-release/voxels2ply"
elif [[ -f "./build/voxels2ply" ]]
then
	exec_path="./build/voxels2ply"
else
	exec_path="."
fi

for infile in $1/*.h5
do
        filename="${infile##*/}"
	sem -j+0 "$exec_path -i $infile -o \"$2/${filename%.h5}.ply\"" 
done
sem --wait
