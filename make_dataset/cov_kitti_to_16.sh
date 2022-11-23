input_path=$1
output_path=$2
mkdir -p ${output_path}
cp ${input_path}/{calib.txt,poses.txt,times.txt} ${output_path} || echo "未能成功复制"
./build/make_dataset_make_dataset --i=${input_path}  --o=${output_path}