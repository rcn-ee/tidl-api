# Set of TIDL benchmarking test cases for AM5729 SoC, with 2xDSP and 4xEVE
export TIDL_NETWORK_HEAP_SIZE_EVE=56623104
export TIDL_NETWORK_HEAP_SIZE_DSP=56623104
./mcbench -g 1 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_j11_v2.txt -f 50 -i ../test/testvecs/input/preproc_0_224x224_multi.y
./mcbench -g 1 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_j11_v2_dense.txt -f 50 -i ../test/testvecs/input/preproc_0_224x224_multi.y

export TIDL_NETWORK_HEAP_SIZE_EVE=67108864
export TIDL_NETWORK_HEAP_SIZE_DSP=8388608
./mcbench -g 2 -d 1 -e 4 -c ../test/testvecs/config/infer/tidl_config_mobileNet1_lg2.txt -f 50 -i ../test/testvecs/input/preproc_2_224x224_multi.y
./mcbench -g 2 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_mobileNet1_lg2.txt -f 50 -i ../test/testvecs/input/preproc_2_224x224_multi.y
./mcbench -g 2 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_mobileNet2_lg2.txt -f 50 -i ../test/testvecs/input/preproc_2_224x224_multi.y
./mcbench -g 2 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_inceptionNetv1_lg2.txt -f 50 -i ../test/testvecs/input/preproc_0_224x224_multi.y
./mcbench -g 2 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_j11_v2_lg2.txt -f 50 -i ../test/testvecs/input/preproc_0_224x224_multi.y
./mcbench -g 2 -d 2 -e 4 -c ../test/testvecs/config/infer/tidl_config_j11_v2_dense_lg2.txt -f 50 -i ../test/testvecs/input/preproc_0_224x224_multi.y
