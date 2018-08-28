#Various use cases:
#
# 1. Live camera input, using 2xEVE and 2xDSP cores, based on model with single layers group 
./tidl_classification -g 1 -d 2 -e 2 -l ./imagenet.txt -s ./classlist.txt -i 1 -c ./stream_config_j11_v2.txt
# 2. Use video clip as input stream, using 2xEVE and 2xDSP cores, based on model with single layers group 
./tidl_classification -g 1 -d 2 -e 2 -l ./imagenet.txt -s ./classlist.txt -i ./clips/test50.mp4 -c ./stream_config_j11_v2.txt
# 3. Use video clip as input stream, using 2xEVE and 1xDSP cores, based on model with two layers group (1st layers group running on EVE, 2nd layers group on DSP)
./tidl_classification -g 2 -d 1 -e 2 -l ./imagenet.txt -s ./classlist.txt -i ./clips/test50.mp4 -c ./stream_config_j11_v2.txt
# 4. Use video clip as input stream, using no EVEs and 2xDSP cores, based on model with single layers group
./tidl_classification -g 1 -d 2 -e 0 -l ./imagenet.txt -s ./classlist.txt -i ./clips/test50.mp4 -c ./stream_config_j11_v2.txt
