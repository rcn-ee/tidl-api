# Live camera input
./tidl_classification -n 2 -t e -l labels.txt -i 1 -s ./classlist.txt -c ./stream_config_j11_v2.txt
# Use video clip as input stream
./tidl_classification -n 2 -t e -l labels.txt -i ./clips/test1.mp4 -s ./classlist.txt -c ./stream_config_j11_v2.txt
