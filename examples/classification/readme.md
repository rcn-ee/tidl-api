# Live camera input
./tidl -n 2 -t e -i 1 -s ./classlist.txt -c ./stream_config_j11_v2.txt
# Use video clip as input stream
./tidl -n 2 -t e -i ./clips/test1.mp4 -s ./classlist.txt -c ./stream_config_j11_v2.txt
