# Use this script on Linux x86
# Make sure that ImageMagick and ffmpeg are installed first

convert ../*.jpg -delay 500 -morph 300 -scale 320x320 %05d.jpg
ffmpeg -i %05d.jpg -vcodec libx264 -profile:v main -pix_fmt yuv420p  -r 15 ../test.mp4
