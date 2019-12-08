# Script to detect number of available EVEs, DSPs, and CMEM memory
export numeve=$(/usr/share/ti/examples/opencl/platforms/platforms  | grep "Embedded Vision Engine" | wc -l)
export numdsp=$(find /proc/device-tree/ocp/ -name "dsp_system*" | wc -l)
export cmemsize=$(cat /proc/cmem | grep "Block 0: Pool 0:" | cut -d ' ' -f8)
echo "Number of EVEs:${numeve}"
echo "Number of DSPs:${numdsp}"
echo "CMEM size:${cmemsize}"
