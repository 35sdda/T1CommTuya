echo "rm ../Yuv/*.yuv"
rm ../Yuv/*.yuv
echo "t1make GetIsp ......"
cd T1GetIsp
make
echo "t1make update ......"
cd ../Update
make
echo "t1make t1one ......"
cd ..
make
echo "t1make t1Update ......"
cd Update
make
echo "t1make complete."
