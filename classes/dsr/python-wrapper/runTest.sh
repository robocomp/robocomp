python DSRGetID.py &
pid=$! 
echo "DSRGetID.py launched with pid $pid."
sleep 1s
echo "Starting tests."
python3  PythonAPITests.py
echo "killing DSRGetID.py."
kill -9 $pid
echo "Done."