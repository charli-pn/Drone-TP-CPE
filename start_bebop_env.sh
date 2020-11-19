echo "starting environnement for bebop"
echo "starting control.py"
xterm -hold -e 'source devel/setup.bash; rosrun mini_projet control.py'
echo "starting altitude.py"
xterm -hold -e 'source devel/setup.bash; rosrun mini_projet altitude.py'
echo "starting rqt.py"
xterm -hold -e 'rqt'