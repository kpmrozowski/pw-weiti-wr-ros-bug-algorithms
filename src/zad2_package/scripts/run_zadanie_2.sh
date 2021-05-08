rosservice call /spawn "{'x': 40.0, 'y': 10.0, 'theta': 0.0, 'name': "turtle1"}"

rosservice call /turtle1/teleport_absolute "{'x': 5.0, 'y': 10.0, 'theta': 1.5707}"; rosservice call /clear; rosrun zad2_package zadanie_2.py