import gps_util
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import cubic_spline_planner
import math

a = Point(0,0,0)
b = Point(0,15,0)
c = Point(15,15,0)

o = Point(5,5,0)
e = Point(16,10,0)

l = []
l.append(e)
l.append(c)
l.append(b)
l.append(o)
l.append(a)

x = []
y = []

for p in l:
    x.append(p.x)
    y.append(p.y)

plt.scatter(x,y)
#plt.ion()
#plt.show()

li = gps_util.add_intermediate_points(l, 5.0)

ax = []
ay = []

for p in li:
    ax.append(p.x)
    ay.append(p.y)


f2 = plt.figure()
plt.scatter(ax, ay)
plt.ion()
plt.show()


#================================================================ SPLINE =========================

goal = [ax[-1], ay[-1]]

cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
    ax, ay, ds=0.1)
target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

#sp = calc_speed_profile(cx, cy, cyaw, target_speed)

#t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal)


#plt.close()
flg, _ = plt.subplots(1)
plt.plot(ax, ay, "xb", label="input")
plt.plot(cx, cy, "-r", label="spline")
plt.plot(x, y, "-g", label="tracking")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()


flg, ax = plt.subplots(1)
plt.plot(s, [math.degrees(iyaw) for iyaw in cyaw], "-r", label="yaw")
plt.grid(True)
plt.legend()
plt.xlabel("line length[m]")
plt.ylabel("yaw angle[deg]")



plt.show()


while (1):
    pass
