lat0 = 60
lon0 = 60
h0 = 0
lat = 60.004
lon = 60
h = 100

v = [0 1 0]
teta = pi/2
vec = [1, 0, 0]
q = [cos(teta/2), vec(1)*sin(teta/2),  vec(2)*sin(teta/2), vec(3)*sin(teta/2)]
wgs84 = wgs84Ellipsoid;

tr_v = quatrotate(q, v)
[xEast, yNorth, zUp] = geodetic2enu(lat, lon, h, lat0, lon0, h0, wgs84)
xNorth = yNorth
yWest = -xEast

x = xNorth - zUp * tr_v(1)/tr_v(3)
y = yWest  - zUp * tr_v(2)/tr_v(3)

[lat1, lon1, h1] = enu2geodetic(-yWest, xNorth, 0, lat0, lon0, h0, wgs84)







