#python version 3.10

import os
import gpxpy
from lxml import etree
from pykml.factory import KML_ElementMaker as KML
from pykml.factory import ATOM_ElementMaker as ATOM
from pykml.factory import GX_ElementMaker as GX
import simplekml as skml
import numpy as np
from math import pi,sqrt,exp,atan2

# Script parameters
# -------------------------
verbose = False
inputFile = 'input.gpx'
outputFile = 'output.kml'
model3DFile = 'A300600.dae'
modelScale = 1
viewTilt = 120
smoothFactor = 4 # (1 is no smoothing)
altitudeMode = 'absolute' #'relativeToGround' #'absolute'
extrude = 0
tessellate = 1
rad2deg = 180.0/pi
deg2rad = 1/rad2deg
location = 'Passy' #None #'Charmey'
crop_start_idx = 0 #420
crop_stop_idx = 0 #900

locations = {'Charmey': {'start': [7.2058246, 46.6251042, 1606.192797], 
                         'stop': [7.1698914, 46.6181322, 872.5266369]},
            'Passy': {'start': [6.739783, 45.949526, 1333],
                      'stop': [6.724793, 45.928321, 609]},
            'Marcel': {'start': [],
                      'stop': []}}

# 
# Functions definitions
# 

def crop_flight(pts, start=None, stop=None, loc=None):
    if start is None or stop is None:
        x = np.array([p.longitude for p in pts])
        y = np.array([p.latitude for p in pts])
        ri = (x-loc['start'][0])**2 + (y-loc['start'][1])**2
        rf = (x-loc['stop'][0])**2 + (y-loc['stop'][1])**2
        start = np.argmin(ri)
        stop = np.argmin(rf)
        print('auto start idx:', start)
        print('auto stop idx:', stop)
    new_pts = []
    for k in range(start, stop):
        new_pts.append(pts[k])
    return new_pts

def get_heading(pts, k):
    if k+1 >= len(pts):
        print(f'Warning in get_heading(): index {k} out of bounds')
        return 0
    dlat = (pts[k+1].latitude - pts[k].latitude) * deg2rad
    dlon = (pts[k+1].longitude - pts[k].longitude) * deg2rad
    heading = atan2(dlon, dlat) * rad2deg # 0 deg is North
    return np.mod(heading, 360)

def get_roll(pts, k):
    dheading = pts[k+1] - pts[k]
    dheading_max = 45
    roll = dheading / d_heading_max * 90
    return roll

def correct_altitude(pts, hi, hf):
    offset_i = pts[0].elevation - hi
    offset_f = pts[-1].elevation - hf
    offset = np.linspace(offset_i, offset_f, len(pts))
    for k in range(len(pts)):
        pts[k].elevation -= offset[k]
        pts[k].elevation = np.maximum(pts[k].elevation, 0)
    return pts

def smooth_points(pts, length=2):
    pts_sm = []
    for k in range(0, len(pts)):
        stop = np.minimum(k+length, len(pts))
        lat = np.median([p.latitude for p in pts[k:stop]])
        lon = np.median([p.longitude for p in pts[k:stop]])
        h = np.median([p.elevation for p in pts[k:stop]])
        time = pts[k].time
        pts_sm.append(gpxpy.gpx.GPXTrackPoint(lat, lon, h, time))
    return pts_sm

# 
# Parsing gpx file
# 

gpx_file = open(os.path.join('./gpx',inputFile), 'r')
gpx = gpxpy.parse(gpx_file)
gpx_file.close()

points = []
for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            if verbose:
                print(f'Point at ({point.latitude},{point.longitude}) -> {point.elevation}')
            points.append(point)

waypoints = []
for waypoint in gpx.waypoints:
    if verbose:
        print(f'waypoint {waypoint.name} -> ({waypoint.latitude},{waypoint.longitude})')
    waypoints.append(waypoint)

routes = []
for route in gpx.routes:
    print('Route:')
    for point in route.points:
        if verbose:
            print(f'Point at ({point.latitude},{point.longitude}) -> {point.elevtion}')
        routes.append(point)

kml_coords_str = ""
for point in points:
    kml_coords_str += f'{point.longitude},{point.latitude},{point.elevation}{os.linesep}'

if crop_start_idx > 0 or crop_stop_idx > 0:
    points = crop_flight(points, crop_start_idx, crop_stop_idx, locations[location])
points = smooth_points(points, smoothFactor)
if location is not None:
    points = correct_altitude(points, locations[location]['start'][2], 
                                      locations[location]['stop'][2])

#
# generating kml
#
kml = skml.Kml(name='Trajectories',open=1)

tour = kml.newgxtour(name=gpx.tracks[0].name)
playlist = tour.newgxplaylist()

model = kml.newmodel(altitudemode=altitudeMode)
model.link.href = model3DFile #os.path.join('./models', model3DFile)
model.name = 'Parapente'
model.scale.x = modelScale
model.scale.y = modelScale
model.scale.z = modelScale

flyto = playlist.newgxflyto(gxduration=0.0)
flyto.lookat.altitudemode = altitudeMode
flyto.lookat.longitude = points[0].longitude
flyto.lookat.latitude = points[0].latitude
flyto.lookat.altitude = points[0].elevation
flyto.lookat.heading = get_heading(points, 0)
flyto.lookat.range = modelScale*4.0
flyto.lookat.tilt = 0
flyto.lookat.roll = 0

print('computing Parapente kml data')
for k in range(len(points)-1):
    heading = get_heading(points, k)
    animatedupdate = playlist.newgxanimatedupdate(gxduration=0.0)
    animatedupdate.update.change = '<Orientation targetId="parapente_orientation">' + \
                                            '<heading>' + str(model.orientation.heading) + '</heading>' + \
                                            '</Orientation>'
    model.location.longitude = points[k].longitude
    model.location.latitude = points[k].latitude
    model.location.altitude = points[k].elevation
    model.orientation.heading = heading
    model.orientation.tilt = 0 #-x['aoa'][k]*rad2deg-x['gam'][k]*rad2deg
    model.orientation.roll = 0 #-x['bank'][k]*rad2deg*np.cos(-x['gam'][k])
    animatedupdate = playlist.newgxanimatedupdate(gxduration=1.0)
    #scale = x['v'][k]*(100.0/90.0)
    animatedupdate.update.change = '<Location targetId="parapente_location"> <altitude>' + \
                                       str(model.location.altitude) + '</altitude>' + \
                                       '<longitude>' + str(model.location.longitude) + '</longitude>' + \
                                       '<latitude>' + str(model.location.latitude) + '</latitude>' + \
                       '</Location>' + '<Orientation targetId="parapente_orientation">' + \
                       '<heading>' + str(model.orientation.heading) + '</heading>' + \
                       '<tilt>' + str(model.orientation.tilt) + '</tilt>' + \
                       '<roll>' + str(model.orientation.roll) + '</roll>' + '</Orientation>' + \
                       '<Scale targetId="parapente_scale">' + '<x>' + str(modelScale) + '</x>' + \
                       '<y>' + str(modelScale) + '</y>' + '<z>' + str(modelScale) + '</z>' + '</Scale>'
    flyto = playlist.newgxflyto(gxduration=1.0, gxflytomode='smooth')
    flyto.lookat.altitudemode = altitudeMode
    flyto.lookat.longitude = points[k].longitude
    flyto.lookat.latitude = points[k].latitude
    flyto.lookat.altitude = points[k].elevation
    flyto.lookat.heading =  heading #-x['bank'][k]*np.sin(-x['gam'][k])*rad2deg
    flyto.lookat.range = modelScale*4.0
    flyto.lookat.tilt = viewTilt #+(x['aoa'][k]*rad2deg+x['gam'][k]*rad2deg)/2.0
    flyto.lookat.roll = 0

#
# Create a linstring with the full trajectory
#

linestring = kml.newlinestring(name="Trajectoire")
linestring.altitudemode = altitudeMode
linestring.extrude = extrude
linestring.tessellate = tessellate
linestring.coords = [(p.longitude, p.latitude, p.elevation) for p in points]

#
# save
#
print('writing kml file')
kml.save(os.path.join('./kml',outputFile))

