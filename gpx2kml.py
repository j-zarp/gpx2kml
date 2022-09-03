#python version 3.10

import os
import gpxpy
from lxml import etree
from pykml.factory import KML_ElementMaker as KML
from pykml.factory import ATOM_ElementMaker as ATOM
from pykml.factory import GX_ElementMaker as GX

# Script parameters
# -------------------------
verbose = False
inputFile = 'input.gpx'
outputFile = 'output.kml'
altitudeMode = 'absolute'
altitudeOffset = 0
extrude = 1
tesselate = 1

# Parsing an existing file:
# -------------------------

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
    kml_coords_str += f'{point.longitude},{point.latitude},{point.elevation + altitudeOffset}{os.linesep}'

doc = KML.kml(
  etree.Comment(' required when using gx-prefixed elements '),
  KML.Placemark(
    KML.name('gx:altitudeMode Example'),
    KML.LookAt(
      KML.longitude('146.806'),
      KML.latitude('12.219'),
      KML.heading('-60'),
      KML.tilt('70'),
      KML.range('6300'),
      GX.altitudeMode(altitudeMode),
    ),
    KML.LineString(
      KML.extrude(extrude),
      GX.altitudeMode(altitudeMode),
      KML.coordinates(
        kml_coords_str
      ),
    ),
  ),
)
doc_str = etree.tostring(etree.ElementTree(doc), pretty_print=True).decode('UTF-8')

with open(os.path.join('./kml',outputFile), 'w') as kml_file:
    if verbose:
        print(doc_str)
    kml_file.write(doc_str)

