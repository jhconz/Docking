# -*- coding: utf-8 -*-
"""
Created on Wed Aug 28 12:33:00 2024

@author: Student
"""
# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import regionToolset
###############################################################################
import numpy as np
import math
###############################################################################
def createShellPattern(lGeo,hGeo,nHoriz,hSizes,optRats,tBeam,s,g,v,d,c):
    lSpacing = lGeo/nHoriz
    for ii in range(np.size(hSizes,0)):
       floor = 0.0
       # Beam vertex midpoints and centerlines
       p1m = (0,hSizes[ii]/2+floor)
       p2m = p1m + ((lSpacing/2)*optRats[0],0 + floor)
       p3m = (lSpacing*optRats[2],hSizes[ii]*optRats[3] + floor)
       p4m = ((lSpacing/2)*(1-optRats[1]),0 + floor)
       p5m = (lSpacing/2,0 + floor)
       p6m = (lSpacing/2,hSizes[ii]/2 + floor)
       midLines = []
       midLines.append(s.Line(point1=p1m,point2=p2m))
       midLines.append(s.Line(point1=p2m,point2=p3m))
       midLines.append(s.Line(point1=p3m,point2=p4m))
       midLines.append(s.Line(point1=(p3m),point2=(p6m)))
       midLines.append(s.Line(point1=(p4m),point2=(p5m)))
       ## FIX ALL, MAKE CONSTRUCTION
       # Left side of pattern vertices and edges
       p1l = (p1m[0],p1m[1]-tBeam/2) #fixed
       p2l = (p2m[0],p2m[1]-tBeam/2)
       p3l = (p3m[0]-tBeam/2,p3m[1]) #fixed
       p4l = p4m - (tBeam/2,0) # fixed
       p5l = p5m #fixed
       leftEdges = []
       leftEdges.append(s.Line(point1=(p1l),point2=(p2l))) #parallel m[0] d=t/2
       leftEdges.append(s.Line(point1=(p2l),point2=(p3l))) #parallel m[1] d=t/2
       leftEdges.append(s.Line(point1=(p3l),point2=(p4l))) #paralell m[2] d=t/2
       leftEdges.append(s.Line(point1=(p4l),point2=(p5l))) #paralell m[-1] d=t/2
       ## Constrain Constrain Constrain
       # Right side of pattern vertices and edges
       p1r = p5m + (0,tBeam/2)  #fixed
       p2r = p1r - (.01*lSpacing,0) 
       p3r = p3m + math.sqrt(2)/4*(tBeam,-tBeam) 
       p4r = p6m - (0,tBeam*math.sqrt(2)/2) #fixed
       rightEdges = []
       rightEdges.append(s.Line(point1=(p1r),point2=(p2r))) #parallel m[-1] d=t/2
       rightEdges.append(s.Line(point1=(p2r),point2=(p3r))) #parallel m[2] d=t/2
       rightEdges.append(s.Line(point1=(p3r),point2=(p4r))) #parallel m[3] d=t/2
       ## Constrain Constrain Constrain
       # Upper V vertices and edges
       p1v = p2m + (tBeam/2,0) 
       p2v = p3m + (tBeam/2,0) 
       p3v = p6m - (tBeam*math.sqrt(2)/2) 
       vEdges = []
       vEdges.append(s.Line(point1=(p1v),point2=(p2v)))
       vEdges.append(s.Line(point1=(p2v),point2=(p3v)))
       ################################
       floor = floor+hSizes[ii]

###############################################################################
#Project info
PROJECT_NAME = 'dockingInstanceBuckling';
PART_NAME = PROJECT_NAME+'_part'
MODEL_NAME    = PROJECT_NAME+'_model';
ASSEMBLY_NAME = PROJECT_NAME+'_assembly';
SKETCH_NAME   = PROJECT_NAME+'_sketch';
CAE_FILE      = PROJECT_NAME+'.cae';
JOB_NAME      = PROJECT_NAME+'_job_case_'
###############################################################################    
# Important variables
path = 'C:/Users/Student.SER3034-002A/Documents/MSoRo/Docking/CAD/CubeDockSoRo/UpperAbaqusSketchBase.IGS'
lGeo = 25
hGeo = 14
nHoriz = 6
hSizes = (3.5,3.5,3.5,3.5)
optRats = (.5,.5,.25,.25) # (l_top, l_bottom, p3x, p3y) 
tBeam = 0.5
origin = (0.0,0.0)
###############################################################################
#Create an empty model
Model = mdb.Model(name=MODEL_NAME);
del mdb.models['Model-1']
###############################################################################
#Material info (Neo Hooke 1st order)
MAT_NAME = 'DragonSkin 10'
RHO_MAT = 1.07E-09
C10 = 0.0425
D1 = 0.0
###############################################################################
# Create the outer shell component
mdb.openIges(fileName=path, msbo=0, scaleFromFile=OFF, topology=WIRE,
             trimCurve=DEFAULT, uniteWires=OFF)
s = Model.ConstrainedSketchFromGeometryFile(geometryFile=mdb.acis, 
    name='UpperAbaqusSketchBase', scale=1.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
s.rectangle(point1=(-lGeo/2,0),point2=(lGeo/2,hGeo))
s.autoTrimCurve(curve1=g.findAt(origin),point1=(origin))
s.autoTrimCurve(curve1=g.findAt(origin),point1=(origin))
p = Model.Part(name='OuterShell',dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
p.BaseShell(sketch=s)
s.unsetPrimaryObject()
###############################################################################
# Create the inner geometry pattern
s = Model.ConstrainedSketch(name='InnerShell', sheetSize=200.0)
g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=STANDALONE)
createShellPattern(lGeo, hGeo, nHoriz, hSizes, optRats, tBeam, s, g, v, d, c)
