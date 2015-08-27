
# This is the object set 1 with only one steel block
shape_db0 = {
  "rect1" : {
    "shape_poly" : [[-0.0985/2, -0.0985/2], [0.0985/2, -0.0985/2]]
  }
}

import math
# The coordinates are wrt the object frame
# The polygon shape must be in counterclock-wise order for the simplicity of finding normals
# The slot is for moving the object back

def makeShapePolyRect(longSide, shortSide):
    a = longSide / 2.0
    b = shortSide / 2.0
    return [[a,b], [-a,b], [-a,-b], [a,-b]]
    
def makeShapePolyTri(shortSide1, shortSide2, longSide):
    a = shortSide1
    b = shortSide2
    c = longSide
    d = 0.090 / 2.0  # from the rectangle coordinate system
    
    return [[d, d], [d,d-b], [d,d-a]]

def makeShapeEllip(a, b):
    return [a, b]

def makeShapePolyNGon(side, n):
    poly = []
    for i in range(n):
        theta = (2*math.pi/n)*i
        poly.append([math.cos(theta), math.sin(theta)])
    return poly

class ShapeDB:
    def __init__(self):
        self.shape_db["rect1"]["shape_poly"] = makeShapePolyRect(0.090, 0.090)
        self.shape_db["rect2"]["shape_poly"] = makeShapePolyRect(0.11258, 0.08991)
        self.shape_db["rect3"]["shape_poly"] = makeShapePolyRect(0.13501, 0.08994)
        
        self.shape_db["tri1"]["shape_poly"] = makeShapePolyTri(0.12587, 0.12590, 0.178)
        self.shape_db["tri2"]["shape_poly"] = makeShapePolyTri(0.12587, 0.15100, 0.1962)
        self.shape_db["tri3"]["shape_poly"] = makeShapePolyTri(0.12561, 0.1765, 0.2152)
        
        self.shape_db["ellip1"]["shape_ellip"] = makeShapeEllip(0.105, 0.105)
        self.shape_db["ellip2"]["shape_ellip"] = makeShapeEllip(0.13089, 0.105)
        self.shape_db["ellip3"]["shape_ellip"] = makeShapeEllip(0.157, 0.105)
        
        self.shape_db["hexagon"]["shape_poly"] = makeShapePolyNGon(0.06050, 6)
        
        # all have same thickness
        for key in self.shape_db:
            self.shape_db[key]["thickness"] = 0.013
            
        for key in self.shape_db:
            self.shape_db[key]["frame_id"] = '/vicon/StainlessSteel/StainlessSteel'
            
        for key in self.shape_db:
            self.shape_db[key]["mesh"] = 'package://pnpush_config/models/object_meshes/StainlessSteel_%s.stl' % key
            
        for key in self.shape_db:
            self.shape_db[key]["slot_pos"] = [-0.03, -0.03]

        
    shape_db = {
        "rect1" : {
            "slot_pos" : [],
            "mass" : None
        },
        "rect2" : {
            "slot_pos" : [],
            "mass" : None
        },
        "rect3" : {
            "slot_pos" : []
        },
        "tri1" : {
            "slot_pos" : []
        },
        "tri2" : {
            "slot_pos" : []
        },
        "tri3" : {
            "slot_pos" : []
        },
        "ellip1" : {
            "slot_pos" : []
        },
        "ellip2" : {
            "slot_pos" : []
        },
        "ellip3" : {
            "slot_pos" : []
        },
        "hexagon" : {
            "slot_pos" : []
        },
        # use polygon?
        "butterfly" : {
            "slot_pos" : []
        }
    }

