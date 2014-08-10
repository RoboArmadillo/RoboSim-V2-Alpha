"""
vpyode/__init__.py
A module containing extensions to the PyODE + VisualPython Libraries.

Written By:
	James Thomas
	Email: jt@missioncognition.net
	Web: http://missioncognition.net/

Copyright 2009

This file is part of the VisualPyODE package.

This library is free software; you can redistribute it and/or
modify it under the terms of EITHER:
  (1) The GNU Lesser General Public License as published by the Free
      Software Foundation; either version 2.1 of the License, or (at
      your option) any later version. The text of the GNU Lesser
      General Public License is included with this library in the
      file LICENSE.
  (2) The BSD-style license that is included with this library in
      the file LICENSE-BSD.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
LICENSE and LICENSE-BSD for more details. 

Note: See http://pyode.sourceforge.net and http://vpython.org 
      to get the PyODE and VisualPython libraries which is required to run this package.
"""

import ode
import visual
import odelib

# Collision callback

def near_callback(Args, Geom1, Geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """

    # Check if the objects do collide
    contacts = ode.collide(Geom1, Geom2)

    # Create contact joints
    world, contactGroup = Args
    for contact in contacts:
        contact.setBounce(0.1)
        contact.setMu(0.1)
        joint = ode.ContactJoint(world, contactGroup, contact)
        joint.attach(Geom1.getBody(), Geom2.getBody())

_bigSpace = ode.Space()

class World(ode.World):
    """An enhanced ODE world class that has a display and manages collisions."""
    def __init__(self):
        ode.World.__init__(self)
        self._myScene = visual.display()

        # A joint group for the contact joints that are generated whenever
        # two bodies collide
        self._contactGroup = ode.JointGroup()

    def step(self, DeltaTime):
        """Performs a step with collision handling."""
        # Detect collisions and create contact joints
        _bigSpace.collide((self, self._contactGroup), near_callback)

        # Simulation step
        ode.World.step(self, DeltaTime)

        # Remove all contact joints
        self._contactGroup.empty()

    # Use for now until I can wrap all the misc scene stuff
    def _getScene(self):
        """Get the VPython scene associated with this world."""
        return self._myScene

    def Select(self):
        """Select this world's scene for display."""
        self._myScene.select()

    def KBHit(self):
        """Returns true if the keyboard has been hit in this world's scene."""
        return self._myScene.kb.keys > 0

    def GetKey(self):
        """Gets a character (in VPython format) hit in this world's scene."""
        return self._myScene.kb.getkey()

    def GetAllKeys(self):
        """Gets a list of characters (in VPython format) hit in this world's scene."""
        return [self._myScene.kb.getkey() for n in range(self._myScene.kb.keys)]

class Mesh(ode.TriMeshData):
    """A mesh object based on ode's TriMesh and VPythons convex display element."""
    def __init__(self):
        ode.TriMeshData.__init__(self)
        self._verticies = []
        self._colors = []
        self._normals = []

    def build(self, Verticies, Faces, Colors=None, Normals=None, IsSoft=False):
        """Build a displayable tri-mesh object.  This is an overload of ode.TriMeshData.build()."""
        # Fill out our TriMeshData object properties by calling the parent
        ode.TriMeshData.build(self, Verticies, Faces)

        self._verticies = []
        self._colors = []
        self._normals = []

        # Expand the mesh to make it suitable for use with Visual
        for face in Faces:
            for index, vertex in enumerate(face):
                self._verticies.append(Verticies[vertex])
                if Colors is not None:
                    self._colors.append(Colors[vertex])
                else:
                    # Assign a default
                    self._colors.append(visual.color.white)
                if Normals is not None:
                    self._normals.append(Normals[vertex])
                else:
                    # Compute the normals using cross products
                    normal = visual.norm(visual.cross(Verticies[face[index]], Verticies[face[(index + 1) % 3]]))
                    self._normals.append(normal)

        if IsSoft:
            # Need to go back and average normals -- implement later
            pass

    def MakeMeshObject(self):
        """Generate a VPython display object based on the tri-mesh properties."""
        #return visual.faces(pos=self._Verticies, colors=self._Colors, normals=self._Normals)
        return visual.convex(pos=self._verticies)

class DisplayElement:
    """A Display-only element class -- no physics properties"""
    def __init__(self, DisplayObject=None):
        """Optionally set the VPython display object that will be associated with this display element."""
        self._display = DisplayObject

    def SetVisible(self, IsVisible=None):
        """Set the visibility of the VPython display object. True=visible, False=not visible, None=toggle.
        By default the visibility is toggled (IsVisible=None)."""
        if IsVisible is not None:
            self._display.visible = IsVisible
        else: # Toggle if None
            self._display.visible = not self._display.visible

    def GetDisplayObject(self):
        """Get the VPython display object associated with this display element."""
        return self._display

    def SetDisplayObject(self, Object):
        """Set the VPython display object that will be associated with this display element."""
        self._display = Object
        return self

class GDMElement(DisplayElement):
    """A display element with physics properties and optionally a collision Geom."""
    def __init__(self, HasGeom=True):
        DisplayElement.__init__(self)

        self._geom = None
        self._hasGeom = HasGeom
        self._mass = None

    def SetDisplayObject(self, Object):
        """This is an overload to prevent the user from calling in this class. Throws an assertion."""
        assert False  # Don't use this function with this class

    def HasGeom(self):
        """Returns True if this element is set to have a Geom"""
        return self._hasGeom

    def GetGeom(self):
        """Get the geom assocaiated with this element."""
        return self._geom
    
    def GetMass(self):
        """Get the mass of this element."""
        return self._mass
    
    def DefineBox(self, Density, SizeX, SizeY, SizeZ, colour, position = (0,0,0)):
        """Define this element as a ode Box."""
        if self._hasGeom:
            self._geom = ode.GeomBox(_bigSpace, (SizeX, SizeY, SizeZ)) # BigSpace was None, problem with local body spaces

        DisplayElement.SetDisplayObject(self, visual.box(length=SizeX, height=SizeY, width=SizeZ, color = colour, pos=position))

        self._mass = ode.Mass()
        self._mass.setBox(Density, SizeX, SizeY, SizeZ)
        return self

    def DefineBoxTotal(self, TotalMass, SizeX, SizeY, SizeZ):
        """Define this element as an ode BoxTotal."""
        if self._hasGeom:
            self._geom = ode.GeomBox(_bigSpace, (SizeX, SizeY, SizeZ))

        DisplayElement.SetDisplayObject(self, visual.box(length=SizeX, height=SizeY, width=SizeZ))

        self._mass = ode.Mass()
        self._mass.setBoxTotal(TotalMass, SizeX, SizeY, SizeZ)
        self._mass.mass = TotalMass # Bug workaround?
        return self

    def DefineSphere(self, Density, Radius):
        """Define this element as an ode Sphere."""
        if self._hasGeom:
            self._geom = ode.GeomSphere(None, Radius)

        DisplayElement.SetDisplayObject(self, visual.sphere(radius=Radius))

        self._mass = ode.Mass()
        self._mass.setSphere(Density, Radius)
        return self

    def DefineSphereTotal(self, TotalMass, Radius):
        """Define this element as an ode SphereTotal."""
        if self._hasGeom:
            self._geom = ode.GeomSphere(None, Radius)

        DisplayElement.SetDisplayObject(self, visual.sphere(radius=Radius))

        self._mass = ode.Mass()
        self._mass.setSphereTotal(TotalMass, Radius)
        self._mass.mass = TotalMass # Bug workaround?
        return self

    def DefineCylinder(self, Density, Radius, Length):
        """Define this element as an ode Cylinder."""
        if self._hasGeom:
            self._geom = ode.GeomCylinder(None, Radius, Length)

        cyl = visual.cylinder(pos=(0, 0, -Length / 2.0), axis=(0, 0, Length), radius=Radius)
        DisplayElement.SetDisplayObject(self, cyl)

        self._mass = ode.Mass()
        self._mass.setCylinderTotal(Density, 3, Radius, Length) # 3 is z-axis -- same as geom
        return self

    def DefineCylinderTotal(self, TotalMass, Radius, Length):
        """Define this element as an ode CylinderTotal."""
        if self._hasGeom:
            self._geom = ode.GeomCylinder(None, Radius, Length)

        cyl = visual.cylinder(pos=(0, 0, -Length / 2.0), axis=(0, 0, Length), radius=Radius)
        DisplayElement.SetDisplayObject(self, cyl)

        self._mass = ode.Mass()
        self._mass.setCylinderTotal(TotalMass, 3, Radius, Length) # 3 is z-axis -- same as geom
        self._mass.mass = TotalMass # Bug workaround?
        return self

    def DefineMeshTotal(self, TotalMass, CenterOfMass, InertiaMatrix, Mesh):
        """Define this element as a mesh.  The Mesh must be a vpyode.Mesh class object.  The
        cg and IntertiaMaxtrix are defined in world coordinates."""
        if self._hasGeom:
            self._geom = ode.GeomTriMesh(Mesh, None)

        DisplayElement.SetDisplayObject(self, Mesh.MakeMeshObject())

        # Setup the mass properties
        self._mass = ode.Mass()
        cgx, cgy, cgz = CenterOfMass
        (I11, I12, I13), (I21, I22, I23), (I31, I32, I33) = InertiaMatrix
        self._mass.setParameters(TotalMass, cgx, cgy, cgz, I11, I22, I33, I12, I13, I23)
        return self

class GDMFrameBody(odelib.BaseBody):
    """A class which defines an ode-like body which has GDM elements and 
    a VPython display fram assocaiated with it."""
    def __init__(self, World):
        # Initialize the Body class
        odelib.BaseBody.__init__(self, World)

        self._elementDict = {}
        #self._mySpace = ode.Space()

        # Add the space to the world's space
        #_bigSpace.add(self._MySpace)

        # Create a visual frame to hold display objects
        self._myFrame = visual.frame()

        # Mass not defined yet
        self._myNetMass = None

    # Overload the setPosition function so we can make sure that the display frame matches
    def setPosition(self, Position):
        """Set the position of the body in world coordinates (the display object is moved to match)."""
        odelib.BaseBody.setPosition(self, Position)
        self._myFrame.pos = Position

    # Overload the setQuaternion function so we can make sure that the display frame matches
    def setQuaternion(self, Quaternion):
        """Set the orientation of the body in world coordinates (the display object is oriented to match)."""
        odelib.BaseBody.setQuaternion(self, Quaternion)

        # now set the orientation of the display frame
        # Rotating a point is q * (0,v) * q-1
        # q-1 is w, -x, -y, -z assuming that q is a unit quaternion
        w1, x1, y1, z1 = Quaternion
        v1 = visual.vector(x1, y1, z1)
        w3 = w1
        v3 = -v1

        # First do the axis vector
        w2 = 0.0
        v2 = visual.vector((1.0, 0.0, 0.0))

        # This code is equivalent to a quaternion multiply: qR = q1 * q2 * q3
        w12 = (w1 * w2) - visual.dot(v1, v2)
        v12 = (w1 * v2) + (w2 * v1) + visual.cross(v1, v2)
        wR = (w12 * w3) - visual.dot(v12, v3)
        vR = (w12 * v3) + (w3 * v12) + visual.cross(v12, v3)

        self._myFrame.axis = vR

        # Do it again for the up vector
        w2 = 0.0
        v2 = visual.vector((0.0, 1.0, 0.0))

        # This code is equivalent to a quaternion multiply: qR = q1 * q2 * q3
        w12 = (w1 * w2) - visual.dot(v1, v2)
        v12 = (w1 * v2) + (w2 * v1) + visual.cross(v1, v2)
        wR = (w12 * w3) - visual.dot(v12, v3)
        vR = (w12 * v3) + (w3 * v12) + visual.cross(v12, v3)

        self._myFrame.up = vR

    def AddGDMElement(self, ElementKey, Element):
        """Add a GDM element to this body and use ElementKey as a handle to it."""
        assert ElementKey not in self._elementDict  # Not ready for replaces yet
        #assert type(Element) == GDMElement

        self._elementDict[ElementKey] = Element

        if Element.HasGeom():
            # Register the geom with our space and the body
            #self._MySpace.add(Element.GetGeom())
            Element.GetGeom().setBody(self)

        # Put this element into our display frame
        Element.GetDisplayObject().frame = self._myFrame

        if self._myNetMass is not None:
            # Incorporate this mass into the cumulative mass for the frame
            self._myNetMass.add(Element.GetMass())
        else:
            self._myNetMass = Element.GetMass()

        # Assign the mass to our body.  Use the base class call
        # since we are blocading use of the function at this class level
        odelib.BaseBody.setMass(self, self._myNetMass)

    def AddDisplayObject(self, ElementKey, DisplayObject):
        """Add a display-only element to the body using ElementKey as the handle.
        This is useful for orientation arrows or other non-physical display features."""
        assert ElementKey not in self._elementDict  # Not ready for replaces yet

        # Create a new display element to hold the object
        element = DisplayElement(DisplayObject)

        self._elementDict[ElementKey] = element

        # Put this element into our display frame
        element.GetDisplayObject().frame = self._myFrame

    def GetElementKeys(self):
        """Returns a list of element keys associated with this body."""
        return self._elementDict.keys()

    def SetElementVisible(self, ElementKey, IsVisible):
        """Set the visibility of the specified Element.  True=visible, False=not visible, None=toggle."""
        assert ElementKey in self._elementDict
        self._elementDict.get(ElementKey).SetVisible(IsVisible)

    def RemoveElement(self, ElementKey):
        """Remove (and destroy) the specified element from the body."""
        assert ElementKey in self._elementDict
        element = self._elementDict.pop(ElementKey)

        # Make the visual feature invisible and unlink it from the frame
        element.SetVisible(False)
        element.GetDisplayObject().frame = None

        if element.HasGeom():
            # Unlink the Geom from its body disable it and remove it from it's space
            element.GetGeom().setBody(None)
            element.GetGeom().disable()
            self._mySpace.remove(element.GetGeom())

        # Recompute the mass using the remaining features
        self._myNetMass.setZero()
        for element in self._elementDict.itervalues():
            self._myNetMass.add(element.GetMass())
        self.setMass(self._myNetMass)

    def GetFeature(self, ElementKey=None):
        """Returns the display object associated with the specified element key.  Returns
        the display frame object if the key is None."""
        if ElementKey in self._elementDict:
            return self._elementDict.get(ElementKey).GetDisplayObject()
        else:
            return self._myFrame

    def GetSpace(self):
        """Returns the ode collision space associated with this body."""
        return self._mySpace

    def GetElementWorldLocation(self, ElementKey):
        """Returns the location of the specified element in world coordinates."""
        assert ElementKey in self._elementDict
        self._elementDict.get(ElementKey).SetVisible(isVisible)
        localPos = self.GetElement(ElementKey).pos
        xAxis = visual.norm(self._myFrame.axis)
        zAxis = visual.norm(visual.cross(self._myFrame.axis, self._myFrame.up))
        yAxis = visual.norm(visual.cross(z_axis, x_axis))
        worldPos = self._myFrame.pos + (localPos.x * xAxis) + (localPos.y * yAxis) + (localPos.z * zAxis)
        return worldPos

    def SetVisible(self, IsVisible=None):
        """Sets the visibility for every element associated with the body.
        True=visible, False=not visible, None=toggle.  Note: the toggle is element by element."""
        for key in self._elementDict:
            self.SetElementVisible(key, IsVisible)

    def UpdateDisplay(self):
        """Sync's the VPython display with the bodies ode physics state."""
        # If we reset position and quaternion it will sync the display
        self.setPosition(self.getPosition())
        self.setQuaternion(self.getQuaternion())

    # Functions that should not be used anymore
    def setMass(self, M):
        """An override to prevent use of this function.  Throws an assertion if called.
        Assign GDMElements to the body to for it to have mass."""
        assert False # Do not use this function anymore

    def setRotation(self, R):
        """An override to prevent use of this function.  Throws an assertion if called.
        This could be implemented in the future if somebody wants to figure out how to do it.
        You have to use setQuaternion() instead -- sorry."""
        assert False # Do not use this function anymore

class GDMFrameAssembly(odelib.Assembly):
    """An assembly of GDMFrameBodies.  This is needed to support picking display objects."""
    def __init__(self, World):
        # Initialize the odelib.Assembly class
        odelib.Assembly.__init__(self, World)

        self._featureMap = {} # a map of display object to body key

    def AddBody(self, BodyKey, Body):
        """Add a body to the assembly using BodyKey for future reference.  BodyKey can be
        any hashable type usable as a dict key."""
        odelib.Assembly.AddBody(self, BodyKey, Body)

        # Add the body to our lookup by visual feature map
        self._featureMap[Body.GetFeature()] = BodyKey

    def SetBodyVisible(self, BodyKey, isVisible):
        assert BodyKey in self._bodyDict
        self._bodyDict.get(BodyKey).SetVisible(isVisible)

    def RemoveBody(self, BodyKey):
        """Remove the body specified by BodyKey from the assembly.  The key must exist."""
        body = self.GetBody(BodyKey)

        # Make the visual feature invisible
        body.SetVisible(False)

        self._featureMap.pop(body.GetFeature(), None)

        return odelib.Assembly.RemoveBody(self, BodyKey)

    def GetFeature(self, BodyKey, FeatureKey=None):
        """Return the specified display feature for the specified body.  If FeatureKey is None then
        the display frame associated with the body is returned."""
        assert BodyKey in self._bodyDict

        return self._bodyDict.get(BodyKey).GetFeature(FeatureKey)

    def Pick(self, PickObject):
        """Determine which body the VPython pick object is associated with."""
        try:
            pick = PickObject.frame
        except AttributeError:
            pick = PickObject

        return self._featureMap.get(pick)

    def UpdateDisplay(self):
        """Make the display for all of the bodies in this assembly consistent with the ode physics.""" 
        for body in self._bodyDict.itervalues():
            # Update the body Display
            body.UpdateDisplay()

