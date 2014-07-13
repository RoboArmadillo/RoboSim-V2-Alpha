"""
odelib/__init__.py
A module containing extensions to the PyODE Library.

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

from math import pi, cos, sin
import ode
import numpy as np
import math

#
# Define some 3d vector operations on tuples
# This is simply to avoid a dependency on a big library like numpy
# It is too bad that pyode does not include these already.
# The routines that use these probably are not called so often as to care
# about the speed penalty. These can be used in a pinch by a user of this lib.
#

def rotateVector(m1,v1):
    '''Rotate a vector using a rotation matrix'''
    v=np.array([v1[0],v1[1],v1[2]])
    m=np.array([[m1[0],m1[1],m1[2]],[m1[3],m1[4],m1[5]],[m1[6],m1[7],m1[8]]])
    newv = np.dot(m,v)
    return newv


def vsum(v1, v2):
    """Perform a sum of two 3D vectors.  v = v1 + v2"""
    x1, y1, z1 = v1
    x2, y2, z2 = v2
    return (x1 + x2, y1 + y2, z1 + z2)

def vdiff(v1, v2):
    """Perform a difference os two 3D vectors.  v = v1 - v2"""
    x1, y1, z1 = v1
    x2, y2, z2 = v2
    return (x1 - x2, y1 - y2, z1 - z2)

def sprod(s, v):
    """Scalar product with a 3D Vector.  vScaled = s * v"""
    x, y, z = v
    return (x * s, y * s, z * s)

def dot(v1, v2):
    """Perform a dot product of two 3D vectors.  dotprod = v1 dot v2"""
    x1, y1, z1 = v1
    x2, y2, z2 = v2
    return x1 * x2 + y1 * y2 + z1 * z2

def cross(v1, v2):
    """Perform a cross product of two 3D vectors.  vCross = v1 x v2"""
    x1, y1, z1 = v1
    x2, y2, z2 = v2
    return (y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2)

def RotatePoint(Point, Quaternion):
    """Rotate a 3D point by a rotation quaternion (must be normalized)"""
    w, x, y, z = Quaternion

    # A point is rotated by the following:  vNew = Q * vOld * Q^-1
    q1 = MultiplyQuaternions((0.0,) + Point, Quaternion)
    
    # The inverse of a quaternion is its conjugate divided by its norm.
    norm = w * w + x * x + y * y + z * z
    qInv = (w / norm, -x / norm, -y / norm, -z / norm)
    
    # Do the second half of the multiply
    q2 = MultiplyQuaternions(q1, qInv)

    return q2[1:] # The rotated point is in the 'vector portion' of the quaternion.

def MultiplyQuaternions(Q1, Q2):
    """Multiply two quaternions together.  qNew = q1 * q2"""
    w1, x1, y1, z1 = Q1
    v1 = (x1, y1, z1)
    w2, x2, y2, z2 = Q2
    v2 = (x2, y2, z2)
            
    # This code is equivalent to a quaternion multiply: qNew = q1 * q2
    wNewOrientation = (w1 * w2) - dot(v1, v2)
    vNewOrientation = vsum(vsum(sprod(w1, v2), sprod(w2, v1)), cross(v1, v2))
    return (wNewOrientation,) + vNewOrientation
  
def RotateQuaternion(Angle, Axis, Quaternion):
    """Rotate a quaternion by an angle about an axis."""
    w1 = cos(Angle / 2.0)
    v1 = sprod(sin(Angle / 2.0), Axis)

    # Rotate by using a quaternion multiply:  qNewOrientation = qRotate * qOldOrientation
    return MultiplyQuaternions((w1,) + v1, Quaternion)

#
# Create a Rotational Actuator class derived from the ode HingeJoint.  This class provides
# all the hooks necessary to have a fully featured joint without a lot of work by the user.
#

class RotationalActuator(ode.HingeJoint):
    """ Can behave like a spring hinge, rotational damper, motor, or
    a combination of all 3."""
    def __init__(self, World):
        # Initialize the HingeJoint class
        ode.HingeJoint.__init__(self, World)

        # Initialize the properties to benign values
        self._k = 0.0
        self._lockedTorqueLimit = 1.0e99
        self._staticFrictionTorque = 0.0
        self._kineticFrictionTorque = 0.0 * self._staticFrictionTorque
        self._frictionTol = 0.0001

        self._externalTorque = 0.0

        self._isLocked = False

        # Set the angular limits
        self.setParam(ode.ParamLoStop, -ode.Infinity)
        self.setParam(ode.ParamHiStop, ode.Infinity)

        # Make the stops non bouncy
        self.setParam(ode.ParamBounce, 0.0)

        # Set the target motor rate to 0 radians/sec
        self.setParam(ode.ParamVel, 0.0)

        # Set the motor to the static friction torque
        self.setParam(ode.ParamFMax, self._staticFrictionTorque)

    def GetProperties(self):
        properties = (self._k, self._lockedTorqueLimit, self._staticFrictionTorque,
                        self._kineticFrictionTorque, self._frictionTol)

        return properties

    def ChangeProperties(self, SpringConstant=None, LockedTorqueLimit=None,
                            StaticFrictionTorque=None, KineticFrictionTorque=None,
                            FrictionTol=None):
        # Make changes to specified properties
        if SpringConstant is not None:
            self._k = SpringConstant
        if LockedTorqueLimit is not None:
            self._lockedTorqueLimit = LockedTorqueLimit
        if StaticFrictionTorque is not None:
            self._staticFrictionTorque = StaticFrictionTorque
        if KineticFrictionTorque is not None:
            self._kineticFrictionTorque = KineticFrictionTorque
        if FrictionTol is not None:
            self._frictionTol = FrictionTol

    def GetExternalTorque(self):
        """Get the external torque currently being applied to the hinge joint."""
        return self._externalTorque

    def SetExternalTorque(self, Torque):
        """Set the external torque to apply to the hinge joint.  This is applied
        along with hinge and frictional torques every time UpdateSettings() is called."""
        self._externalTorque = Torque

    def UpdateSettings(self):
        """This function must be called before each time step is executed.  It computes
        and updates the torque applied to the joint.  It also handles joint locking."""
        #print self._Locked
        #print self.GetProperties()

        if self._isLocked:
            self.setParam(ode.ParamFMax, self._lockedTorqueLimit)
        else:
            # Set the motor to the static friction torque
            self.setParam(ode.ParamFMax, self._staticFrictionTorque)

        # Compute spring torque
        angle = self.getAngle()
        springTorque = -self._k * angle

        # Compute kinetic friction torque
        angularVelocity = self.getAngleRate()
        if angularVelocity < -self._frictionTol:
            kineticFrictionTorque = self._kineticFrictionTorque
        elif angularVelocity > self._frictionTol:
            kineticFrictionTorque = -self._kineticFrictionTorque
        else:
            kineticFrictionTorque = 0.0

        sumTorques = springTorque + kineticFrictionTorque + self._externalTorque
        #print sumTorques, SpringTorque, KineticFrictionTorque, self._ExternalTorque

        self.addTorque(sumTorques)

    def Lock(self):
        """Lock the joint so it will not rotate."""
        self._isLocked = True

    def Unlock(self):
        """Unlock the joint so it will rotate freely."""
        self._isLocked = False

#
# Create a Linear Actuator class derived from the ode SliderJoint.  This class provides
# all the hooks necessary to have a fully featured joint without a lot of work by the user.
#

class LinearActuator(ode.SliderJoint):
    """ Can behave like a spring coil, shock absorber, linear position actuator,
    a combination of all 3. It provides no rotational constraints at all."""
    def __init__(self, World):
        # Initialize the LinearActuator class
        ode.SliderJoint.__init__(self, World)

        # Initialize the properties to benign values
        self._minSpringForce = 0.0
        self._maxSpringForce = 0.0

        self._lockedForceLimit = 1.0e99
        self._staticFrictionForce = 0.0
        self._kineticFrictionForce = 0.0
        self._frictionTol = 0.0001

        self._externalForce = 0.0
        self._isLocked = False

        self._connectionIDs = [None, None]

        # Set the linear limits
        self.setParam(ode.ParamLoStop, -ode.Infinity)
        self.setParam(ode.ParamHiStop, ode.Infinity)

        # Make the stops non bouncy
        self.setParam(ode.ParamBounce, 0.0)

        # Set the target motor rate to 0 m/sec
        self.setParam(ode.ParamVel, 0.0)

        # Set the motor to the static friction force
        self.setParam(ode.ParamFMax, self._staticFrictionForce)

    def GetProperties(self):
        properties = (self._minSpringForce, self._maxSpringForce, self._lockedForceLimit,
                        self._staticFrictionForce, self._kineticFrictionForce, self._frictionTol,
                        self.getParam(ode.ParamLoStop), self.getParam(ode.ParamHiStop))

        return properties

    def ChangeProperties(self, MinSpringForce=None, MaxSpringForce=None, LockedForceLimit=None,
                            StaticFrictionForce=None, KineticFrictionForce=None,
                            FrictionTol=None, MinStop=None, MaxStop=None):
        # Make changes to specified properties
        if MinSpringForce is not None:
            self._minSpringForce = MinSpringForce
        if MaxSpringForce is not None:
            self._maxSpringForce = MaxSpringForce
        if LockedForceLimit is not None:
            self._lockedForceLimit = LockedForceLimit
        if StaticFrictionForce is not None:
            self._staticFrictionForce = StaticFrictionForce
        if KineticFrictionForce is not None:
            self._kineticFrictionTorque = KineticFrictionForce
        if FrictionTol is not None:
            self._frictionTol = FrictionTol
        if MinStop is not None:
            self.setParam(ode.ParamLoStop, MinStop)
        if MaxStop is not None:
            self.setParam(ode.ParamHiStop, MaxStop)

    def GetExternalForce(self):
        """Get the external force currently being applied to the slider joint."""
        return self._externalForce

    def SetExternalForce(self, Force):
        """Set the external force to apply to the slider joint.  This is applied
        along with spring and frictional forces every time UpdateSettings() is called."""
        self._externalForce = Force

    def UpdateSettings(self): # Called before the next time step
        """This function must be called before each time step is executed.  It computes
        and updates the forces applied to the joint.  It also handles joint locking."""
        if self._isLocked:
            self.setParam(ode.ParamFMax, self._lockedForceLimit)
        else:
            # Set the motor to the static friction torque
            self.setParam(ode.ParamFMax, self._staticFrictionForce)

        # Compute spring force
        pos = self.getPosition()
        minPos = self.getParam(ode.ParamLoStop)
        maxPos = self.getParam(ode.ParamHiStop)
        if pos < minPos:
            springForce = self._minSpringForce
        elif pos > maxPos:
            springForce = self._maxSpringForce
        else:
            springForce = self._minSpringForce + (self._maxSpringForce - self._minSpringForce) * (pos - minPos)

        # Compute kinetic friction force
        linearVelocity = self.getPositionRate()

        if linearVelocity < -self._frictionTol:
            kineticFrictionForce = self._kineticFrictionForce
        elif linearVelocity > self._frictionTol:
            kineticFrictionForce = -self._kineticFrictionForce
        else:
            kineticFrictionForce = 0.0

        self.addForce(springForce + kineticFrictionForce + self._externalForce)

        #print self.GetProperties()

    def Lock(self):
        """Lock the joint so it will not slide."""
        self._isLocked = True

    def Unlock(self):
        """Unlock the joint to allow it to slide."""
        self._isLocked = False

#
# Create a Base Body object class derived from the ode Body.  This class provides
# all the hooks necessary to have a fully featured Body without a lot of work by the user.
#

class BaseBody(ode.Body):
    def __init__(self, World):
        # Initialize the Body class
        ode.Body.__init__(self, World)

        # Remember the world so we don't need it passed everywhere
        self._world = World

        # Initialize the connections as none defined.
        # The connections can be made later by calling the appropriate functions.
        self._connections = {} # Use a dictionary so we don't have to use an index for ID
        self._connectionLocations = {}

    def RotateOrientation(self, Angle, Axis):
        """Rotate the body orientation by the angle about the specified axis (must be unit vector).
        This rotation does not result any any translation of the body location.
        The axis is specified in the world coordinate system and the angle is in radians."""
        qNew = RotateQuaternion(Angle, Axis, self.getQuaternion())
        self.setQuaternion(qNew)

    def RotateAboutPoint(self, Point, Quaternion):
        """Rotates the body about the specified point by the rotation quaternion (normalized).
        the location of the body is moved as a result of the rotation.
        The point and quaternion are specified in the world coordinate system."""
        # Get the body position relative to the Point
        relPos = vdiff(self.getPosition(), Point)

        # Rotate the cg-relative position by the quaternion
        newRelPos = RotatePoint(relPos, Quaternion)
        
        # Move the body to the new absolute location
        self.setPosition(vsum(newRelPos, Point))
   
        # A rotation is equivalent to a quaternion multiply: qNewOrientation = qRotate * qOldOrientation
        qNew = MultiplyQuaternions(Quaternion, self.getQuaternion())
        
        # Set the new orientation
        self.setQuaternion(qNew)

    def MoveByDelta(self, Delta):
        """Translate the body position by Delta 3-tuple vector."""
        self.setPosition(vsum(self.getPosition(), Delta))

    def MoveToPlaceConnectionAt(self, ConnectionID, Position):
        """Move the body such that the specified connection point is located at the specified
        3-tuple position vector.  The position is specified in world coordinates."""
        assert ConnectionID in self._connectionLocations

        # Get the location of the specified connection point.
        location = self.GetConnectionLocation(ConnectionID)

        # Calculate the delta between the new position and the connectoin point
        delta = vdiff(Position, location)

        # Move the body
        self.MoveByDelta(delta)

    def DefineConnectionPoint(self, ConnectionID, Position, Axis=None):
        """Define a connection point on the body for use in placing joints.  The position
        is a 3-tuple vector in world coordinates.  The connection point can have an optional
        axis (specified in world coordinates) which will be used by a joint that requires
        an orientation axis."""
        # For now we cannot deal with 'moving' a location
        assert ConnectionID not in self._connectionLocations

        # Convert the position to local coordinates before storing
        localPos = self.getPosRelPoint(Position)

        if Axis is not None:
            localAxis = self.vectorFromWorld(Axis)
        else:
            localAxis = None

        self._connectionLocations[ConnectionID] = (localPos, localAxis)

    def Connect(self, JointInstance, ConnectionID, OtherBody, OtherConnectionID):
        """Make a connection using the specified joint instance between the specified connection
        on this body and the specified connection on the other body."""
        assert ConnectionID in self._connectionLocations
        assert ConnectionID not in self._connections

        if OtherBody is not None:
            assert OtherConnectionID in OtherBody._connectionLocations
            assert OtherConnectionID not in OtherBody._connections

            # Make the connection.
            JointInstance.attach(self, OtherBody)
        else:
            # If OtherBody is None then it attaches to the environment
            JointInstance.attach(self, ode.environment)

        # Get the information for this joint location
        location = self.GetConnectionLocation(ConnectionID)
        axis = self.GetConnectionAxis(ConnectionID)

        # Set the location of the joint to the prepared location
        try:
            JointInstance.setAnchor(location)
        except AttributeError:
            pass

        # Set the joint axis ODE will normalize it for us if needed
        try:
            # If you get a TypeError exception for NoneType then you forgot
            # to define the axis for this joint when calling DefineConnectionPoint()
            JointInstance.setAxis(axis)
        except AttributeError:
            pass

        # Put the joint object into this object and the other objects's connection lists
        self._connections[ConnectionID] = JointInstance
        if OtherBody is not None:
            OtherBody._connections[OtherConnectionID] = JointInstance

    def IsConnected(self, ConnectionID):
        """Returns True if the specified connectionID has a joint connected."""
        return ConnectionID in self._connections

    def GetConnectionJoint(self, ConnectionID):
        """Gets the joint instance for the specified connection.  Use IsConnected() to 
        make sure the connection has a joint before calling this function."""
        assert ConnectionID in self._connections

        if ConnectionID in self._connections:
            # Get the joint instance for this connection
            return self._connections[ConnectionID]
        else:
            return None

    def GetConnectionIDFromJoint(self, JointInstance):
        """Call this function when you have the joint instance but don't know where it is
        connected on the body.  returns None if joint is not connected on this body.""" 
        keys = [key for key, value in self._connections.items() if value == JointInstance]
        if keys:
            connectionID = keys[0]
        else:
            connectionID = None
        return connectionID

    def Disconnect(self, ConnectionID):
        """Disconnect the joint at this connection and any connection on another body.
        The joint instance will be lost if it is not held elsewhere.  Use IsConnected() to 
        make sure the connection has a joint before calling this function."""
        # Get the joint instance
        jointInstance = self.GetConnectionJoint(ConnectionID)

        # Figure out the other body
        otherBody = jointInstance.getBody(0)
        if otherBody == self:
            otherBody = jointInstance.getBody(1)

        # Figure out the connection number on the other body
        otherConnectionID = otherBody.GetConnectionIDFromJoint(jointInstance)

        if otherConnectionID is not None:
            # Disconnect the other body
            otherBody._Connections.pop(otherConnectionID)

        # Disconnect the joint
        self._connections[ConnectionID].attach(None, None)

        # Now disconnect ourselves
        self._connections.pop(ConnectionID)

    def SetFixed(self):
        """Fix this body to the ode world using a FixedJoint."""
        self._fixedJoint = ode.FixedJoint(self._world)
        self._fixedJoint.attach(self, ode.environment)
        self._fixedJoint.setFixed()

    def ReleaseFixed(self):
        """Release a fixed joint from SetFixed() if one was specified for this body."""
        try:
            del self._fixedJoint
        except AttributeError:
            pass

    def GetConnectionLocation(self, ConnectionID):
        """Get the connection point location in world coordinates."""
        assert ConnectionID in self._connectionLocations
        return self.getRelPointPos(self._connectionLocations[ConnectionID][0])

    def GetConnectionIDs(self):
        """Get a list of the IDs of any specified connections."""
        return self._connections.keys()

    def GetConnectionAxis(self, ConnectionID):
        """Get the axis of a connection in world coordinates."""
        assert ConnectionID in self._connectionLocations
        localAxis = self._connectionLocations[ConnectionID][1]
        if localAxis is not None:
            return self.vectorToWorld(localAxis)
        else:
            return None

    def GetMassCGAndInertia(self):
        """Gets the mass, cg, and intertia matrix of the body."""
        massObj = self.getMass()
        return self.massObj.mass, self.getRelPointPos(self.massObj.c), self.massObj.I

    def UpdateSettings(self, TimeNow=None):
        """This function must be called before each time step is executed.  It will
        call the UpdateSettings() function for any connected joints that have it."""
        # Update the joints
        for key, jointInstance in self._connections.iteritems():
            # This may seem convoluted but I do it this way on purpose.
            # When I originally called the function in the try: the exception
            # was catching errors in the implementation of the update function.
            # This is made it a pain to debug so now I do it in two passes.
            updateFunc = None
            try:
                updateFunc = jointInstance.UpdateSettings
            except AttributeError:
                # The ode joints don't have an UpdateSettings function
                pass

            # If updateFunc is still None then obviously it didn't exist
            if updateFunc is not None:
                updateFunc()

#
# Create an Assembly class so we can manage collections of bodies that form one object.
#

class Assembly:
    def __init__(self, World):
        self._world = World
        self._bodyDict = {}

    def AddBody(self, BodyKey, Body):
        """Add a body to the assembly using BodyKey for future reference.  BodyKey can be
        any hashable type usable as a dict key."""
        assert BodyKey not in self._bodyDict  # Not ready for replaces yet
        self._bodyDict[BodyKey] = Body

    def GetBodyKeys(self):
        """Get a list of body key values of bodies in the assembly."""
        return self._bodyDict.keys()

    def IsIn(self, BodyObject):
        """Return True if the body instance is part of this assembly.  Use GetBodyKeys if
        you have the body by key and want to know if it is in the assembly."""
        return BodyObject in self._bodyDict.values()

    def RemoveBody(self, BodyKey):
        """Remove the body specified by BodyKey from the assembly.  The key must exist."""
        assert BodyKey in self._bodyDict
        body = self._bodyDict.pop(BodyKey)

        # Disconnect this body from everything
        for connectionID in body.GetConnectionIDs():
            body.Disconnect(connectionID)

        return body

    def GetBody(self, BodyKey):
        """Get the body instance associated with BodyKey"""
        assert BodyKey in self._bodyDict
        return self._bodyDict.get(BodyKey)

    def GetBodyPosition(self, BodyKey):
        """Get the body position in world coordinates for the body associated with BodyKey"""
        return self.GetBody(BodyKey).getPosition()

    def GetBodyQuaternion(self, BodyKey):
        """Get the orientation quaternion (in world coordinates) for the body associated with BodyKey"""
        return self.GetBody(BodyKey).getQuaternion()

    def SetBodyPosition(self, BodyKey, Position):
        """Set the position of the body associated with BodyKey (Position is specified in world coordinates)."""
        self.GetBody(BodyKey).setPosition(Position)

    def SetBodyQuaternion(self, BodyKey, Quaternion):
        """Set the orientation of the body associated with BodyKey (Quaternion
        is specified in world coordinates)."""
        self.GetBody(BodyKey).setQuaternion(Quaternion)

    def GetMassAndCG(self):
        """Get the mass and cg of the assembly returned as mass, cg (3-tuple vector)."""
        sumCGMoments = (0.0, 0.0, 0.0)
        sumMass = 0.0
        for body in self._bodyDict.itervalues():
            m, cg, I = body.GetMassCGAndInertia()
            # Add the mass from this body
            sumMass += m

            # Add the moment from this body
            sumCGMoments = vsum(sumCGMoments, sprod(m, cg))

        return sumMass, sprod(1.0 / sumMass, sumCGMoments)

    def MoveCGTo(self, NewPosition):
        """Move the entire assembly so that it's cg is located at NewPosition (3-tuple vector
        in world coordinates)"""
        m, cg = self.GetMassAndCG()

        # Calculate the delta between the new position and the current CG
        delta = vdiff(NewPosition, cg)

        self.MoveByDelta(delta)

    def MoveByDelta(self, Delta):
        """Move the entire assembly by the 3-tuple vector Delta (in world coordinates)."""
        # Apply the delta to all the bodies
        for body in self._bodyDict.itervalues():
            body.MoveByDelta(Delta)

    def MoveToPlaceConnectionAt(self, BodyKey, ConnectionID, Position):
        """Move the entire assembly such that the specified connection on the specified body
        is located at Position (3-tuple vector in world coordinates)."""
        # Lookup the body and get the location of the specified connection point.
        location = self.GetBody(BodyKey).GetConnectionLocation(ConnectionID)

        # Calculate the delta between the new position and the connection point
        delta = vdiff(Position, location)

        # Move the assembly
        self.MoveByDelta(delta)

    def RotateAboutCG(self, Quaternion):
        """Rotate the entire assembly about the cg by the Quaternion (normalized in world coordinates)."""
        # Get the current CG
        m, cg = self.GetMassAndCG();

        # Work on every body in the assembly.
        for body in self._bodyDict.itervalues():
            body.RotateAboutPoint(cg, Quaternion)

    def UpdateSettings(self, TimeNow=None):
        """This function must be called before each time step is executed.  It will
        call the UpdateSettings() function for every body in the assembly."""
        for key, body in self._bodyDict.iteritems():
            # Update the body Settings
            body.UpdateSettings(TimeNow=None)

