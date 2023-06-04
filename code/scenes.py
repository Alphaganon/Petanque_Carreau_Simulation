#!/usr/bin/env python3
#-*- coding: utf-8 -*-
#
# This file is part of SimulationTeachingElan, a python code used for teaching at Elan Inria.
#
# Copyright 2020 Mickael Ly <mickael.ly@inria.fr> (Elan / Inria - Université Grenoble Alpes)
# SimulationTeachingElan is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# SimulationTeachingElan is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with SimulationTeachingElan.  If not, see <http://www.gnu.org/licenses/>.
#

import numpy as np
import random
from dynamics.sphere_dynamic_system import SphereDynamicSystem
from dynamics.pendulum_dynamic_system import PendulumDynamicSystem

from graphics import *
from dynamics import *
from geom import *

from geom.cube3D import Cube3D
from geom.sphere3D import Sphere3D
from graphics.cube3D_renderable import Cube3DRenderable
from graphics.sphere3D_renderable import Sphere3DRenderable

def indexedTest(viewer):
    """
    @brief Demonstration for a basic static rendering
           Renders a simple square 
    """

    # Indexed square
    positions = np.array([0., 0.,   # x0, y0
                          1., 0.,   # x1, y1
                          0., 1.,   # x2, y2
                          1., 1.],  # x3, y3
                         np.float64)
    colours = np.array([1., 0., 0.,  # (r, g, b) for vertex 0
                        0., 0., 1.,  # (r, g, b) for vertex 1
                        0., 1., 0.,  # ...
                        1., 1., 1.]) # ...
    indices = np.array([0, 1, 2,   # First triangle composed by vertices 0, 1 and 2
                        1, 2, 3])  # Second triangle composed by vertices 1, 2 and 3

    # Create the object
    squareMesh = Mesh2D(positions, indices, colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)

def dynamicTest(viewer):
    """
    @brief Demonstration for a basic dynamic rendering
           Renders a simple square, moved by a dummy dynamic system
    """

    # Indexed square
    positions = np.array([0., 0.,   # x0, y0
                          1., 0.,   # x1, y1
                          0., 1.,   # x2, y2
                          1., 1.],  # x3, y3
                         np.float64)
    colours = np.array([1., 0., 0.,  # (r, g, b) for vertex 0
                        0., 0., 1.,  # (r, g, b) for vertex 1
                        0., 1., 0.,  # ...
                        1., 1., 1.]) # ...
    indices = np.array([0, 1, 2,   # First triangle composed by vertices 0, 1 and 2
                        1, 2, 3])  # Second triangle composed by vertices 1, 2 and 3

    # Create the object
    squareMesh = Mesh2D(positions, indices, colours)
    # Create the correspondung GPU object
    squareMeshRenderable = Mesh2DRenderable(squareMesh)
    # Add it to the list of objects to render
    viewer.addRenderable(squareMeshRenderable)

    # Create a dynamic system
    dyn = DummyDynamicSystem(squareMesh)
    # And add it to the viewer
    # Each frame will perform a call to the 'step' method of the viewer
    viewer.addDynamicSystem(dyn)
    


def rodTest(viewer):

    """
    @brief Demonstration for a rendering of a rod object
           Specific case, as a rod is essentialy a line, we
           need to generate a mesh over it to git it a thickness
           + demonstration of the scaling matrix for the rendering
    """
    positions = np.array([-1., 1.,
                          -1., 0.,
                          -0.5, -0.25],
                         np.float64)
    colours = np.array([1., 0., 0.,
                        0., 1., 0.,
                        0., 0., 1.])

    rod = Rod2D(positions, colours)

    rodRenderable = Rod2DRenderable(rod, thickness = 0.005)
    viewer.addRenderable(rodRenderable)
    
    positionsScaled = np.array([0., 1.,
                                0., 0.,
                                0.5, -0.25],
                               np.float64)
    rodScaled = Rod2D(positionsScaled, colours)

    rodRenderableScaled = Rod2DRenderable(rodScaled, thickness = 0.005)
    rodRenderableScaled.modelMatrix[0, 0] = 2.   # scale in X
    rodRenderableScaled.modelMatrix[1, 1] = 0.75 # scale in Y
    viewer.addRenderable(rodRenderableScaled)

def PendulumDTest(viewer):

    """
    @brief Demonstration for a rendering of a rod object
           Specific case, as a rod is essentialy a line, we
           need to generate a mesh over it to git it a thickness
           + demonstration of the scaling matrix for the rendering
    """
    positions = np.array([0., 0.,
                          np.cos(np.pi/5.), - np.sin(np.pi/5)],
                         np.float64)
    colours = np.array([0., 0., 0.,
                        0., 0., 0.])

    rod = Rod2D(positions, colours)

    rodRenderable = Rod2DRenderable(rod, thickness = 0.005)
    viewer.addRenderable(rodRenderable)
    dyn = PendulumDynamicSystem(rod, symplectic = False)
    viewer.addDynamicSystem(dyn)

def CarreTest(viewer):
    positions = np.array([-1., -2., 1., -1.], np.float64)
    colours = np.array([0., 0., 0., 0., 0., 0.], np.float64)
    rod = Rod2D(positions, colours)
    rodRenderable = Rod2DRenderable(rod, thickness = 0.005)
    viewer.addRenderable(rodRenderable)
    CarrePos = np.array([0., 0., 
                        0.5, 0.5, 
                        -0.5, 0.5, 
                        -0.5, -0.5, 
                        0.5, -0.5], np.float64)
    CarreCol = np.array([0., 0., 0., 
                        0., 0., 0., 
                        0., 0., 0., 
                        0., 0., 0., 
                        0., 0., 0.])
    CarreInd = np.array([0, 1, 2, 
                        0, 2, 3,
                        0, 3, 4,
                        0, 4, 1])
    carre = Mesh2D(CarrePos, CarreInd, CarreCol)
    carreRenderable = Mesh2DRenderable(carre)
    viewer.addRenderable(carreRenderable)
    dyn = CarreDynamicSystem(carre, rod)
    viewer.addDynamicSystem(dyn)

def CircleTest(viewer):
    positions = np.array([-1., -1., 1., -2.], np.float64)
    colours = np.array([0., 0., 0., 0., 0., 0.], np.float64)
    rod = Rod2D(positions, colours)
    rodRenderable = Rod2DRenderable(rod, thickness = 0.005)
    viewer.addRenderable(rodRenderable)
    rayon = 0.5
    CirclePos = [0., 0., rayon, 0.]
    CircleCol = [0., 0., 0., 0., 0., 0.]
    CircleInd = []
    theta = 0
    facets = 100
    for i in range(facets-1):
        theta += 2*np.pi / facets
        CirclePos.append(rayon*np.cos(theta))
        CirclePos.append(rayon*np.sin(theta))
        CircleCol.append(0.)
        CircleCol.append(0.)
        CircleCol.append(0.)
        CircleInd.append(0)
        CircleInd.append(len(CirclePos)-5-i)
        CircleInd.append(len(CirclePos)-4-i)
    CircleInd.append(0)
    CircleInd.append(len(CirclePos)/2 - 1)
    CircleInd.append(1)
    CirclePos = np.array(CirclePos, np.float64)
    CircleInd = np.array(CircleInd, np.float64)
    CircleCol = np.array(CircleCol, np.float64)
    circle = Mesh2D(CirclePos, CircleInd, CircleCol)
    circleRenderable = Mesh2DRenderable(circle)
    viewer.addRenderable(circleRenderable)
    dyn = CarreDynamicSystem(circle, rod)
    viewer.addDynamicSystem(dyn)

def SphereTest(viewer):
    # Plane
    thickness = 0.05
    size = 10.
    x_angle = 0.#np.pi/4
    y_angle = 0.#np.pi/4
    z_angle = 0.#np.pi/6
    rx = np.array([[1., 0., 0.], [0., np.cos(x_angle), -np.sin(x_angle)], [0., np.sin(x_angle), np.cos(x_angle)]])
    ry = np.array([[np.cos(y_angle), 0., np.sin(y_angle)], [0., 1., 0.], [-np.sin(y_angle), 0., np.cos(y_angle)]])
    rz = np.array([[np.cos(z_angle), -np.sin(z_angle), 0.], [np.sin(z_angle), np.cos(z_angle), 0.], [0., 0., 1.]])
    rotation = np.matmul(np.matmul(rz, ry), rx)
    plane = Cube3D(center=[0,-1,0], rotation=rotation, lengths=[size, thickness, size])
    plane.angles = np.array([x_angle, y_angle, z_angle], np.float64)
    planeRenderable = Cube3DRenderable(plane)
    viewer.addRenderable(planeRenderable)

    # Throwable Ball
    rayon = 0.25
    sphere = Sphere3D(center=np.array([0, 0, 0], np.float64), radius=rayon)
    sphereRenderable = Sphere3DRenderable(sphere)
    viewer.addRenderable(sphereRenderable)

    # Other Ball
    ob_rayon = 0.25
    ob_angle = random.uniform(-np.pi/2, np.pi/2)
    ob_sphere = Sphere3D(center=plane.center + np.array([-2. * np.sin(ob_angle), plane.lengths[1]/2 + ob_rayon, -2. * np.cos(ob_angle)], np.float64), radius=ob_rayon)
    ob_sphere_renderable = Sphere3DRenderable(ob_sphere)
    ob_sphere_renderable.colors = np.array( [
            1., 0., 0.,  # (r, g, b) for (+,+,+)
            1., 0., 0.,  # (r, g, b) for (+,+,-)
            1., 0., 0.,  # (r, g, b) for (+,-,+)
            1., 1., 0.,  # ...
            0., 1., 1.,
            0., 0., 1.,
            0., 0., 1.,
            0., 0., 1. ]
            )
    viewer.addRenderable(ob_sphere_renderable)

    # Dynamic System
    dyn = SphereDynamicSystem(sphere, ob_sphere, plane)
    viewer.addDynamicSystem(dyn)


