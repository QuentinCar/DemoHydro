# -*- coding: utf-8 -*-
"""
Created on Wed Jul 17 17:18:03 2019

@author: Quentin
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.examples as ex
import numpy as np
import sys

app = QtGui.QApplication([])
w = gl.GLViewWidget()
w.setWindowTitle('pyqtgraph example: GLSurfacePlot')

## Add a grid to the view
g = gl.GLGridItem()
w.addItem(g)

## Add Axis
a = gl.GLAxisItem()
w.addItem(a)

# add a line
el2 = gl.GLLinePlotItem(pos=np.array([[0, 0, 1], [1, 1, 1]]), color=[1, 0.0, 1., 1.], width=10)
w.addItem(el2)

# add scatter plots
el1 = gl.GLScatterPlotItem(pos=np.array([[0, 0, 1], [1, 1, 1]]), color=[1, 0.0, 1., 1.], size=10)
w.addItem(el1)

w.show()
