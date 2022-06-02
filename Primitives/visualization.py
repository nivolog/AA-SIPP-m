import numpy as np
import math
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import xml.etree.ElementTree as ET

prim_name = 'Primitives_AA2_cells.xml'




def draw_prim(coeffs, figure, sx, sy, duration, phi0):
    t = np.arange(0, duration+0.01, 0.01)
    if phi0 == 22.5:
        print(math.cos(phi0 * math.pi / 180))
    dx = np.array([a*(t**i) for i, a in enumerate(coeffs[0], start=0)]).sum(axis = 0)
    dy = np.array([b*(t**i) for i, b in enumerate(coeffs[1], start=0)]).sum(axis = 0)
    #dx  = _dx*math.cos(math.radians(phi0)) - _dy*math.sin(math.radians(phi0))
    #dy  = _dx*math.sin(math.radians(phi0)) + _dy*math.cos(math.radians(phi0))
    #tx = 2
    #ty = 0
    #ttx = tx*math.cos(math.radians(phi0)) - ty*math.sin(math.radians(phi0))
    #tty = tx*math.sin(math.radians(phi0)) + ty*math.cos(math.radians(phi0))
    x = sx + dx
    y = sy + dy
    #print(ttx,tty)
    plt.plot(x, y)
    
    #plt.plot(np.arange(0,10,1), np.arange(0,20,2))
    #pass

if __name__ == '__main__':
    fig = plt.figure(figsize = (20,20))
    ax = fig.add_subplot(111)
    plt.xlim(-25,25)
    plt.ylim(-25,25)
    ax.set_aspect('equal', adjustable='box')
    resolution = 0.2
    prims_root = ET.parse(prim_name)
    prims = {-1: [[], []]}
    for primitive in prims_root.iter('coeff'):
        a1 = float(primitive.attrib['a1'])/resolution
        a2 = float(primitive.attrib['a2'])/resolution
        a3 = float(primitive.attrib['a3'])/resolution
        a4 = float(primitive.attrib['a4'])/resolution
        b1 = float(primitive.attrib['b1'])/resolution
        b2 = float(primitive.attrib['b2'])/resolution
        b3 = float(primitive.attrib['b3'])/resolution
        b4 = float(primitive.attrib['b4'])/resolution
        duration = float(primitive.attrib['Tf'])
        phi0 = float(primitive.attrib['phi0'])
        phif = float(primitive.attrib['phif'])
        id = int(primitive.attrib['id'])
        coeffs = [[a1, a2, a3, a4], [b1, b2, b3, b4], duration, id]
        prims.update({id: coeffs})
        #print(duration, phi0 * math.pi / 180, id)
        if True: #phi0 == 0:
            draw_prim(coeffs, fig, 0, 0, duration, phi0)
	
plt.show()

