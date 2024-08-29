import matplotlib.pyplot as mpl
import matplotlib.collections as mc
import os.path
import math
import zlib

from nsim import *

COMPRESSED_INPUTS = False #Only set to False when manually running the script and using regular uncompressed input files.

#Required names for files.
RAW_INPUTS = ["inputs_0", "inputs_1", "inputs_2", "inputs_3"]
RAW_MAP_DATA = "map_data"
MAP_IMG = None #Screenshot of level for the background of the plot. Optional.

#Import inputs.
inputs_list = []
for rinputs in RAW_INPUTS:
        if os.path.isfile(rinputs):
            with open(rinputs, "rb") as f:
                if COMPRESSED_INPUTS:
                    inputs_list.append([int(b) for b in zlib.decompress(f.read())])
                else:
                    inputs_list.append([int(b) for b in f.read()][215:])
        else:
            break

#import map data
mdata_list = []
with open(RAW_MAP_DATA, "rb") as f:
    mdata = [int(b) for b in f.read()]
for _ in range(len(inputs_list)):
    mdata_list.append(mdata)

#These dictionaries convert raw input data into the horizontal and jump components.
HOR_INPUTS_DIC = {0:0, 1:0, 2:1, 3:1, 4:-1, 5:-1, 6:-1, 7:-1}
JUMP_INPUTS_DIC = {0:0, 1:1, 2:0, 3:1, 4:0, 5:1, 6:0, 7:1}

xposlog = []
yposlog = []

#Repeat this loop for each individual replay
for i in range(len(inputs_list)):
    valid = False

    #Extract inputs and map data from the list
    inputs = inputs_list[i]
    mdata = mdata_list[i]

    #Convert inputs in a more useful format.
    hor_inputs = [HOR_INPUTS_DIC[inp] for inp in inputs]
    jump_inputs = [JUMP_INPUTS_DIC[inp] for inp in inputs]
    inp_len = len(inputs)

    #Initiate simulator and load the level
    sim = Simulator()
    sim.load(mdata)

    #Execute the main physics function once per frame
    while sim.frame < inp_len:
        hor_input = hor_inputs[sim.frame]
        jump_input = jump_inputs[sim.frame]
        sim.tick(hor_input, jump_input)
        if sim.ninja.state == 6:
            break
        if sim.ninja.state == 8:
            if sim.frame == inp_len:
                valid = True
            break

    #Append to the logs for each replay.
    xposlog.append(sim.ninja.xposlog)
    yposlog.append(sim.ninja.yposlog)

    print(sim.ninja.speedlog)
    print(sim.ninja.poslog)
    print(valid)

#Plot the route. Only ran in manual mode.
if len(inputs_list) >= 4:
    mpl.plot(xposlog[3], yposlog[3], "#910A46")
if len(inputs_list) >= 3:
    mpl.plot(xposlog[2], yposlog[2], "#4D31AA")
if len(inputs_list) >= 2:
    mpl.plot(xposlog[1], yposlog[1], "#EADA56")
mpl.plot(xposlog[0], yposlog[0], "#000000")
mpl.axis([0, 1056, 600, 0])
mpl.axis("off")
ax = mpl.gca()
ax.set_aspect("equal", adjustable="box")
if MAP_IMG:
    img = mpl.imread(MAP_IMG)
    ax.imshow(img, extent=[0, 1056, 600, 0])
lines = []
for cell in sim.segment_dic.values():
    for segment in cell:
        if segment.type == "linear":
            lines.append([(segment.x1, segment.y1), (segment.x2, segment.y2)])
        elif segment.type == "circular":
            angle = math.atan2(segment.hor, segment.ver) + (math.pi if segment.hor != segment.ver else 0)
            a1 = angle - math.pi/4
            a2 = angle + math.pi/4
            dist = a2 - a1
            quality = 8
            inc = dist / quality
            x1 = segment.xpos + segment.radius*math.cos(a1)
            y1 = segment.ypos + segment.radius*math.sin(a1)
            for i in range(1, quality+1):
                a1 += inc
                x2 = segment.xpos + segment.radius*math.cos(a1)
                y2 = segment.ypos + segment.radius*math.sin(a1)
                lines.append([(x1, y1), (x2, y2)])
                x1, y1 = x2, y2
lc = mc.LineCollection(lines)
ax.add_collection(lc)
mpl.show()