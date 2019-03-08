#!/usr/bin/python3
import csv
import time
import tkinter as tk
from LightSkin.LightSkin import LightSkin, ValueMap
from LightSkin.GUI import Views
from LightSkin.Algorithm.RayInfluenceModels.DirectSampledRayGridInfluenceModel import DirectSampledRayGridInfluenceModel
from LightSkin.Algorithm.Reconstruction.LogarithmicLinSysOptimize2 import LogarithmicLinSysOptimize2
from LightSkin.Algorithm.Reconstruction.SimpleRepeatedDistributeBackProjection import SimpleRepeatedDistributeBackProjection
from LightSkin.Algorithm.Reconstruction.SimpleRepeatedLogarithmicBackProjection import SimpleRepeatedLogarithmicBackProjection
from LightSkin.Algorithm.ForwardModels.SimpleProportionalForwardModel import SimpleProportionalForwardModel
from LightSkin.Algorithm.ForwardModels.Calibration.SimpleIdealProportionalCalibration import SimpleIdealProportionalCalibration
from LightSkin.Algorithm.Reconstruction.SimpleBackProjection import SimpleBackProjection

# Source: https://code.activestate.com/recipes/410687-transposing-a-list-of-lists-with-different-lengths/


def transposed(lists):
    if not lists:
        return []
    return list(map(lambda *row: list(row), *lists))


# Main Code

ls = LightSkin()

# LOAD Sensor and LED coordinates from CSV

with open('CSV_Files/sensors.csv', 'r') as csvfile:
    read = csv.reader(csvfile)
    for r in read:
        s = (float(r[0]), float(r[1]))
        ls.sensors.append(s)
#
with open('CSV_Files/leds.csv', 'r') as csvfile:
    read = csv.reader(csvfile)
    for r in read:
        s = (float(r[0]), float(r[1]))
        ls.LEDs.append(s)

gridVals = []
with open('CSV_Files/translucency.csv', 'r') as csvfile:
    read = csv.reader(csvfile)
    for r in read:
        vals = list(map(float, r))
        gridVals.append(vals)

translucency = ValueMap(ls.getGridArea(), grid=transposed(gridVals))
ls.translucencyMap = translucency

recSize = 10
repetitions = 20

ls.forwardModel = SimpleProportionalForwardModel(ls, DirectSampledRayGridInfluenceModel())
ls.backwardModel = SimpleBackProjection(ls, recSize,
                                        recSize,
                                        SimpleIdealProportionalCalibration(ls),
                                        DirectSampledRayGridInfluenceModel())
repeated = SimpleRepeatedLogarithmicBackProjection(ls, recSize,
                                        recSize,
                                        SimpleIdealProportionalCalibration(ls),
                                        DirectSampledRayGridInfluenceModel(), repetitions)
repeated2 = SimpleRepeatedDistributeBackProjection(ls, recSize,
                                        recSize,
                                        SimpleIdealProportionalCalibration(ls),
                                        DirectSampledRayGridInfluenceModel(), repetitions)

linsys = LogarithmicLinSysOptimize2(ls, recSize,
                                        recSize,
                                        SimpleIdealProportionalCalibration(ls),
                                        DirectSampledRayGridInfluenceModel())

ls.backwardModel.calculate()

start_time = time.time()
repeated.calculate()
t = time.time() - start_time
print("Total time needed for calculation: %f " % t)

start_time = time.time()
repeated2.calculate()
t = time.time() - start_time
print("Total time needed for calculation: %f " % t)

start_time = time.time()
linsys.calculate()
t = time.time() - start_time
print("Total time needed for calculation: %f " % t)

# print(ls.sensors)
# print(ls.LEDs)
# print(flush=True)


pressure_colormap = 'nipy_spectral'


# Build Window with widgets etc...
window = tk.Tk()
window.title('Light Skin Simulation')
window.minsize(900, 300)

topViewsFrame = tk.Frame(window)
topViewsFrame.pack(side=tk.TOP)
topViews2Frame = tk.Frame(window)
topViews2Frame.pack(side=tk.TOP)


'''topViewTransl = Views.LightSkinTopView(topViewsFrame, ls, highlightbackground='#aaa', highlightthickness=1,
                                       width=300, height=300,
                                       measurable_grid=ls.translucencyMap,
                                       display_function=Views.Colorscales.MPColorMap(pressure_colormap)
                                       )
topViewTransl.pack(side=tk.LEFT)
'''
'''topView = Views.LightSkinTopView(topViewsFrame, ls, highlightbackground='#aaa', highlightthickness=1,
                                 width=300, height=300,
                                 gridWidth=50, gridHeight=50,
                                 display_function=Views.Colorscales.MPColorMap('plasma', lambda x: x ** 0.5),
                                 measure_function=ls.forwardModel.measureAtPoint
                                 )
topView.pack(side=tk.LEFT)'''
T = tk.Text(topViewsFrame, height=2, width=10)
T.pack(side=tk.TOP)
T.insert(tk.END, "Point 2x7")


T = tk.Text(topViewsFrame, height=2, width=30)
T.pack(side=tk.TOP, anchor="w")
T.insert(tk.END, "Simple Backprojection")
T = tk.Text(topViewsFrame, height=2, width=40)
T.pack(side=tk.TOP, anchor="e")
T.insert(tk.END, "Repeated Logarithmic BackProjection")


T = tk.Text(topViews2Frame, height=2, width=60)
T.pack(side=tk.TOP, anchor="w")
T.insert(tk.END, "Repeated Distributed BackProjection")
T = tk.Text(topViews2Frame, height=2, width=30)
T.pack(side=tk.TOP, anchor="e")
T.insert(tk.END, "Linear System")


topViewReconstructed = Views.LightSkinTopView(topViewsFrame, ls, highlightbackground='#aaa', highlightthickness=1,
                                              width=300, height=300,
                                              measurable_grid=ls.backwardModel,
                                              display_function=Views.Colorscales.MPColorMap(pressure_colormap),
                                              )
topViewReconstructed.pack(side=tk.LEFT)

topViewReconstructed2 = Views.LightSkinTopView(topViewsFrame, ls, highlightbackground='#aaa', highlightthickness=1,
                                               width=300, height=300,
                                               measurable_grid=repeated,
                                               display_function=Views.Colorscales.MPColorMap(pressure_colormap),
                                               )
topViewReconstructed2.pack(side=tk.LEFT)



topViewReconstructed3 = Views.LightSkinTopView(topViews2Frame, ls, highlightbackground='#aaa', highlightthickness=1,
                                               width=300, height=300,
                                               measurable_grid=repeated2,
                                               display_function=Views.Colorscales.MPColorMap(pressure_colormap),
                                               )
topViewReconstructed3.pack(side=tk.LEFT)


topViewReconstructed4 = Views.LightSkinTopView(topViews2Frame, ls, highlightbackground='#aaa', highlightthickness=1,
                                               width=300, height=300,
                                               measurable_grid=linsys,
                                               display_function=Views.Colorscales.MPColorMap(pressure_colormap),
                                               )
topViewReconstructed4.pack(side=tk.LEFT)

gridView = Views.LightSkinGridView(window, ls, width=400, height=400, highlightbackground='#aaa', highlightthickness=1,
                                   display_function=Views.Colorscales.MPColorMap('plasma')
                                   )
gridView.pack(side=tk.BOTTOM)

# topViewReconstructed.postscript(file='Testfile.ps')

window.mainloop()

# Main program logic follows:
# if __name__ == '__main__':
#
