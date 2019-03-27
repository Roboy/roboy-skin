from matplotlib import cm

from typing import Callable, TypeVar, Union

from matplotlib.colors import Colormap

from .Color import Color

# Type variables exist primarily for the benefit of static type checkers. M can be any type.
M = TypeVar('M')

# A callable is passed to grayscale
# In this code a lambda function is passed, which receives a value as input and ouputs the input to the power of 2
# grayscale returns a callable: the callable expects anything as input. calculates it's power of 2, returns Color.from..


def Grayscale(tofloat: Callable[[M], float] = lambda x: x) -> Callable[[M], Color]:
    def display(val: M) -> Color:
        v = tofloat(val)
        return Color.fromFloats(v, v, v)

    return display


def MPColorMap(cmap: Union[str, Colormap], tofloat: Callable[[M], float] = lambda x: x) -> Callable[[M], Color]:
    """
        Returns a usable color translation function to a color map from MatPlotLib.
        Available colormaps are listed here: https://matplotlib.org/examples/color/colormaps_reference.html
    """
    if type(cmap) is str:
        cmap = cm.get_cmap(cmap)


    def display(val: M) -> Color:
        v = tofloat(val)
        c = cmap(v)
        return Color.fromFloats(c[0], c[1], c[2])

    return display
