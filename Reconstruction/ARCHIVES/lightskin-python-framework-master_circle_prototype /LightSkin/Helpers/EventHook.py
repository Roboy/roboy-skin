from typing import Callable, List


# Modeled after Michael Foord's Event Pattern:
# http://www.voidspace.org.uk/python/weblog/arch_d7_2007_02_03.shtml#e616

"""EventHook Objekt: Instanzattribut __delegates ist eine Liste von Callables.
   iadd Callable in Liste hinzufügen. isub Callable aus Liste entfernen. call returned liste mit return werten 
   der callables der __delegates liste mit parametern. clear __delegates liste leeren."""

class EventHook(object):

    def __init__(self):
        self.__delegates: List[Callable] = []

    def __iadd__(self, delegate: Callable):
        self.__delegates.append(delegate)
        return self

    def __isub__(self, delegate: Callable):
        self.__delegates.remove(delegate)
        return self

    def __call__(self, *args, **kwargs):
        ret = []
        for delegate in self.__delegates:
            ret.append(delegate(*args, **kwargs))
        return ret

    def clear(self):
        self.__delegates = []
