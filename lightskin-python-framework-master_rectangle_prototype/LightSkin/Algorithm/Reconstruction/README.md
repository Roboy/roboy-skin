# Reconstruction algorithms
The hardware gives us a set of translucency values, for each pair of LED and sensor possible. The goal is to use these measurements together with the known arrangement of the components to reconstruct a map of the pressure applied on the skin.
This kind of problem is generally known as an inverse problem, and is quite similar to some medical applications: In computer tomography, for example, similar translucency values are measured through the human body from many angles and subsequently re- constructed to an image of a single slice. The main difference to this example is, that our measurements have rather chaotic arrangement and are not as numerous as in CT.
We implement the following reconstruction algorithms:

 * `SimpleBackProjection.py`: 
 * `SimpleDumbProportionalBackProjection.py`: 
 * `SimpleRepeatedBackProjection.py`:
 * `SimpleRepeatedDistributeBackProjection.py`:  
 * `SimpleRepeatedLogarithmicBackProjection.py`:  
 * `LogarithmicLinSysOptimize.py`: 
 * `LogarithmicLinSysOptimize2.py`: 
  
        
