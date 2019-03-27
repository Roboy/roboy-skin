# Ray models
The ray models are the implementation of the pressure influence model. They describe how much influence the different cells of our reconstruction have on a given ray between a LED and a sensor.
This conceptual model is defined in `RayGridInfluenceModel.py`. Here, we define the class `Ray`, which provides an interface to basic ray calculation such as length,
horizontal/vertical distance, points on the ray etc. Furthermore, we also define the class `RayGridInfluenceModel`, which describes the weight of specific cells in a grid for a specific ray.

We use two different Ray models: 

 * `DirectSampledRayGridInfluenceModel.py`:
 * `WideRayGridInfluenceModel.py`: 
