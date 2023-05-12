# autonomous-python-agents
a collection of python files which create simulations of autonomous steering agents and use TKinter to create a widget

# To Use
- create a new python file
- import GraphicsLib (this contains the scene class which is needed for rendering with TKinter)
- you can create basic geometry with the methods on a scene instance and can add your own
- also you can create Particle objects which can be animated with some simple motion (look at the testing method in GraphicsLib for an example
- For more advanced simulations import Automatons and create classes which inherit from the Vehicle class
- the Vehicle class has a number of behavior methods
- apply the behavior methods by overriding the the Vehicle classes update method (see Automotons file for a number of different examples)
