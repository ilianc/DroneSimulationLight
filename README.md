# DroneSimulationLite
![](https://github.com/JqkerN/DroneSimulationLite/blob/master/DroneSimulatorLight.png)
An lighter version to visualize the simulation from gazebo. 
## Needed Packages
[Tkinter](https://riptutorial.com/tkinter/example/3206/installation-or-setup) -
*Should be pre-installed for Python.*<br/>
Otherwise try: <br/>
```
$ pip install python-tk
```
[pyopengltk](https://riptutorial.com/tkinter/example/3206/installation-or-setup)<br/>
```
$ pip install pyopengltk
```
[PyOpenGL](https://stackabuse.com/brief-introduction-to-opengl-in-python-with-pyopengl/)<br/>
```
$ pip install PyOpenGL
```
Add following line too your .launch file
```
<node pkg="DroneSimulationLight_2" type="GUI.py" name="DroneSimulationLight_2" args="demo01.world.json"/>
```
Finnaly, set your gazebo "gui" to false in your launch file. 
