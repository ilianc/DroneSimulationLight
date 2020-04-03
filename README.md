# DroneSimulationLite
![](https://github.com/ilianc/DroneSimulationLight/blob/master/src/scripts/DroneSimulationLight2.png)
A lighter version to visualize the simulation from gazebo. 
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

## Setup 

Add following line to your .launch file
```
<node pkg="DroneSimulationLight" type="GUI.py" name="DroneSimulationLight" args="demo01.world.json"/>
```
Finally, set your gazebo "gui" to false in your launch file. 
