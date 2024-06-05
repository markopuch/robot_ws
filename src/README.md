# Proyecto_funda_utec
## dependiendo de tu espacio de trabajo
cd ~
source ~/lab_ws/devel/setup.bash
## Descarga
descargar estas carpetas dentro de tu src de tu espacio de trabajo

## visualizacion del robot
roslaunch lab1 display_irb5400_sliders.launch

### otra forma

roslaunch lab1 display_irb5400.launch

rosrun lab1 node_joints.py


## visualizar 
roslaunch lab1 display_irb5400.launch

## cinematica directa
rosrun lab1 test_fkine

## cinematica inversa
rosrun lab1 test_ikine

## control cinematico
rosrun lab1 control_cinematico

## control Dinamico
Descargar RBDL y asegurarse de que la libreria funcione correctamente.
### PD
rosrun lab1 control_pdg.py

### Feedforward linealization
rosrun lab1 control_feed.py
