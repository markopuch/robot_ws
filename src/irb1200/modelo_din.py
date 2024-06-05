import rbdl
import numpy as np

# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('./urdf/irb1200.urdf')
# Grados de libertad
ndof = modelo.q_size


# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])


# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de CoSriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(6)               # Vector identidad


# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
#print(tau)
# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics
rbdl.InverseDynamics(modelo,q,zeros,zeros,g)
#print(np.round(g,3))
rbdl.InverseDynamics(modelo,q,dq,zeros,c)
c=c-g
#print(np.round(c,3))
for i in range (ndof):
    rbdl.InverseDynamics(modelo,q,zeros,e[i,:],M[i,:])
    M[i,:]=M[i,:]-g
#print(np.round(M,3))




# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
rbdl.CompositeRigidBodyAlgorithm(modelo,q,M2)
#print(np.round(M2,3))
rbdl.NonlinearEffects(modelo,q,dq,b2)
#print(np.round(b2,3))


# Parte 2: Verificacion de valores


# Parte 3: Verificacion de la expresion de la dinamica
#SE restaran los valores y deben ser igual a 0
r=M+c+g-M2-b2
print(np.round(r,2))