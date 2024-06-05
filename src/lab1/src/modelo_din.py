import rbdl
import numpy as np


# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/irb5400_1.urdf')
# Grados de libertad
ndof = modelo.q_size
print(ndof)
# Configuracion articular
q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Velocidad articular
dq = np.array([0.0, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1])
# Aceleracion articular
ddq = np.array([0.0, 0.1, 0.0, 0.1, 0.1, 0.1, 0.1])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau = np.zeros(ndof)          # Para torque
g = np.zeros(ndof)          # Para la gravedad
c = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M = np.zeros([ndof, ndof])  # Para la matriz de inercia
e = np.eye(7)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c=c-g

for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i], M[i])
    M[i] = M[i]-g

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)
rbdl.NonlinearEffects(modelo, q, dq, b2)


# Parte 2: Verificacion de valores
round = np.round
print("inverse", round(M, 1))
print("comprobar", round(M2, 1))

print("coreolis")
print("inverse", round(b2, 1))
print("comprobar", round(c+g, 1))


# Parte 3: Verificacion de la expresion de la dinamica

t = M.dot(ddq)+c+g
print("T1")
print(round(t, 1))
t2 = M2.dot(ddq)+b2
print("T2")
print(round(t2, 1))
print("T3")
print(round(tau, 1))
