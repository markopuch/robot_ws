from cgitb import reset
import numpy as np
from copy import copy
import time
import rbdl

cos = np.cos
sin = np.sin
pi = np.pi
time_start = time.time()


def Trasl(x, y, z):
    T = np.array([[1, 0, 0, x],
                  [0, 1, 0, y],
                  [0, 0, 1, z],
                  [0, 0, 0, 1]])
    return T


def Trotx(ang):
    T = np.array([[1, 0, 0, 0],
                  [0, np.cos(ang), -np.sin(ang), 0],
                  [0, np.sin(ang),  np.cos(ang), 0],
                  [0, 0, 0, 1]])
    return T


def Troty(ang):
    T = np.array([[cos(ang), 0, sin(ang), 0],
                  [0, 1, 0, 0],
                  [-sin(ang), 0, cos(ang), 0],
                  [0, 0, 0, 1]])
    return T


def Trotz(ang):
    T = np.array([[cos(ang), -sin(ang), 0, 0],
                  [sin(ang), cos(ang), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return T


def dh(d, th, a, alpha):
    cth = np.cos(th)
    sth = np.sin(th)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0,        sa,     ca,      d],
                  [0,         0,      0,      1]])
    return T


def fkine_irb(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    ("joint1", "joint2", "joint3","joint4_new", "joint4", "joint5", "joint6")
    """
    # Longitudes (en metros)
    l1 = 0.66
    l2 = 1.2
    l3 = 0.3
    l4 = 0.186
    l5 = 1.339
    l6 = 0.5
    l7 = 0.0693
    # l8 = 0.281-l7
    ag = 0.619
    a = 0.106246
    b = 0.074394
    c = 0.0672
    d = 0.047

   # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(l1, q[0], l3, -pi/2)
    T2 = dh(0, -pi/2+q[1], l2, 0)
    T3 = dh(0, pi+q[2], -l4, pi/2)
    T4 = dh(q[3]+l5+l6, 0, 0, 0)
    T5 = dh(l7, pi/2+q[4], 0, ag)
    T6 = Trotz(q[5]).dot(Trasl(0, b, a).dot(Trotx(-2*ag)))
    T7 = Trasl(0, -d, c).dot(Trotx(ag))
    T8 = dh(0.0, -pi/2+q[6], 0, 0)

    # Efector final con respecto a la base
    # T = T1.dot(T2.dot(T3.dot(T4.dot(T5.dot(T6.dot(T7))))))
    T = T1.dot(T2.dot(T3.dot(T4.dot(T5.dot(T6.dot(T7.dot(T8)))))))
    return T


def jacobian(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x7 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4_new, q4, q5, q6]
    """
    # Crear una matriz 3x7
    J = np.zeros((3, 7))
    # Utilizar la funcion que calcula la cinematica directa, para encontrar x,y,z usando q
    T_i = fkine_irb(q)
    # Vector posicion inicial
    T_i = T_i[0:3, 3:]

    # Iteracion para la derivada de cada columna
    for i in range(7):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        T = fkine_irb(dq)
        # Vector Posicion
        T = T[0:3, 3:]
        # Aproximacion del Jacobiano de posicion usando diferencias finitas, para la articulacion i-esima
        Jq = 1/delta*(T-T_i)
        J[:, i:i+1] = Jq

    return J


def ikine_irb(xdes, q0):
    """
    Calcular la cinematica inversa de irb numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon = 0.001
    max_iter = 1000
    delta = 0.00001

    q = copy(q0)

    for i in range(max_iter):
        # Main loop
        J = jacobian(q, delta)

        # Pseudo inversa del Jacobiano (matriz no es cuadrada)
        J = np.linalg.pinv(J)

        #  transformada homogenea
        T_i = fkine_irb(q)
        # Vector posicion actual
        T_i = T_i[0:3, 3]

        # Error = x deseado - x actual
        e = xdes - T_i

        # q(k+1) = q + Jacobiano * error
        q = q + np.dot(J, e)

        # Normal del error
        n_error = np.linalg.norm(e)

        # Condicion para finalizar
        if (n_error < epsilon):
            print("Error en la iteracion", i, {np.round(n_error, 4)})
            print("--- %s seconds ---" % (time.time() - time_start))
            break

        if (i == max_iter-1):
            print("El algoritmo no llego al valor deseado")
            print("Error en la iteracion", i, {np.round(n_error, 4)})

    return q


def ik_gradient_irb(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon = 0.001
    max_iter = 1000
    delta = 0.00001
    # Valor de alfa mas comun
    alpha = 0.1

    q = copy(q0)
    for i in range(max_iter):
        # Main loop
        J = jacobian(q, delta)

        # Pseudo inversa del Jacobiano (matriz no es cuadrada)
        J = J.T

        #  transformada homogenea
        T_i = fkine_irb(q)
        # Vector posicion actual
        T_i = T_i[0:3, 3]

        # Error = x deseado - x actual
        e = xdes - T_i

        # q(k+1) = q + Jacobiano * error
        q = q + alpha*np.dot(J, e)

        # Normal del error
        n_error = np.linalg.norm(e)

        # Condicion para finalizar
        if (n_error < epsilon):
            print("Error en la iteracion", i, {np.round(n_error, 4)})
            print("--- %s seconds ---" % (time.time() - time_start))
            break

        if (i == max_iter-1):
            print("El algoritmo no llego al valor deseado")
            print("Error en la iteracion", i, {np.round(n_error, 4)})

    return q

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/irb5400_1.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq