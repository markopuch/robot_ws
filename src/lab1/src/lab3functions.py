import numpy as np

cos = np.cos
sin = np.sin
pi = np.pi

def Trasl(x,y,z):
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
    T = np.array([[cos(ang), 0, sin(ang),0],
                   [0, 1, 0,0],
                   [-sin(ang), 0, cos(ang),0],
                  [0, 0, 0, 1]])
    return T
def Trotz(ang):
    T = np.array([[cos(ang), -sin(ang), 0,0],
                   [sin(ang), cos(ang), 0,0],
                   [0,0,1,0],
                   [0,0,0,1]])
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
    l1=0.66
    l2=1.2
    l3=0.3
    l4=0.186
    l5=1.339
    l6=0.5
    l7=0.0693
    l8=0.281-l7
    ag=0.619
    a=0.106246
    b=0.074394
    c=0.0672
    d=0.047

   # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(l1,q[0],l3,-pi/2)
    T2 = dh(0,-pi/2+q[1],l2,0)
    T3 = dh(0,pi+q[2],-l4, pi/2)
    T4 = dh(q[3]+l5+l6,0,0, 0)
    T5 = dh(l7,pi/2+q[4],0,ag)
    T6= Trotz(q[5]).dot(Trasl(0,b,a).dot(Trotx(-2*ag)))
    T7=  Trasl(0,-d,c).dot(Trotx(ag))
    T8= dh(0,-pi/2+q[6],0,0)


    # Efector final con respecto a la base
    # T = T1.dot(T2.dot(T3.dot(T4.dot(T5.dot(T6.dot(T7))))))
    T=T1.dot(T2.dot(T3.dot(T4.dot(T5.dot(T6.dot(T7.dot(T8)))))))
    return T




def jacobian_position(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    delta=0.0001
    # Crear una matriz 3x6
    J = np.zeros((3,7))
    # Transformacion homogenea inicial (usando q)

    T=fkine_irb(q)
    # Iteracion para la derivada de cada columna
    for i in xrange(6):
        # Copiar la configuracion articular inicial
        dq = q
        # Incrementar la articulacion i-esima usando un delta
	dq[i]=dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
	Td=fkine_irb(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
	#columna =1/delta * (fkine(dq) - fkine(q))
	J[0:3,i] = 1/delta * (Td[0:3,3] - T[0:3,3])
    return J


def jacobian_geom(q):
    
    # Longitudes (en metros)
    l1=0.66
    l2=1.2
    l3=0.3
    l4=0.186
    l5=1.339
    l6=0.5
    l7=0.0693
    l8=0.281-l7
    ag=0.619
    a=0.106246
    b=0.074394
    c=0.0672
    d=0.047

    T01 = dh(l1,q[0],l3,-pi/2)
    T12 = dh(0,-pi/2+q[1],l2,0)
    T23 = dh(0,pi+q[2],-l4, pi/2)
    T34 = dh(q[3]+l5+l6,0,0, 0)
    T45 = dh(l7,pi/2+q[4],0,ag)
    T56= Trotz(q[5]).dot(Trasl(0,b,a).dot(Trotx(-2*ag)))
    T67=  Trasl(0,-d,c).dot(Trotx(ag))
    

    T02=T01.dot(T12)
    T03=T01.dot(T12.dot(T23))
    T04=T01.dot(T12.dot(T23.dot(T34)))
    T05=T01.dot(T12.dot(T23.dot(T34.dot(T45))))
    T06=T01.dot(T12.dot(T23.dot(T34.dot(T45.dot(T56)))))
    T07=T01.dot(T12.dot(T23.dot(T34.dot(T45.dot(T56.dot(T67))))))



    z0=np.array([0,0,1])
    z1=T01[0:3,2]
    z2=T12[0:3,2]
    z3=T23[0:3,2]
    z4=T34[0:3,2]
    z5=T45[0:3,2]
    z6=T56[0:3,2]
    z7=T67[0:3,2]
    

    p0=np.array([0,0,0])
    p1=T01[0:3,3]
    p2=T02[0:3,3]
    p3=T03[0:3,3]
    p4=T04[0:3,3]
    p5=T05[0:3,3]
    p6=T06[0:3,3]
    p7=T07[0:3,3]
    

    Jv1=np.cross(z0,p7-p0)
    Jv2=np.cross(z1,p7-p1)
    Jv3=np.cross(z2,p7-p2)
    Jv4=np.cross(z3,p7-p3)
    Jv5=np.cross(z4,p7-p4)
    Jv6=np.cross(z5,p7-p5)
    Jv7=np.cross(z6,p7-p6)
   

    

    Jw1=z0
    Jw2=z1
    Jw3=z2
    Jw4=z3
    Jw5=z4
    Jw6=z5
    Jw7=z6
    
   

    J1=np.concatenate([Jv1,Jw1])
    J2=np.concatenate([Jv2,Jw2])
    J3=np.concatenate([Jv3,Jw3])
    J4=np.concatenate([Jv4,Jw4])
    J5=np.concatenate([Jv5,Jw5])
    J6=np.concatenate([Jv6,Jw6])
    J7=np.concatenate([Jv7,Jw7])
    

    J=np.matrix([J1,J2,J3,J4,J5,J6,J7])
    
    return J

def ikine_ur5(xd, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    fqact = open("/tmp/xactual.txt", "w")
    fqdes = open("/tmp/xdeseado.txt", "w")
    qact=open("/tmp/qact.txt", "w")
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
    q  = q0
    q_col = np.transpose(q)
    #for i in range(max_iter):
        # Main loop
     #   pass
   
    
    norm_e = 0
    for i in range(max_iter):
        J = jacobian_position(q)
	Jinv = np.linalg.pinv(J)
	f = fkine_irb(q)
        xact=f[0:3,3]
	e = xd-xact
	q = q + Jinv.dot(e);
	if q[0]<-2.62:
		
		q[0]=-2.62
		
	if q[0]>2.62:
		q[0]=1.40 

	if q[1]<-1.40:
		
		q[1]=-1.40
		
	if q[1]>1.40:
		q[1]=1.40

	if q[2]<-1.31:
		q[2]=-1.31
		
	if q[2]>1.31:
		q[2]=1.31
	        
	if q[3]<-0.44:
		q[3]=-0.44
		
	if q[3]>0.44:
		q[3]=0.44

	if q[4]<-6:
		
		q[4]=-6
		
	if q[4]>6:
		q[4]=6

        if q[5]<-6:
		
		q[5]=-6
		
	if q[5]>6:
		q[5]=6

        if q[6]<-6:
		
		q[6]=-6
		
	if q[6]>6:
		q[6]=6
	if np.linalg.norm(e)<epsilon:
	    break
            # Almacenamiento de datos
        fqact.write(str(f[0,3])+' '+str(f[1,3])+' '+str(f[2,3])+'\n')
        fqdes.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
        qact.write(str(q[0])+' '+ str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+ str(q[4])+' '+ str(q[5])+' '+ str(q[6])+'\n')
   
    return q
   

def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R


    return q
