# Ejemplo de matlab de la clase en python

import numpy as np
import math
import matplotlib.pyplot as plt 

import roboticstoolbox as rtb

# Funcion para graficar la respuesta de los estados en Lazo Cerrado
def close_loop_plot(t_array, data1, data2, data3, data4, data5, data6):

	fig1, ax1 = plt.subplots()
	ax1.set_title('Respuesta del sistema controlado y los estados deseados')
	# ax1.set_ylim(-20, 20)
	# ax1.set_xlim(-20, 20)

	ax1.plot(t_array, data1, label='x1')
	ax1.plot(t_array, data2, label='x2')
	ax1.plot(t_array, data3, label='x3')

	ax1.plot(t_array, data4, '--', label='x1d')
	ax1.plot(t_array, data5, '--', label='x2d')
	ax1.plot(t_array, data6, '--', label='x3d')

	ax1.legend()

# Funcion para graficar la respuesta de los controles
def velocities_plot(t_array, data1, data2, data3):

	fig3, ax3 = plt.subplots()
	ax3.set_title('Velocidades de las articulaciones')
	# ax3.set_ylim(-20, 4)
	# ax3.set_xlim(-0.01, 10)

	ax3.plot(t_array, data1, label='da1')
	ax3.plot(t_array, data2, label='da2')
	ax3.plot(t_array, data3, label='da3')

	ax3.legend()

# Funcion para graficar la respuesta de los errores
def error_plot(t_array, e1, e2, e3):

	fig4, ax4 = plt.subplots()
	ax4.set_title('Respuesta de los errores')
	# ax4.set_ylim(-10, 2)
	# ax4.set_xlim(-0.01, 10)

	ax4.plot(t_array, e1, label='e1')
	ax4.plot(t_array, e2, label='e2')
	ax4.plot(t_array, e3, label='e3')

	ax4.legend()

# Funcion para graficar los angulos de las articulaciones
def angles_plot(t_array, data1, data2, data3):

	fig5, ax5 = plt.subplots()
	ax5.set_title('Angulos de las articulaciones')
	# ax3.set_ylim(-20, 4)
	# ax3.set_xlim(-0.01, 10)

	ax5.plot(t_array, data1, label='q1')
	ax5.plot(t_array, data2, label='q2')
	ax5.plot(t_array, data3, label='q3')

	ax5.legend()

# Longitud de los brazos del robot
d1 = 1.0
d2 = 0.8
d3 = 0.4

# Angulos iniciales
# q0 = np.array([[0.2],
# 			   [0.2],
# 			   [0.2]])
q0 = [0.2, 0.2, 0.2]
q = q0

# Tiempo de muestreo
dt = 0.05
da = np.array([[0.0], # Velocidades de las articulaciones
			   [0.0],
			   [0.0]])

# Definiendo el robot
L1 = rtb.robot.DHLink(d=0, alpha=0, a=d1)
L2 = rtb.robot.DHLink(d=0, alpha=0, a=d2)
L3 = rtb.robot.DHLink(d=0, alpha=0, a=d3)

bot = rtb.robot.DHRobot([L1, L2, L3], name='my robot')
print(bot)

# Estimando la posicion cartesiana de la configuracion inicial
T0 = bot.fkine(q0)
x0 = np.array([[T0.t[0]],
			   [T0.t[1]],
			   [math.atan2(T0.n[1], T0.n[0])]])
x = x0

# Parametros del controlador 
xd = np.array([[0.8],
			   [1.5],
			   [0.3]])

e = x - xd

K = np.array([[0.5, 0.0, 0.0],
			  [0.0, 0.5, 0.0],
			  [0.0, 0.0, 0.5]])

# Arreglos para guardar parametros
t_data = [0.0]

x1_data = [x[0][0]]
x2_data = [x[1][0]]
x3_data = [x[2][0]]
xd1_data = [xd[0][0]]
xd2_data = [xd[1][0]]
xd3_data = [xd[2][0]]

e1_data = [e[0][0]]
e2_data = [e[1][0]]
e3_data = [e[2][0]]

q1_data = [q[0]]
q2_data = [q[1]]
q3_data = [q[2]]

da1_data = [da[0][0]]
da2_data = [da[1][0]]
da3_data = [da[2][0]]

# Inicio de bucle
t = 0.0
while t <= 10:

	# bot.plot(q)

	# Jacobiano
	s1 = np.sin(q[0])
	s12 = np.sin(q[0] + q[1])
	s123 = np.sin(q[0] + q[1] + q[2])
	c1 = np.cos(q[0])
	c12 = np.cos(q[0] + q[1])
	c123 = np.cos(q[0] + q[1] + q[2])

	J = np.array([[-d1*s1 - d2*s12 - d3*s123, -d2*s12 - d3*s123, -d3*s123],
				  [d1*c1 + d2*c12 + d3*c123, d2*c12 + d3*c123, d3*c123],
				  [1, 1, 1]])

	# Integracion de modelo dinamicos
	x = x + (J@da)*dt
	e = x - xd # Error cartesiano
	da = np.linalg.inv(J) @ (-K@e)
	q = q + da.T[0]*dt

	x1_data.append(x[0][0])
	x2_data.append(x[1][0])
	x3_data.append(x[2][0])

	xd1_data.append(xd[0][0])
	xd2_data.append(xd[1][0])
	xd3_data.append(xd[2][0])

	e1_data.append(e[0][0])
	e2_data.append(e[1][0])
	e3_data.append(e[2][0])

	q1_data.append(q[0])
	q2_data.append(q[1])
	q3_data.append(q[2])

	da1_data.append(da[0][0])
	da2_data.append(da[1][0])
	da3_data.append(da[2][0])

	print(t)
	t_data.append(t)
	t += dt

bot.plot(q)

print(bot.fkine(q), '\n', x)

close_loop_plot(t_data, x1_data, x2_data, x3_data, xd1_data, xd2_data, xd3_data)
velocities_plot(t_data, da1_data, da2_data, da3_data)
error_plot(t_data, e1_data, e2_data, e3_data)
angles_plot(t_data, q1_data, q2_data, q3_data)

plt.show()

input()