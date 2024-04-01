# Robot Puma, similar al ejemplo en clase

import numpy as np
import math
import matplotlib.pyplot as plt 

import roboticstoolbox as rtb

# Funcion para graficar la respuesta de los estados en Lazo Cerrado
def close_loop_plot(t_array, data1, data2, data3, data7, data8, data9,
					 data4, data5, data6, data10, data11, data12):

	fig1, ax1 = plt.subplots()
	ax1.set_title('Respuesta del sistema controlado y los estados deseados')

	ax1.plot(t_array, data1, label='x1')
	ax1.plot(t_array, data2, label='x2')
	ax1.plot(t_array, data3, label='x3')
	ax1.plot(t_array, data7, label='x4')
	ax1.plot(t_array, data8, label='x5')
	ax1.plot(t_array, data9, label='x6')

	ax1.plot(t_array, data4, '--', label='x1d')
	ax1.plot(t_array, data5, '--', label='x2d')
	ax1.plot(t_array, data6, '--', label='x3d')
	ax1.plot(t_array, data10, '--', label='x4d')
	ax1.plot(t_array, data11, '--', label='x5d')
	ax1.plot(t_array, data12, '--', label='x6d')

	ax1.legend()

# Funcion para graficar la respuesta de los controles
def velocities_plot(t_array, data1, data2, data3, data4, data5, data6):

	fig3, ax3 = plt.subplots()
	ax3.set_title('Velocidades de las articulaciones')

	ax3.plot(t_array, data1, label='da1')
	ax3.plot(t_array, data2, label='da2')
	ax3.plot(t_array, data3, label='da3')
	ax3.plot(t_array, data4, label='da4')
	ax3.plot(t_array, data5, label='da5')
	ax3.plot(t_array, data6, label='da6')

	ax3.legend()

# Funcion para graficar la respuesta de los errores
def error_plot(t_array, e1, e2, e3, e4, e5, e6):

	fig4, ax4 = plt.subplots()
	ax4.set_title('Respuesta de los errores')

	ax4.plot(t_array, e1, label='e1')
	ax4.plot(t_array, e2, label='e2')
	ax4.plot(t_array, e3, label='e3')
	ax4.plot(t_array, e4, label='e4')
	ax4.plot(t_array, e5, label='e5')
	ax4.plot(t_array, e6, label='e6')

	ax4.legend()

# Funcion para graficar los angulos de las articulaciones
def angles_plot(t_array, data1, data2, data3, data4, data5, data6):

	fig5, ax5 = plt.subplots()
	ax5.set_title('Angulos de las articulaciones')

	ax5.plot(t_array, data1, label='q1')
	ax5.plot(t_array, data2, label='q2')
	ax5.plot(t_array, data3, label='q3')
	ax5.plot(t_array, data4, label='q4')
	ax5.plot(t_array, data5, label='q5')
	ax5.plot(t_array, data6, label='q6')

	ax5.legend()

def get_jacobian(d, a, q, option=1):

	c1 = np.cos(q[0])
	c2 = np.cos(q[1])
	c3 = np.cos(q[2])
	c23 = np.cos(q[1] + q[2])
	c4 = np.cos(q[3])
	c5 = np.cos(q[4])

	s1 = np.sin(q[0])
	s2 = np.sin(q[1])
	s3 = np.sin(q[2])
	s23 = np.sin(q[1] + q[2])
	s4 = np.sin(q[3])
	s5 = np.sin(q[4])

	# Primero	
	if option == 1:

		j11 = -s1*(c23*a[2] + s23*d[3] + c2*a[1]) - c1*(d[1] + d[2])
		j12 = c1*(-s23*a[2] + c23*d[3] - s2*a[1])
		j13 = c1*(-s23*a[2] + c23*d[3])

		j21 = c1*(c23*a[2] + s23*d[3] + c2*a[1]) - s1*(d[1] + d[2])
		j22 = s1*(-s23*a[2] + c23*d[3] - s2*a[1])
		j23 = s1*(-s23*a[2] + c23*d[3])

		j32 = -(c23*a[1] + s23*d[3] + c2*a[1])
		j33 = -(c23*a[2] + s23*d[3])

		j44 = c1*s23
		j45 = -c1*c23*s4 - s1*c4
		j46 = (c1*c23*c4 - s1*s4)*s5 + c1*s23*c5

		j54 = s1*s23
		j55 = -s1*c23*s4 + c1*c4
		j56 = (s1*c23*c4 + c1*s4)*s5 + s1*s23*c5

		j65 = s23*s4
		j66 = -s23*c4*s5 + c23*c5

		J = np.array([[j11, j12, j13, 0, 0, 0],
					  [j21, j22, j23, 0, 0, 0],
					  [0, j32, j33, 0, 0, 0],
					  [0, -s1, -s1, j44, j45, j46],
					  [0, -c1, -c1, j54, j55, j56],
					  [1, 0, 0, c23, j65, j66]])

	# Segundo
	if option == 2:

		j11 = -c1*c23*a[2] + s1*s23*a[2] - c1*s23*d[3] - s1*c23*d[3] + s2*a[1] + s1*(d[1] + d[2])
		j12 = s1*s23*a[2] - c1*c23*a[2] - s1*c23*d[3] - c1*s23*d[3] + s1*s2*a[1] - c1*c2*a[1]
		j13 = s1*s23*a[2] - c1*c23*a[2] - s1*c23*d[3] - c1*s23*d[3]

		j21 = -s1*c23*a[2] - c1*s23*a[2] - s1*s23*d[3] + c1*c23*d[3] - s1*c2*a[1] - c1*s2*a[1] - c1*(d[1] + d[2])
		j22 = -c1*s23*a[2] - s1*c23*a[2] + c1*c23*d[3] - s1*s23*d[3] - c1*s2*a[1] - s1*c2*a[1]
		j23 = -c1*s23*a[2] - s1*c23*a[2] + c1*c23*d[3] - s1*s23*d[3]

		j32 = s23*a[1] - c23*d[3] + s2*a[1]
		j33 = s23*a[2] - c23*d[3]

		j44 = -s1*s23 + c1*c23
		j45 = s1*c23*s4 + c1*s23*s4 - c1*c23*c4 - c1*c4 + s1*s4
		j46 = -s1*c23*c4*s5 - c1*s23*c4*s5 - c1*c23*s4*s5 + c1*c23*c4*c5 - c1*s4*s5 - s1*c4*s5 - s1*s4*c5 - s1*c23*c5 - c1*s23*c5 - c1*c23*s5

		j54 = c1*c23 + s1*c23
		j55 = -c1*c23*s4 + s1*s23*s4 - s1*c23*c4 - s1*c4 - c1*s4
		j56 = c1*c23*c4*s5 - s1*s23*c4*s5 - s1*c23*s4*s5 + s1*c23*c4*c5 - s1*s4*s5 + c1*c4*s5 + c1*s4*c5 + c1*c23*c5 + s1*c23*c5 - s1*s23*s5
	 
		j65 = c23*s4 + s23*c4
		j66 = -c23*c4*s5 + s23*s4*s5 - s23*c4*c5 - s23*c5 - c23*s5

		J = np.array([[j11, j12, j13, 0, 0, 0],
					  [j21, j22, j23, 0, 0, 0],
					  [0, j32, j33, 0, 0, 0],
					  [0, -c1, -c1, j44, j45, j46],
					  [0, -s1, -s1, j54, j55, j56],
					  [0, 0, 0, -s23, j65, j66]])

	# Tercero
	if option == 3:

		x1 = 0.4318
		x3 = 0.15005

		j11 = x3*c1 - x1*c2*s1 - x1*c2*c3*s1 + x1*s1*s2*s3
		j12 = -c1*(x1*s2 + x1*c2*s3 + x1*c3*s2)
		j13 = -c1*(x1*c2*s3 + x1*c3*s2)

		j21 = x3*s1 + x1*c1*c2 + x1*c1*c2*c3 - x1*c1*s2*s3
		j22 = -s1*(x1*s2 + x1*c2*s3 + x1*c3*s2)
		j23 = -s1*(x1*c2*s3 + x1*c3*s2)

		j32 = c1*(x3*s1 + x1*c1*c2 + x1*c1*c2*c3 - x1*c1*s2*s3) - s1*(x3*c1 - x1*c2*s1 - x1*c2*c3*s1 + x1*s1*s2*s3)
		j33 = c1*(x3*s1 + x1*c1*c2*c3 - x1*c1*s2*s3) - s1*(x3*c1 - x1*c2*c3*s1 + x1*s1*s2*s3)

		j44 = -c1*c2*s3 - c1*c3*s2
		j45 = c4*s1 + s4*(c1*c2*c3 - c1*s2*s3)
		j46 = s5*(s1*s4 - c4*(c1*c2*c3 - c1*s2*s3)) - c5*(c1*c2*s3 + c1*c3*s2)

		j54 = -c2*s1*s3 - c3*s1*s2
		j55 = s4*(c2*c3*s1 - s1*s2*s3) - c1*c4
		j56 = -c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 + c4*(c2*c3*s1 - s1*s2*s3))
	 	
		j64 = c2*c3 - s2*s3
		j65 = s4*(c2*s3 + c3*s2)
		j66 = c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)

		J = np.array([[j11, j12, j13, 0, 0, 0],
					  [j21, j22, j23, 0, 0, 0],
					  [0, j32, j33, 0, 0, 0],
					  [0, s1, s1, j44, j45, j46],
					  [0, -c1, -c1, j54, j55, j56],
					  [1, 0, 0, j64, j65, j66]])

	return J

# Funcion principal...
if __name__ == '__main__':

	# Angulos iniciales
	q0 = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
	q = q0

	# Tiempo de muestreo
	dt = 0.05
	da = np.array([[0.0], # Velocidades de las articulaciones
				   [0.0],
				   [0.0],
				   [0.0],
				   [0.0],
				   [0.0]])

	# Definiendo el robot
	bot = rtb.models.DH.Puma560()
	print(bot)

	# Longitud de los brazos del robot
	a1 = bot.a[0]
	a2 = bot.a[1]
	a3 = bot.a[2]
	a4 = bot.a[3]
	a5 = bot.a[4]
	a6 = bot.a[5]

	d1 = bot.d[0]
	d2 = bot.d[1]
	d3 = bot.d[2]
	d4 = bot.d[3]
	d5 = bot.d[4]
	d6 = bot.d[5]

	# Estimando la posicion cartesiana de la configuracion inicial
	q0 = bot.qn
	q = q0
	T0 = bot.fkine(q0)
	print(T0)

	x0 = np.array([[T0.t[0]],
				   [T0.t[1]],
				   [T0.t[2]],
				   [math.atan2(T0.n[1], T0.n[0])],
				   [math.atan2(T0.n[2], T0.n[0])],
				   [math.atan2(T0.n[2], T0.n[1])]])

	input()
	x = x0

	# Parametros del controlador 
	t = 0.0
	xd = np.array([[0.5 + np.sin(t)],
				   [0.5 + np.sin(t)],
				   [0.5 + np.sin(t)],
				   [0.7],
				   [0.5],
				   [1.5]])

	xd_diff = np.array([[np.cos(t)],
				   		[np.cos(t)],
				   		[np.cos(t)],
				   		[0.0],
				   		[0.0],
				   		[0.0]])

	e = x - xd 

	K = np.array([[2, 0.0, 0.0, 0.0, 0.0, 0.0],
				  [0.0, 3, 0.0, 0.0, 0.0, 0.0],
				  [0.0, 0.0, 4, 0.0, 0.0, 0.0],
				  [0.0, 0.0, 0.0, 1, 0.0, 0.0],
				  [0.0, 0.0, 0.0, 0.0, 2.5, 0.0],
				  [0.0, 0.0, 0.0, 0.0, 0.0, 3.5]])

	# Arreglos para guardar parametros
	t_data = [0.0]

	x1_data = [x[0][0]]
	x2_data = [x[1][0]]
	x3_data = [x[2][0]]
	x4_data = [x[3][0]]
	x5_data = [x[4][0]]
	x6_data = [x[5][0]]
	xd1_data = [xd[0][0]]
	xd2_data = [xd[1][0]]
	xd3_data = [xd[2][0]]
	xd4_data = [xd[3][0]]
	xd5_data = [xd[4][0]]
	xd6_data = [xd[5][0]]

	e1_data = [e[0][0]]
	e2_data = [e[1][0]]
	e3_data = [e[2][0]]
	e4_data = [e[3][0]]
	e5_data = [e[4][0]]
	e6_data = [e[5][0]]

	q1_data = [q[0]]
	q2_data = [q[1]]
	q3_data = [q[2]]
	q4_data = [q[3]]
	q5_data = [q[4]]
	q6_data = [q[5]]

	da1_data = [da[0][0]]
	da2_data = [da[1][0]]
	da3_data = [da[2][0]]
	da4_data = [da[3][0]]
	da5_data = [da[4][0]]
	da6_data = [da[5][0]]

	# Inicio de bucle
	while t <= 5:

		bot.plot(q)

		# Jacobiano
		# J = get_jacobian(bot.d, bot.a, q, option=2)
		J = bot.jacobe(q)

		xd = np.array([[0.5 + np.cos(t)],
				   [0.5 + np.sin(t)],
				   [0.5 + np.sin(t)],
				   [0.7],
				   [0.5],
				   [1.5]])

		xd_diff = np.array([[-np.sin(t)],
				   			[np.cos(t)],
				   			[np.cos(t)],
				   			[0.0],
				   			[0.0],
				   			[0.0]])

		# Integracion de modelo dinamicos
		e = x - xd # Error cartesiano
		da = np.linalg.inv(J) @ (xd_diff-K@e) # Calculo de control
		# da = (-K@e) # Calculo de control
		if t == 0.0:
			x = x
		else:
			x = x + (J@da)*dt # Nuevos estados
			# x = x + (da)*dt # Nuevos estados
		q = q + da.T[0]*dt # Angulo deseado 

		# Gurdando datos para graficas
		x1_data.append(x[0][0])
		x2_data.append(x[1][0])
		x3_data.append(x[2][0])
		x4_data.append(x[3][0])
		x5_data.append(x[4][0])
		x6_data.append(x[5][0])

		xd1_data.append(xd[0][0])
		xd2_data.append(xd[1][0])
		xd3_data.append(xd[2][0])
		xd4_data.append(xd[3][0])
		xd5_data.append(xd[4][0])
		xd6_data.append(xd[5][0])

		e1_data.append(e[0][0])
		e2_data.append(e[1][0])
		e3_data.append(e[2][0])
		e4_data.append(e[3][0])
		e5_data.append(e[4][0])
		e6_data.append(e[5][0])

		q1_data.append(q[0])
		q2_data.append(q[1])
		q3_data.append(q[2])
		q4_data.append(q[3])
		q5_data.append(q[4])
		q6_data.append(q[5])

		da1_data.append(da[0][0])
		da2_data.append(da[1][0])
		da3_data.append(da[2][0])
		da4_data.append(da[3][0])
		da5_data.append(da[4][0])
		da6_data.append(da[5][0])

		print(t)
		t_data.append(t)
		t += dt

	print(bot.fkine(q))
	print(x)
	# Graficas
	bot.plot(q)
	close_loop_plot(t_data, x1_data, x2_data, x3_data, x4_data, x5_data, x6_data, 
					xd1_data, xd2_data, xd3_data, xd4_data, xd5_data, xd6_data)
	velocities_plot(t_data, da1_data, da2_data, da3_data, da4_data, da5_data, da6_data)
	error_plot(t_data, e1_data, e2_data, e3_data, e4_data, e5_data, e6_data)
	angles_plot(t_data, q1_data, q2_data, q3_data, q4_data, q5_data, q6_data)

	plt.show()
	input()