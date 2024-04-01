# Tarea 3: Seguimiento de trayectoria de un carro Car-Like (controles velocidades)

import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt 

# Funcion para graficar la respuesta de los estados en Lazo Cerrado
def close_loop_plot(t_array, data1, data2, data3):

	fig1, ax1 = plt.subplots()
	ax1.set_title('Respuesta del sistema controlado y1 vs y2')
	ax1.set_ylim(-20, 20)
	ax1.set_xlim(-20, 20)

	ax1.plot(data1[0], data2[0], 'o', color='red', label='x_0')
	ax1.plot(data1[-1], data2[-1], 'o', color='green', label='x_f')
	ax1.plot(data1, data2, '--', color='black')
	ax1.legend()
	ax1.grid()

	fig2, ax2 = plt.subplots()
	ax2.set_title('Respuesta del sistema controlado')
	# ax2.set_ylim(-2, 2)
	# ax2.set_xlim(-0.01, 10)

	ax2.plot(t_array, data1, label='xc1')
	ax2.plot(t_array, data2, label='xc2')
	ax2.plot(t_array, data3, label='xc3')

	ax2.legend()

# Funcion para graficar la respuesta de los controles
def control_plot(t_array, u1, u2):

	fig3, ax3 = plt.subplots()
	ax3.set_title('Respuesta de los controles u1 y u2')
	# ax3.set_ylim(-20, 4)
	# ax3.set_xlim(-0.01, 10)

	ax3.plot(t_array, u1, label='u1')
	ax3.plot(t_array, u2, label='u2')

	ax3.legend()

# Funcion para graficar la respuesta de los errores
def error_plot(t_array, e1, e2):

	fig4, ax4 = plt.subplots()
	ax4.set_title('Respuesta de los errores')
	# ax4.set_ylim(-10, 2)
	# ax4.set_xlim(-0.01, 10)

	ax4.plot(t_array, e1, label='e1')
	ax4.plot(t_array, e2, label='e2')

	ax4.legend()

# Funcion para graficar la respuesta de las salidas y referencias
def output_plot(t_array, y1, y2, ref1, ref2):

	fig5, ax5 = plt.subplots()
	ax5.set_title('Respuesta de las salidas y referecnias')
	# ax5.set_ylim(-2, 1.5)
	# ax5.set_xlim(-0.01, 10)

	ax5.plot(t_array, y1, label='y1')
	ax5.plot(t_array, y2, label='y2')
	ax5.plot(t_array, ref1, '--', label='ref1')
	ax5.plot(t_array, ref2, '--', label='ref2')

	ax5.legend()

# Funcion principal...
if __name__ == '__main__':

	# Condiciones iniales
	d = 2 # Distancia del eje de giro del robot al punto de control

	# x = 12 # 1ras condiciones inciales...............
	# y = 15
	# theta = np.pi/2

	# x = -1 # 2das condiciones inciales............
	# y = 2
	# theta = np.pi

	x = -7 # 3ras condiciones inciales.............
	y = 10
	theta = 3*np.pi/2

	# Condiciones iniciales
	t = 0.0

	X = np.array([[x],
				  [y],
				  [theta]])

	ref = np.array([[5],
					[6]])

	dref = np.array([[0],
					 [0]])

	# Ganancias K
	K = np.array([[2, 0],
				  [0, 3]])

	
	# Arreglos para guardar datos
	x1_c_data = []
	x2_c_data = []
	x3_c_data = []

	y1_data = []
	y2_data = []

	ref_1_data = []
	ref_2_data = []

	e_1_data = []
	e_2_data = []

	u1_data = []
	u2_data = []

	# Inicio de ciclo
	dt = 0.01
	t_array = []
	while t <= 10:

		# Obtenemos controles

		ref_1_data.append(ref[0][0])
		ref_2_data.append(ref[1][0])

		Y = np.array([[X[0][0] + d*np.cos(theta)],
				  	  [X[1][0] + d*np.sin(theta)]])

		errors = Y - ref

		e_1_data.append(errors[0][0])
		e_2_data.append(errors[1][0])

		M_inv = np.array([[np.cos(X[2][0]), np.sin(X[2][0])],
						  [-np.sin(X[2][0]/d), np.cos(X[2][0]/d)]])

		V = -K@errors # Controles Auxiliares

		# Control por controles auxiliares
		u = M_inv @ (dref + V)

		u1_data.append(u[0][0])
		u2_data.append(u[1][0])

		# Sistema en Lazo Cerrado
		G = np.array([[np.cos(X[2][0]), 0],
				  	  [np.sin(X[2][0]), 0],
				  	  [0, 1]])

		X = X + (G @ u) * dt

		x1_c_data.append(X[0][0])
		x2_c_data.append(X[1][0])
		x3_c_data.append(X[2][0])

		y1_data.append(Y[0][0])
		y2_data.append(Y[1][0])

		# Tiempo
		t_array.append(t)
		t += dt

	close_loop_plot(t_array, x1_c_data, x2_c_data, x3_c_data)
	control_plot(t_array, u1_data, u2_data)
	error_plot(t_array, e_1_data, e_2_data)
	output_plot(t_array, y1_data, y2_data, ref_1_data, ref_2_data)
	plt.show()