# Tarea 3: Seguimiento de trayectoria de un carro Car-Like (controles velocidad y aceleracion)

import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt 

# Funcion para graficar la respuesta de los estados en Lazo Cerrado
def close_loop_plot(t_array, data1, data2, data3, data4):

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
    ax2.plot(t_array, data4, label='xc4')

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
def error_plot(t_array, e1, e2, e3, e4):

    fig4, ax4 = plt.subplots()
    ax4.set_title('Respuesta de los errores')
    # ax4.set_ylim(-10, 2)
    # ax4.set_xlim(-0.01, 10)

    ax4.plot(t_array, e1, label='e1')
    ax4.plot(t_array, e2, label='e2')
    ax4.plot(t_array, e3, label='e3')
    ax4.plot(t_array, e4, label='e4')

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

    # x = 12 # 1ras condiciones inciales...............
    # y = 15
    # theta = np.pi/2
    # x_1 = 1 # Para las referecnias
    # x_2 = 9
    # y_1 = 1
    # y_2 = 7
    # tau = 10 # Tiempo para llegar a la referencia

    # x = -1 # 2das condiciones inciales............
    # y = 2
    # theta = np.pi
    # x_1 = 2 # Para las referecnias
    # x_2 = 5
    # y_1 = -1
    # y_2 = -4
    # tau = 10 # Tiempo para llegar a la referencia

    x = -7 # 3ras condiciones inciales.............
    y = 10
    theta = 3*np.pi/2
    x_1 = -3 # Para las referecnias
    x_2 = 8
    y_1 = 0
    y_2 = 7
    tau = 10 # Tiempo para llegar a la referencia

    # v = 0.0001
    v = 5

    # Condiciones iniciales
    t = 0.0

    X = np.array([[x],
                  [y],
                  [theta],
                  [v]])

    mx = (x_2 - x_1) / tau
    my = (y_2 - y_1) / tau

    ddref = np.array([[0],
                     [0]])

    # Ganancias K
    K = np.array([[2, 3, 0, 0],
                  [0, 0, 4, 5]])

    
    # Arreglos para guardar datos
    x1_c_data = []
    x2_c_data = []
    x3_c_data = []
    x4_c_data = []

    y1_data = []
    y2_data = []

    ref_1_data = []
    ref_2_data = []

    e_1_data = []
    e_2_data = []
    e_3_data = []
    e_4_data = []

    u1_data = []
    u2_data = []

    # Inicio de ciclo
    dt = 0.01
    t_array = []
    while t <= 20:

        # Obtenemos controles
        if t < 5:

            ref = np.array([[x_1],
                            [y_1]])
            dref = np.array([[0],
                             [0]])

        elif t < tau+5:

            ref = np.array([[mx*(t-5) + x_1],
                            [my*(t-5) + y_1]])
            dref = np.array([[mx],
                             [my]])

        else:

            ref = np.array([[x_2],
                            [y_2]])

            dref = np.array([[0],
                             [0]])

        ref_1_data.append(ref[0][0])
        ref_2_data.append(ref[1][0])

        Y = np.array([[X[0][0]],
                      [X[1][0]]])

        # Errores
        e1 = Y[0][0] - ref[0][0]
        e2 = X[3][0] * np.cos(X[2][0]) - dref[0][0]
        e3 = Y[1][0] - ref[1][0]
        e4 = X[3][0] * np.sin(X[2][0]) - dref[1][0]

        errors = np.array([[e1],
                           [e2],
                           [e3],
                           [e4]])

        e_1_data.append(errors[0][0])
        e_2_data.append(errors[1][0])
        e_3_data.append(errors[2][0])
        e_4_data.append(errors[3][0])

        M_inv = np.array([[-np.sin(X[2][0])/X[3][0], np.cos(X[2][0])/X[3][0]],
                          [np.cos(X[2][0]), np.sin(X[2][0])]])

        V = -K@errors # Controles Auxiliares

        # Control por controles auxiliares
        u = M_inv @ (ddref + V)

        if u[0][0] > 5:
            u[0][0] = 5
        if u[0][0] < -5:
            u[0][0] = -5

        if u[1][0] > 2:
            u[1][0] = 2
        if u[1][0] < -2:
            u[1][0] = -2

        u1_data.append(u[0][0])
        u2_data.append(u[1][0])

        # Sistema en Lazo Cerrado
        F = np.array([[X[3][0] * np.cos(X[2][0])],
                      [X[3][0] * np.sin(X[2][0])],
                      [0],
                      [0]])
        G = np.array([[0, 0],
                      [0, 0],
                      [1, 0],
                      [0, 1]])

        X = X + (F + (G @ u)) * dt

        x1_c_data.append(X[0][0])
        x2_c_data.append(X[1][0])
        x3_c_data.append(X[2][0])
        x4_c_data.append(X[3][0])

        y1_data.append(Y[0][0])
        y2_data.append(Y[1][0])

        # Tiempo
        t_array.append(t)
        t += dt

    close_loop_plot(t_array, x1_c_data, x2_c_data, x3_c_data, x4_c_data)
    control_plot(t_array, u1_data, u2_data)
    error_plot(t_array, e_1_data, e_2_data, e_3_data, e_4_data)
    output_plot(t_array, y1_data, y2_data, ref_1_data, ref_2_data)
    plt.show()