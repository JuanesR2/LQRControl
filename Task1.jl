using LinearAlgebra
using ControlSystems
using DifferentialEquations
using Plots

# --------------------------------------------------
# BLOQUE COMÚN: Parámetros y Diseño LQR
# --------------------------------------------------

# 1) Parámetros del sistema
J  = 1.2           # Inercia del rotor
D  = 0.01           # Amortiguamiento
E  = 1.0           # Tensión interna (pu)
V  = 1.05           # Tensión de la barra infinita (pu)
X  = 0.3           # Reactancia síncrona (pu)
ω_s = 2π*60       # Velocidad síncrona (rad/s)

T_t = 1.0         # Constante de tiempo de la turbina
K_t = 1.0         # Ganancia de la turbina

# Punto de operación
delta0 = 0.5
Pe0 = (E*V/X)*sin(delta0)  # Potencia eléctrica en equilibrio
Pm0 = Pe0                # Potencia mecánica en equilibrio (se iguala)
y0  = Pm0 / K_t          # Apertura de válvula en equilibrio

# 2) Modelo linealizado (3 estados: Δδ, Δω, ΔPm) con entrada u = Δy
# Definimos las variables de desviación:
#   Δδ = δ - delta0,  Δω = ω - ω_s,  ΔPm = Pm - Pm0
#
# Las ecuaciones linealizadas son:
#   Δδ̇   = Δω
#   J Δω̇ = ΔPm - (E*V/X)*cos(delta0)*Δδ - D Δω
#   T_t ΔPṁ = -ΔPm + K_t Δy
A3 = [
    0                     1                     0;
    -(E*V/X)*cos(delta0)/J   -D/J         1/J;
    0                     0                -1/T_t
]
B3 = [
    0;
    0;
    K_t/T_t
]

# Salidas: se usa la identidad
C3 = Matrix(I, 3, 3)
D3 = zeros(3, 1)

# 3) Creación del sistema lineal y diseño del LQR
sys  = ss(A3, B3, C3, D3)
# Matriz de peso Q (penaliza errores en δ, ω y Pm) y R (esfuerzo de control)
Q    = diagm([5.0, 5.0, 5.0])
R    = 2.0
K_lqr, S, eigvals = lqr(sys, Q, R)

println("Ganancia LQR K = ", K_lqr)
println("Autovalores en lazo cerrado = ", eigvals)

# --------------------------------------------------
# BLOQUE DE SIMULACIÓN: Función helper para simular y graficar
# --------------------------------------------------
function sim_and_plot(closed_loop_func; 
    x0 = [delta0 + 0.1, ω_s, Pm0], 
    tspan = (0.0, 100.0)
)
    # 1) Definir y resolver el problema ODE
    prob = ODEProblem(closed_loop_func, x0, tspan)
    sol  = solve(prob, RK4(), saveat=0.1)

    # 2) Extraer las trayectorias de los estados
    δ_array  = sol[1, :]  # delta(t)
    ω_array  = sol[2, :]  # omega(t)
    Pm_array = sol[3, :]  # Pm(t)
    t_array  = sol.t      # Instantes de tiempo

    # 3) Calcular P_e(t) a partir de δ(t). 
    #    Es la misma fórmula no lineal que usas en 'closed_loop_stepPm!':
    Pe_array = (E*V/X) .* sin.(δ_array)

    # 4) Graficar los cuatro valores: δ, f, Pm y Pe

    # Ángulo del rotor δ(t)
    p1 = plot(t_array, δ_array,
        label="δ (rad)", xlabel="Tiempo (s)", ylabel="δ (rad)",
        lw=2, title="Ángulo del Rotor")

    # Frecuencia en Hz = ω/(2π)
    p2 = plot(t_array, ω_array./(2π),
        label="f (Hz)", xlabel="Tiempo (s)", ylabel="f (Hz)",
        lw=2, title="Frecuencia")

    # Potencia mecánica Pm
    p3 = plot(t_array, Pm_array,
        label="Pm (pu)", xlabel="Tiempo (s)", ylabel="Pm (pu)",
        lw=2, title="Potencia Mecánica")

    # Potencia eléctrica Pe
    p4 = plot(t_array, Pe_array,
        label="Pe (pu)", xlabel="Tiempo (s)", ylabel="Pe (pu)",
        lw=2, title="Potencia Eléctrica")

    # 5) Combinar los cuatro gráficos
    return plot(p1, p2, p3, p4, layout=(4,1), size=(800,600))
end


function closed_loop_stepPm!(du, x, p, t)
    # x = [δ, ω, Pm]
    δ  = x[1]
    ω  = x[2]
    Pm = x[3]

    # Calcular las desviaciones respecto al punto de operación
    Δδ  = δ - delta0
    Δω  = ω - ω_s
    ΔPm = Pm - Pm0

    # Ley de control LQR: u = -K_lqr * [Δδ; Δω; ΔPm]
    u = -K_lqr * [Δδ; Δω; ΔPm]
    y = y0 + u[1]  # La señal de control aplicada a la turbina

    # Modelo no lineal de la máquina: potencia eléctrica
    Pe = (E*V/X)*sin(δ)


    # Ecuaciones no lineales del sistema:
    du[1] = ω - ω_s
    du[2] = (1/J)*(Pm - Pe) - (D/J)*(ω - ω_s)
    du[3] = (1/T_t)*(-Pm + K_t*y)
end  

# Ejecuta la simulación y muestra las gráficas
sim_and_plot(closed_loop_stepPm!)
