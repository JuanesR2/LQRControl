############################################################################
# ESCENARIO 1: STEP en la potencia mecánica
############################################################################

# Parámetros de la perturbación
ΔP_step = 0.2      # Magnitud del escalón en Pm (pu)
t_step  = 50.0     # Tiempo en el que se activa el escalón

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

    # Se añade un escalón en la potencia mecánica a partir de t >= t_step
    Pm_dist = Pm + (t >= t_step ? ΔP_step : 0.0)

    # Ecuaciones no lineales del sistema:
    du[1] = ω - ω_s
    du[2] = (1/J)*(Pm_dist - Pe) - (D/J)*(ω - ω_s)
    du[3] = (1/T_t)*(-Pm + K_t*y)
end

# Ejecuta la simulación y muestra las gráficas
sim_and_plot(closed_loop_stepPm!)
