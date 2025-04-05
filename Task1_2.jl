############################################################################
# ESCENARIO 2: STEP en la potencia eléctrica (carga)
############################################################################

ΔPe_step = -0.4    # Magnitud del escalón en Pe (pu)
t_step   = 50.0   # Instante del escalón

function closed_loop_stepPe!(du, x, p, t)
    # x = [δ, ω, Pm]
    δ  = x[1]
    ω  = x[2]
    Pm = x[3]

    # Calculamos las desviaciones
    Δδ  = δ - delta0
    Δω  = ω - ω_s
    ΔPm = Pm - Pm0

    # Ley de control LQR: u = -K_lqr * [Δδ; Δω; ΔPm]
    u = -K_lqr * [Δδ; Δω; ΔPm]
    y = y0 + u[1]

    # Modelo no lineal de la máquina: potencia eléctrica base
    basePe = (E*V/X)*sin(δ)
    # Se añade un escalón en la potencia eléctrica a partir de t >= t_step
    Pe_dist = basePe + (t >= t_step ? ΔPe_step : 0.0)

    # Ecuaciones no lineales:
    du[1] = ω - ω_s
    du[2] = (1/J)*(Pm - Pe_dist) - (D/J)*(ω - ω_s)
    du[3] = (1/T_t)*(-Pm + K_t*y)
end

# Ejecuta la simulación y muestra las gráficas
sim_and_plot(closed_loop_stepPe!)
