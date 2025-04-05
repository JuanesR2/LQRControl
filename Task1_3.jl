############################################################################
# ESCENARIO 3: Rampa en la potencia mecánica
############################################################################
# Se introduce una rampa en Pm a partir de t >= t_ramp_on.
slope     = 0.002    # Pendiente de la rampa (pu/s)
t_ramp_on = 10.0    # Tiempo a partir del cual inicia la rampa

function closed_loop_rampPm!(du, x, p, t)
    # x = [δ, ω, Pm]
    δ  = x[1]
    ω  = x[2]
    Pm = x[3]

    # Cálculo de las desviaciones
    Δδ = δ - delta0
    Δω = ω - ω_s
    ΔPm = Pm - Pm0

    # Ley de control LQR: u = -K_lqr*[Δδ; Δω; ΔPm]
    u = -K_lqr * [Δδ; Δω; ΔPm]
    y = y0 + u[1]  # La señal de control aplicada a la turbina

    # Modelo no lineal de la máquina
    Pe = (E*V/X)*sin(δ)

    # Rampa en Pm a partir de t_ramp_on
    ramp = (t >= t_ramp_on) ? slope*(t - t_ramp_on) : 0.0
    Pm_dist = Pm + ramp

    # Ecuaciones no lineales:
    du[1] = ω - ω_s
    du[2] = (1/J)*(Pm_dist - Pe) - (D/J)*(ω - ω_s)
    du[3] = (1/T_t)*(-Pm + K_t*y)
end

# Ejecuta la simulación y muestra el gráfico
sim_and_plot(closed_loop_rampPm!)
