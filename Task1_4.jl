############################################################################
# ESCENARIO 4: Pulso en la Potencia Mecánica
############################################################################
# En este escenario se introduce una falla (pulso) en la potencia mecánica.
# La falla se activa en el intervalo [t_on, t_off].

ΔP_pulse = -0.2    # Amplitud del pulso (falla)
t_on  = 20.0      # Tiempo de inicio de la falla
t_off = 50.0      # Tiempo en que la falla termina

function closed_loop_pulsePm!(du, x, p, t)
    # x = [δ, ω, Pm]
    δ  = x[1]
    ω  = x[2]
    Pm = x[3]
    
    # Calculamos las desviaciones respecto al punto de operación
    Δδ = δ - delta0
    Δω = ω - ω_s
    ΔPm = Pm - Pm0

    # Ley de control LQR: u = -K_lqr * [Δδ; Δω; ΔPm]
    u = -K_lqr * [Δδ; Δω; ΔPm]
    y = y0 + u[1]  # La señal de control aplicada a la turbina

    # Potencia eléctrica (modelo no lineal de la máquina)
    Pe = (E * V / X) * sin(δ)
    
    # Falla: pulso en la potencia mecánica
    in_pulse = (t >= t_on && t < t_off) ? 1.0 : 0.0
    Pm_dist = Pm + ΔP_pulse * in_pulse

    # Modelo no lineal:
    du[1] = ω - ω_s
    du[2] = (1/J) * (Pm_dist - Pe) - (D/J) * (ω - ω_s)
    du[3] = (1/T_t) * (-Pm + K_t * y)
end


sim_and_plot(closed_loop_pulsePm!)

