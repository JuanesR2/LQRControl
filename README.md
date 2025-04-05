# Simulación de Fallas con Control LQR en un Sistema Generador Síncrono ⚙️⚡

Este repositorio contiene una simulación en Julia de un generador síncrono conectado a una barra infinita, controlado mediante un regulador LQR. Se incluyen simulaciones ante fallas pequeñas mediante perturbaciones iniciales en el ángulo del rotor.

## 📌 Descripción

Se modela un generador síncrono con turbina, considerando tres variables de estado:

- Ángulo del rotor (δ)
- Velocidad angular (ω)
- Potencia mecánica (Pm)

Se linealiza el sistema alrededor de un punto de operación y se diseña un controlador **LQR** sobre el modelo linealizado. La simulación se realiza con el modelo no lineal, y se estudia el desempeño del sistema en lazo cerrado ante perturbaciones.

---

## ⚙️ Tecnologías usadas

- [Julia](https://julialang.org/)
- [ControlSystems.jl](https://github.com/JuliaControl/ControlSystems.jl)
- [DifferentialEquations.jl](https://diffeq.sciml.ai/stable/)
- [Plots.jl](http://docs.juliaplots.org/)

---

## 📐 Modelo del sistema

Ecuaciones no lineales del generador con turbina:

- δ̇ = ω - ω_s
- ω̇ = (1/J)(Pm - Pe) - (D/J)(ω - ω_s)
- Pṁ = (1/T_t)(-Pm + K_t y)

Donde:
- Pe = (E * V / X) * sin(δ)
- y es la apertura de la válvula, regulada por el control LQR

---

## 🧠 Controlador LQR

Se diseña un controlador lineal óptimo LQR sobre el modelo linealizado:

\[
\dot{x} = A x + B u, \quad u = -Kx
\]

Con matrices de ponderación definidas en `Q` y `R` para ajustar la respuesta del sistema.

---

## 📈 Simulación

La simulación se realiza con el modelo **no lineal** en lazo cerrado, integrando las ecuaciones diferenciales con perturbaciones iniciales.

### Variables graficadas:
- Ángulo del rotor (δ)
- Frecuencia (ω en Hz)
- Potencia mecánica (Pm)
- Potencia eléctrica (Pe)

---

## ▶️ Cómo ejecutar

Asegúrate de tener Julia instalado. Luego, ejecuta los siguientes comandos:

```julia
using Pkg
Pkg.add(["LinearAlgebra", "ControlSystems", "DifferentialEquations", "Plots"])
