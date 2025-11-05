# State-to-state transfer in a two-level quantum system


The goal of this example is misimise the cost

$$\mathcal{C} = |\langle \uparrow |\psi(t_f)\rangle |^2 + \frac{\gamma}2 \int_0^{t_f}u(t)^2\mathrm{d}t$$
while we steer the system bettwen to states, $|\uparrow\;\rangle$ to $|\downarrow\;\rangle$, described by following Hamiltonian:

```math
H(t) = \frac{\Delta}{2} \sigma_z + \frac{u(t)}{2} \sigma_x
```
where 
 - The final time $t_f$ is set equalt to $2\pi/\sqrt{1+\Delta^2}$;
 - $\psi(t_f)$ is the state at the final time $t_f$;
 - $\gamma$ is a factor to adjust the relative weight of the two terms in the cost
 - $\Delta$ represents the frequency offset; 
 - $u(t) \in [-u_0,u_0]$ is the control;
 - $\sigma_z,\sigma_x$ are Pauli matrices. 
 
The dynamics is governed by the Schrödinger equation
$$
i\hbar |\dot{\psi}\rangle = H(t) |\psi\rangle.
$$ 
However, ths states here belong to a Hilber space $\mathcal{H} = \mathbb{C}^2$, which is not easy to visuzalize. To improve that, we can tranform in Bloch representation to have a real system. It can be shown applinng $\dot x_j = [H,\sigma_j ]$, where $[ \cdot,\cdot]$ denote the Lie bracket. In other words,

$$
\begin{cases}
\dot x = -\Delta \cdot y\\
\dot y = \Delta \cdot x - u \cdot z\\
\dot z = u \cdot y
\end{cases}$$
with $|\uparrow\;\rangle = (0,0,1)$ to $|\downarrow\;\rangle= (0,0,-1)$. Furthermore, we will set $\Delta = 0.5$ and $p_0= 0.1$.


```@example main
using OptimalControl  
using OrdinaryDiffEq  
using LinearAlgebra
using NLPModelsIpopt


q0 = [0.0, 0.0, 1.0] 
qf = [0.0, 0.0, -1.0] 
Δ = 0.5
tf = 2 * π / (√( 1 + Δ^2))
γ = 0.1


ocp1 = @def begin
    t ∈ [0, tf], time
    q = [x, y, z] ∈ R³, state
    u ∈ R, control
    
    q(0) == q0 
    ∂(x)(t) == - Δ * y(t)
    ∂(y)(t) == Δ * x(t) - u(t) * z(t)
    ∂(z)(t) == u(t) * y(t)
    sum((q(tf) - qf).^2) + (γ / 2) * ∫(u(t)^2) → min
end
nothing # hide
```

## Direct solve

```@example main
N = 100
direct_sol1 = solve(ocp1, grid_size=N)
nothing #hide
```

```@example main
using Plots
plt = plot(direct_sol1)
nothing #hide
```



## Indirect solve

We can also solve the problem with shooting thechincs. Using the Pontryagin’s Maximum Principle, the pseudo-Hamiltonian is given by

```math
H_p(x, p, u) = \Delta(p_yx - p_x y) + u(p_z y - p_yz)+ \mathcal{V} \frac{\gamma}{2} u^2,
```

where $p = (p_x, p_y,p_z)$ is the costate vector. The optimal control is given by the maximization of $H_p$:  

```math
    u  = \frac{p_zy - p_yz}{\gamma}.
```

Define the packages:
```@example main
using OrdinaryDiffEq  
using MINPACK 
```

Define the control and Hamiltonian flow:

```@example main
# Control
u(q, p) = (p[3] * q[2] - p[2] * q[3]) / γ

# Hamiltonian flow
f = Flow(ocp1, u)
nothing # hide
```

The shooting function enforces the conditions:
```math
    S : \mathbb{R}^3 \longrightarrow \mathbb{R}^3, \\
    S(p_0) := p(t_f,q_0,p_0) + 2(q(t_f,x_0,p_f)-q_f)
```
```@example main
p0 = direct_sol1.costate(0); 

function shoot!(s, p0)
    qqf, pf = f(0, q0, p0, tf)
    s[1:3] .= pf + 2(qqf - qf)
    return nothing
end
s = similar(p0, 3)
shoot!(s, p0)
println("\nNorm of the shooting function: ‖s‖ = ", norm(s), "\n")
nothing # hide
```

We are now ready to solve the shooting equations:

```@example main
backend = AutoForwardDiff();

ξ = p0 # initial guess
nle! = (s, ξ) -> shoot!(s, ξ)
jnle! = (js, ξ) -> jacobian!(nle!, similar(ξ), js, backend, ξ)

indirect_sol = fsolve(nle!, jnle!, ξ; show_trace=true)
p0 = indirect_sol.x
nothing # hide
```

```@example main
shoot!(s, p0)
println("\nNorm of the shooting function: ‖s‖ = ", norm(s), "\n")
nothing # hide
```


Finally, we reconstruct and plot the solution obtained by the indirect method:

```@example main
flow_sol = f((0, tf), q0, p0)
plot!(plt, flow_sol, solution_label="(indirect)")
nothing # hide
```

## Trjajectorie in the Bloch sphere

```@example main
using Plots
x = []
y = []
z = []

for t in time_grid(flow_sol)
    push!(x, state(flow_sol)(t)[1])
    push!(y, state(flow_sol)(t)[2])
    push!(z, state(flow_sol)(t)[3])
end
gr()
θ = 0:0.01:π    
φ = 0:0.01:2π  
xs = [sin(t) * cos(p) for t in θ, p in φ]
ys = [sin(t) * sin(p) for t in θ, p in φ]
zs = [cos(t) for t in θ, p in φ]



p = plot(xs, ys, zs, 
    st=:surface, 
    color=:lightblue,
    alpha=0.5, 
    legend=false, 
    axis = nothing, 
    background_color=:transparent,
    grid=false,
)

plot!(x, y, z, lw=2, color=:blue, label="Trajectory")
scatter!([x[1]], [y[1]], [z[1]], markersize=2, color=:green, label="Start")  
scatter!([x[end]], [y[end]], [z[end]], markersize=2, color=:red, label="End") 
```



# Reproducibility 

```@setup main
using Pkg
using InteractiveUtils
using Markdown 

# Download links for the benchmark environment
function _downloads_toml(DIR)
    link_manifest = joinpath("assets", DIR, "Manifest.toml")
    link_project = joinpath("assets", DIR, "Project.toml")
    return Markdown.parse("""
    You can download the exact environment used to build this documentation:
    - 📦 [Project.toml]($link_project) - Package dependencies
    - 📋 [Manifest.toml]($link_manifest) - Complete dependency tree with versions
    """)
end
```

```@example main
_downloads_toml(".") # hider
```

```@raw html
<details style="margin-bottom: 0.5em; margin-top: 1em;"><summary>ℹ️ Version info</summary>
```

```@example main
versioninfo() # hide
```

```@raw html
</details>
```

```@raw html
<details style="margin-bottom: 0.5em;"><summary>📦 Package status</summary>
```

```@example main
Pkg.status() # hide
```

```@raw html
</details>
```

```@raw html
<details style="margin-bottom: 0.5em;"><summary>📚 Complete manifest</summary>
```

```@example main
Pkg.status(; mode = PKGMODE_MANIFEST) # hide
```

```@raw html
</details>
```
