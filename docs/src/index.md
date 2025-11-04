# State-to-state transfer in a two-level quantum system


The goal of this example is misimise the cost

$$\mathcal{C} = |\langle \uparrow |\psi(t_f)\rangle |^2 + \frac{p_0}2 \int_0^{t_f}u(t)^2\mathrm{d}t$$
while we steer the system bettwen to states, $|\uparrow\;\rangle$ to $|\downarrow\;\rangle$, described by following Hamiltonian:

```math
H(t) = \frac{\Delta}{2} \sigma_z + \frac{u(t)}{2} \sigma_x
```
where 
 - The final time $t_f$ is set equalt to $2\pi/\sqrt{1+\Delta^2}$;
 - $\psi(t_f)$ is the state at the final time $t_f$;
 - $p_0$ is a factor to adjust the relative weight of the two terms in the cost
 - $\Delta$ represents the frequency offset; 
 - $u(t) \in [-u_0,u_0]$ is the control;
 - $\sigma_z,\sigma_x$ are Pauli matrices. 
 
The dynamics is governed by the Schrödinger equation
$$
i\hbar |\dot{\psi}\rangle = H(t) |\psi\rangle.
$$ 
However, ths states here belong to a Hilber space $\mathcal{H} = \mathbb{C}^2$, which is not easy to visuzalize. To improve that, we can tranform in Bloch representation to have a real system. It can be shown that

$$
\begin{cases}
\dot x = -\Delta \cdot y\\
\dot y = \Delta \cdot x - u \cdot z\\
\dot z = u \cdot y
\end{cases}$$
with $|\uparrow\;\rangle = (0,0,1)$ to $|\downarrow\;\rangle= (0,0,-1)$. Furthermore, we will set $\Delta = 0.5$ and $p_0= 0.1$.


```@setup main
using OptimalControl  
using OrdinaryDiffEq  
using LinearAlgebra


q0 = [0.0, 0.0, 1.0] 
qf = [0.0, 0.0, -1.0] 
Δ = 0.5
tf = 2 * π / (√( 1 + Δ^2))
p_zero = 0.1


ocp1 = @def begin
    t ∈ [0, tf], time
    q = [x, y, z] ∈ R³, state
    u ∈ R, control
    
    q(0) == q0 
    ∂(x)(t) == - Δ * y(t)
    ∂(y)(t) == Δ * x(t) - u(t) * z(t)
    ∂(z)(t) == u(t) * y(t)
    sum((q(tf) - qf).^2) + (p_zero / 2) * ∫(u(t)^2) → min
end
```

# IGNORE 

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
