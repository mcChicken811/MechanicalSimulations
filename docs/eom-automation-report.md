# Automated Derivation of Equations of Motion from Lagrangian Mechanics

*Research summary — 2026-11-24*

## 1. Executive summary

The standard way to automate equations of motion is to treat q and q̇ as independent symbols, build the mass matrix from position Jacobians, and keep the equations in mass-matrix form M(q) q̈ = f(q, q̇). Code is then generated through common-subexpression elimination (CSE), and the linear system is solved numerically at each time step. Every serious tool follows this pattern, from SymPy and Maple to CasADi and JAX.

For a system specified by particle positions rᵢ(q), kinetic energy T and potential energy V, two routes fit:

- **Symbolic (SymPy):** best up to roughly 10 degrees of freedom (DOF), when readable equations matter.
- **Automatic differentiation (JAX or CasADi):** better for larger systems, optimization or gradients.

A 15-line SymPy reference implementation (Section 6) was checked against SymPy's built-in `LagrangesMethod` on a double pendulum and matched exactly.

## 2. Problem statement

The goal is to generate equations of motion automatically from a kinematic description, with no algebra done by hand.

**Inputs**

- Generalized coordinates q = (q₁, …, qₙ) and their velocities q̇.
- The position of each particle as a function of the coordinates, rᵢ(q), with mass mᵢ.
- The potential energy V(q). Kinetic energy follows from the positions as T = ½ Σ mᵢ |ṙᵢ|².

**Output:** the equations of motion in a form a numerical integrator can use:

$$
M(q)\,\ddot{q} = f(q, \dot{q}, t)
$$

Here M is the n × n symmetric positive-definite mass matrix, and f collects the gravitational, velocity-dependent (Coriolis and centrifugal) and applied generalized forces.

## 3. Methodology

The method has four steps. Each one avoids a specific failure that makes naive symbolic derivation slow or intractable.

### 3.1 Treat q and q̇ as independent symbols

Do not model q as a function of time and differentiate L with respect to t literally. The total time derivative has a closed form:

$$
\frac{d}{dt}\frac{\partial L}{\partial \dot{q}}
= \frac{\partial^2 L}{\partial \dot{q}\,\partial \dot{q}}\,\ddot{q}
+ \frac{\partial^2 L}{\partial \dot{q}\,\partial q}\,\dot{q}
+ \frac{\partial^2 L}{\partial \dot{q}\,\partial t}
$$

Substituting it into the Euler–Lagrange equations leaves only partial derivatives, which every computer algebra system (CAS) and autodiff library computes reliably:

$$
M\,\ddot{q} = \frac{\partial L}{\partial q}
- \frac{\partial^2 L}{\partial \dot{q}\,\partial q}\,\dot{q}
- \frac{\partial^2 L}{\partial \dot{q}\,\partial t},
\qquad M = \frac{\partial^2 L}{\partial \dot{q}^2}
$$

The last term vanishes when the rᵢ do not depend on time explicitly (scleronomic systems).

### 3.2 Build the mass matrix from Jacobians

With Jᵢ = ∂rᵢ/∂q, the velocity of each particle is ṙᵢ = Jᵢ q̇, so:

$$
M(q) = \sum_i m_i\, J_i^{\mathsf{T}} J_i,
\qquad T = \tfrac{1}{2}\,\dot{q}^{\mathsf{T}} M(q)\,\dot{q}
$$

This is cheaper than squaring and expanding |ṙᵢ|², and it produces much smaller expressions. It also gives the robotics form M q̈ + C(q, q̇) q̇ + g(q) = Q. Here g = ∂V/∂q, and C is built from the Christoffel symbols of M:

$$
c_{ijk} = \tfrac{1}{2}\left(
\frac{\partial M_{kj}}{\partial q_i} + \frac{\partial M_{ki}}{\partial q_j} - \frac{\partial M_{ij}}{\partial q_k}
\right)
$$

### 3.3 Keep mass-matrix form; never invert M symbolically

This is the most common mistake. The symbolic inverse of M grows combinatorially with n, and users report that it is intractable even for a 7-DOF robot arm. Generate code for M and f separately, then solve M q̈ = f numerically at each time step with a Cholesky or LU factorization.

### 3.4 Use common-subexpression elimination and code generation, not general simplification

General-purpose `simplify()` is slow and unpredictable on large expressions. The standard pipeline is:

1. Apply a few cheap, targeted rewrites, such as `trigsimp` on M and collecting terms.
2. Run common-subexpression elimination (CSE) to pull out repeated subterms.
3. Generate compiled code: C, Cython, Numba or JAX.
4. Hand the resulting function to a standard ODE integrator.

## 4. Extensions

The same pipeline handles forces, damping and constraints with small additions to the right-hand side.

| Extension | Treatment | Effect on the equations |
| --- | --- | --- |
| Applied (non-conservative) forces Fᵢ on particle i | Project onto the coordinates with the same Jacobians | Add Q = Σ Jᵢᵀ Fᵢ to f |
| Viscous damping | Rayleigh dissipation function D(q, q̇) | Add −∂D/∂q̇ to f |
| Holonomic constraints | Preferred: choose minimal coordinates that satisfy them identically | No change; n shrinks |
| Constraints that cannot be eliminated | Lagrange multipliers λ with constraint Jacobian Φ_q | Gives a differential-algebraic system (DAE); needs index reduction or Baumgarte stabilization |
| Explicit time dependence in rᵢ(q, t) | Keep the ∂²L/∂q̇∂t term | Adds a time-dependent forcing term |

## 5. Industry landscape

Five families of tools are in use. They differ mainly in whether they keep closed-form expressions or evaluate the dynamics numerically.

| Approach | Representative tools | Strengths | Limitations | Typical use |
| --- | --- | --- | --- | --- |
| Symbolic Lagrange or Kane, then code generation | SymPy `physics.mechanics` + PyDy, Maple/MapleSim, Mathematica, MotionGenesis (successor to Autolev) | Readable equations; symbolic linearization; fast generated code | Expressions grow with DOF; derivation time grows quickly | Small to medium systems, controller design. Kane's method is often preferred because it avoids differentiating energy. |
| Equation-based acausal modelling | Modelica (Dymola, OpenModelica), ModelingToolkit.jl + Symbolics.jl | The compiler does index reduction, tearing and code generation | No built-in Lagrangian front end in MTK; Euler–Lagrange equations are built by hand | Industrial system-level simulation |
| Symbolic graph + automatic differentiation | CasADi | AD of T and V gives M and the forcing term; integrates with optimal control solvers | Large expanded expressions are slow to build | Trajectory optimization, model-predictive control |
| Pure automatic differentiation, no CAS | JAX, PyTorch | No expression swell; just-in-time compiled; differentiable end to end | No closed-form equations to inspect | Research, learned dynamics |
| Recursive numerical algorithms (Featherstone) | Pinocchio, RBDL, Drake, MuJoCo | O(n) algorithms: RNEA for inverse dynamics, CRBA for M, ABA for forward dynamics | Rigid-body trees; not stated in terms of rᵢ(q), T and V | The standard in robotics |

SymPy 1.13 replaced `JointsMethod` and `Body` with a `System` class and `RigidBody`/`Particle`. It still provides `LagrangesMethod` and `KanesMethod` directly.

## 6. Validation

The methodology in Section 3 was implemented in SymPy and tested on a planar double pendulum (masses m₁, m₂; lengths l₁, l₂; gravity g).

```python
import sympy as sp

def lagrange_eom(r_list, masses, V, q, qd):
    """Return (M, f) such that M(q) q̈ = f(q, q̇) for a scleronomic system."""
    n = len(q)
    M = sp.zeros(n, n)
    for r, m in zip(r_list, masses):
        J = r.jacobian(q)                 # J_i = ∂r_i/∂q
        M += m * J.T * J                  # M = Σ m_i J_iᵀ J_i
    M = M.applyfunc(sp.trigsimp)
    L = (qd.T * M * qd)[0] / 2 - V
    p = sp.Matrix([L]).jacobian(qd).T     # ∂L/∂q̇
    f = sp.Matrix([L]).jacobian(q).T - p.jacobian(q) * qd
    return M, f

# Double pendulum
q1, q2, w1, w2 = sp.symbols('q1 q2 w1 w2')
m1, m2, l1, l2, g = sp.symbols('m1 m2 l1 l2 g', positive=True)
q, qd = sp.Matrix([q1, q2]), sp.Matrix([w1, w2])
r1 = sp.Matrix([l1*sp.sin(q1), -l1*sp.cos(q1)])
r2 = r1 + sp.Matrix([l2*sp.sin(q2), -l2*sp.cos(q2)])
V = m1*g*r1[1] + m2*g*r2[1]
M, f = lagrange_eom([r1, r2], [m1, m2], V, q, qd)

# Numerical function with CSE; at each step solve M q̈ = f
fn = sp.lambdify((q, qd, (m1, m2, l1, l2, g)), [M, f], cse=True)
```

**Result.** The generated mass matrix and forcing vector are:

$$
M = \begin{bmatrix}
l_1^2 (m_1 + m_2) & l_1 l_2 m_2 \cos(q_1 - q_2) \\
l_1 l_2 m_2 \cos(q_1 - q_2) & l_2^2 m_2
\end{bmatrix},
\quad
f = \begin{bmatrix}
-l_1\left[(m_1 + m_2) g \sin q_1 + l_2 m_2 \dot{q}_2^2 \sin(q_1 - q_2)\right] \\
l_2 m_2\left[-g \sin q_2 + l_1 \dot{q}_1^2 \sin(q_1 - q_2)\right]
\end{bmatrix}
$$

Both match the output of `sympy.physics.mechanics.LagrangesMethod` exactly: the symbolic difference simplifies to zero. The CSE-generated function evaluated and solved without issue. With q = (0.3, −0.2), q̇ = (0.1, 0.5) and parameters (m₁, m₂, l₁, l₂, g) = (1, 2, 1, 0.5, 9.81), it gives q̈ = (−8.39, 18.63) rad/s².

## 7. Recommendations and next steps

1. **Start symbolic with SymPy** using the Section 6 implementation. It accepts rᵢ(q), mᵢ and V directly, gives inspectable equations, and is enough for systems up to roughly 10 DOF.
2. **Generate code for M and f separately** with CSE, and solve M q̈ = f numerically. Never invert M symbolically.
3. **Add a JAX backend** behind the same interface for larger systems or when gradients are needed. M is the Hessian of T with respect to q̇, computed by AD, so there is no expression swell.
4. **Cross-validate the two backends** on reference systems: single and double pendulum, cart-pole, spherical pendulum. Also check energy conservation over long integrations.
5. **Defer constraints and DAEs** until a system needs them. Prefer minimal coordinates; use Lagrange multipliers with Baumgarte stabilization only when elimination is impractical.

## 8. References

- SymPy `System` class (1.14 docs mirror): https://eksamensvedlegg.it.ntnu.no/Felles/sympy-docs-html-1.14.0/modules/physics/mechanics/api/system.html
- SymPy Joints framework: https://osgeo.cn/sympy/modules/physics/mechanics/joints.html
- SymPy deprecated classes (`JointsMethod`, `Body`): https://osgeo.cn/sympy/modules/physics/mechanics/api/deprecated_classes.html
- SymPy `LagrangesMethod`: https://dokk.org/documentation/sympy/sympy-1.11.1/modules/physics/mechanics/lagrange.html
- PyDy: https://github.com/pydy/pydy
- PyDy System docs: https://pydy.readthedocs.io/en/latest/system.html
- Branlard & Geisler, *A symbolic framework to obtain mid-fidelity models of flexible multibody systems*, Wind Energy Science 7 (2022): https://wes.copernicus.org/articles/7/2351/2022/
- Julia Discourse, Symbolics & MTK pipeline for Lagrangians: https://discourse.julialang.org/t/symbolics-mtk-pipeline-lagrangians/129730
- Julia Discourse, Symbolics/MTK/DifferentialEquations and the Lagrangian approach: https://discourse.julialang.org/t/symbolics-jl-modelingtoolkit-jl-differentialequations-jl-and-the-lagrangian-approach/89629
- Damped pendulum with Symbolics/MTK: https://cooperrc.github.io/dynamics/tutorials/09_damped-pendulum
- CasADi discussion on large symbolic dynamics: https://www.github.com/casadi/casadi/discussions/3652
- *Symbolic Multibody Methods for Real-Time Simulation of Railway Vehicles* (arXiv:1706.01657): https://arxiv.org/pdf/1706.01657
- CasADi overview: https://en.wikipedia.org/wiki/CasADi
