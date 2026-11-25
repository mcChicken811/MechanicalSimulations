# MechanicalSimulations

**Linkage Lab** is a browser app for building planar mechanisms out of bars, springs, gears and racks. It derives their equations of motion automatically with Lagrangian mechanics and simulates them in real time.

- You draw the mechanism. The app picks generalized coordinates, builds the kinetic and potential energy symbolically, and shows Lagrange's equations, M(q) q̈ = f(q, q̇) + Q + Φ_qᵀλ, as typeset math.
- Closed loops, locked axes, meshing gears, rack-and-pinion pairs, fixed angles between bars and joined bodies become constraints, handled with Lagrange multipliers.
- The equations are integrated with 4th-order Runge–Kutta (RK4) with a 1 ms step. While it runs, you can drag any part to pull on it.
- Optional joint and gear friction turns energy into heat, which you see as joints glowing.

The method behind the equation generation is described in [docs/eom-automation-report.md](docs/eom-automation-report.md).

## How to run

You only need Git and a web browser:

```bash
git clone https://github.com/mcChicken811/MechanicalSimulations.git
```

Then open `MechanicalSimulations/dist/index.html` in your browser, for example by double-clicking it. That's all: the page is one self-contained file and needs no install, server or internet connection.

To change the code, follow the installation steps below.

## Installation

You need [Node.js](https://nodejs.org/) **22.12 or newer**, which includes npm.

```bash
git clone https://github.com/mcChicken811/MechanicalSimulations.git
cd MechanicalSimulations
npm install
```

### Open it as a single file

```bash
npm run build
```

This writes one self-contained file, `dist/index.html`, which is committed to the repository so that it works straight after cloning. The scripts, styles and math fonts are all inside it. Open it directly in a browser by double-clicking it; it needs no server and no internet connection, so you can copy it anywhere. Run `npm run build` again after changing the source.

`src/index.html` is the source page and won't work if you open it directly, because browsers can't run the TypeScript it loads.

### Development server

```bash
npm run dev
```

Open the address it prints (usually http://localhost:5173). The page reloads when you edit files in `src/`.

### Other scripts

| Command | What it does |
|---|---|
| `npm test` | Runs the test suite (Vitest). |
| `npm run typecheck` | Type-checks the project without building. |
| `npm run preview` | Serves the built `dist/` folder locally. |

## Using the simulation

The first time you open the app, it shows one leg of Theo Jansen's Strandbeest. After that, it remembers your last design in the browser. Use **Examples…** to load other mechanisms, or **Clear** to start from scratch. Examples marked *top view* lie in the horizontal plane, such as the two car steering linkages, so loading one turns gravity off; loading any other example turns it back on.

### Building a mechanism

Pick a tool in the left toolbar (or press its key) and click or drag on the canvas.

| Tool | Key | How to use it |
|---|:-:|---|
| Select | V | Click a part to edit it in the inspector (bottom right). Drag joints to reshape the mechanism. Drag on empty space to draw a selection box. |
| Draw bar | B | Drag, or click twice. Start or end on an existing joint to pin bars together. |
| Draw rack | K | Drag to draw a rack, a toothed bar. If it passes along a gear, it snaps tangent and meshes with it. Lock its joints on rails (X or Y) to make it slide. |
| Draw spring | S | Drag between joints. An end in empty space is anchored to the ground. Set the stiffness and rest length in the inspector. |
| Gears | G | Click a centre, then drag out the radius. If the gear touches another gear or a rack, it snaps into mesh with it. |
| Pin to ground | F | Click a joint to fix it in place. Click again to free it. |
| Lock x / Lock y | X / Y | The joint slides on a vertical or horizontal rail. |
| Free joint | N | Removes a joint's constraint. |
| Mass | M | Click a joint to hang a 1 kg mass on it. Click on a gear to pin a weight to it, or in empty space for a free mass. |
| Lock gear | L | Click a gear to stop it turning. Click again to unlock it. |
| Fix angle | A | Click one bar, then another, to hold the angle between them at its drawn value. Click the same pair again to free it. |
| Join bodies | J | Click two bodies (bars, racks or gears) to make them one rigid body. Click the same pair again to separate them. |
| Delete | D | Click a joint, bar, rack, spring or gear to delete it. |

Bars and springs that start or end on a gear are pinned to the gear and turn with it.

A rack is a rigid body like a bar, with the same mass and inertia, and bars attach to its two end joints. A gear meshing with it rolls along its pitch line without slipping, which turns rotation into sliding: the gear centre's position ξ along the rack and its rotation φ satisfy ξ + R(φ − θ_rack) = const. Its centre also stays a distance R from the rack. You can tick **Toothed** in a bar's inspector to turn it into a rack, or untick it to turn a rack back into a bar.

A fixed angle adds the constraint θ_b − θ_a = α, with α the angle as drawn. Two bars that share a joint are welded there into one rigid piece, for example an L-shaped or bent arm. A purple double arc at the joint shows the angle in degrees. Bars that don't touch are joined by a dashed line and stay at their drawn angle to each other wherever they move. If the mechanism already holds that angle, as for two sides of a rigid triangle, the constraint is dropped with a warning, because a duplicate would make the equations singular.

Joining two bodies makes them one rigid body: they can no longer turn or move relative to each other. This works for any pair of bars, racks and gears, whether or not they touch. Examples are a crank arm fixed to a gear, a pinion fixed to a rack, or a bent lever made of two bars. Joined bodies are drawn in the same colour: the colour of the group's lowest-numbered bar, or of its lowest-numbered gear if the group has no bars. Each body has a frame: a bar's frame is its first joint and its angle, and a gear's frame is its centre and its rotation. Joining body b to body a then adds up to three constraints: θ_b − θ_a = α, and r_b − r_a = Rot(θ_a)(u, v), with α, u and v taken from the drawing. Constraints the mechanism already enforces are left out automatically. For example, a gear joined to a bar on its own axle only needs to turn with the bar. If all three are already enforced, a warning says the bodies already move as one.

**Selecting, copying and pasting groups.** In Select mode, drag on empty space to draw a box. This selects every joint inside it, every bar, rack and spring with both ends inside, and every gear whose centre is inside. Ctrl/⌘+A selects everything. Then:

- Drag any selected part to move the whole group.
- Ctrl/⌘+C copies the group, Ctrl/⌘+X cuts it, and Delete removes it. Esc deselects.
- Ctrl/⌘+V pastes a copy centred on the cursor. You can paste as many times as you like.

A paste keeps every constraint among the copied parts: ground pins and rails, joints pinned on copied gears, gear and rack meshes, fixed angles and joined bodies. Constraints to parts that weren't copied are left out. For example, a joint pinned on a gear you didn't copy becomes a plain joint. The pasted parts stay selected, ready to drag into place. Copy and paste also work on a single selected part.

The inspector lets you change masses, positions, spring stiffness and rest length, gear radius, tooth count and mass, which side a rack's teeth are on, which gears and racks mesh, and which bars have a fixed angle, and which bodies are joined.

Other controls:
- Undo with Ctrl/Cmd+Z.
- Right-drag or middle-drag to pan, and use the mouse wheel to zoom.
- **Fit view** re-centres the mechanism.

### Running it

- **Play** / **Pause** (Space) start and stop the simulation. **Step** advances one frame. **Reset** (R) goes back to editing.
- While it runs, **drag any bar, joint or gear** to pull it with a spring force. A red arrow shows the spring's pull, which points at the cursor and grows with the stretch. The status bar shows the same value as |F|. The grab also damps the point's motion, like a steady hand, and that damping isn't part of the arrow. You can click a spring to change its stiffness without stopping.
- **Select a joint** to draw the path it moves along, as an orange trail. Select it before pressing Play, or click it while the simulation runs. The trail keeps about the last 24 000 points (a point is added each time the joint moves about a pixel), and its oldest end fades out. Selecting another joint, or pressing Reset, starts a new trail.
- The settings let you turn gravity on or off, add air drag, change how strongly a drag pulls, and change the playback speed. Untick **Show labels** to hide the names of joints, bars, springs and gears, which helps with big mechanisms like the 30-link ring chain.
- The status bar shows the time, the total energy E = T + V, and the constraint error max|Φ|. With no air drag and no friction, E should stay constant.

### Equations panel

The top-right panel updates as you edit. It shows the generalized coordinates and degrees of freedom, one equation of motion per coordinate, and the constraints. Collapsible sections list the kinetic and potential energy and every parameter with its value. For a big mechanism, deriving and typesetting can take a moment. The panel then fills with green to show progress, and the new equations appear when it is done. An equation too long to typeset quickly (over about 4 kB of LaTeX, as in the 30-link ring chain) shows a **Show** button instead.

### Friction and heating

Tick **Friction & heating** to add viscous friction in pin joints, rails and gear teeth. It is modelled with a Rayleigh dissipation function. The energy friction removes heats each joint, which glows from red to white and cools by Newton's law of cooling. The status bar shows the heat produced, W, and E + W, which should stay constant when air drag is off.

### Limitations

- Everything is in a plane, and bars are uniform rigid rods.
- Gears are external spur gears and straight racks. There are no ring gears, and parts do not collide with each other.
- Springs have no mass, and friction is viscous (proportional to speed).
- Nothing is motor-driven. Mechanisms move under gravity, springs and your pulls.

## Project layout

```
src/
  cas/          small computer-algebra kernel: trig polynomials, LaTeX, code generation
  mechanics/    design model, equation derivation, RK4 simulator, tests
  ui/           canvas rendering, equations panel, example mechanisms
  main.ts       app controller (tools, input, inspector)
docs/           background report on automating equations of motion
```

## License

[MIT](LICENSE)
