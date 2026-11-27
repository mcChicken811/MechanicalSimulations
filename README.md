# MechanicalSimulations

**Linkage Lab** is a browser app for building planar mechanisms out of bars, springs, gears and racks. It derives their equations of motion automatically with Lagrangian mechanics and simulates them in real time.

- You draw the mechanism. The app picks generalized coordinates, builds the kinetic and potential energy symbolically, and shows Lagrange's equations, M(q) q̈ = f(q, q̇) + Q + Φ_qᵀλ, as typeset math.
- Closed loops, locked axes, meshing gears, rack-and-pinion pairs, fixed angles between bars and joined bodies become constraints, handled with Lagrange multipliers.
- An optional ground that parts land on, bounce off and slide along, with adjustable restitution and friction.
- Motors that drive gears with a constant torque, or one that falls off with speed like a DC motor.
- The equations are integrated with 4th-order Runge–Kutta (RK4) with a 1 ms step. While it runs, you can drag any part to pull on it.
- Optional joint and gear friction turns energy into heat, which you see as joints glowing.

The examples include a **human leg swing** built from the biomechanics literature:
- **Knee:** O'Connor et al.'s crossed four-bar linkage of the cruciate ligaments (Proc. IMechE H, 1989), with its sagittal ligament and bone-link lengths as tabulated by Hernaiz Alzamora et al. (Eur. J. Anat., 2017).
- **Segments:** Winter's lengths, masses and centres of mass for a 1.75 m, 70 kg adult.
- **Swing:** passive, as in Mochon & McMahon's ballistic walking (J. Biomech., 1980).
- **Extension stop:** the posterior capsule, modelled as a tension-only spring that goes taut at full extension.

The sources and every assumption are listed in the comment above `humanLeg` in `src/ui/presets.ts`.

There is also a **whole human hand**, flattened into the plane of the palm as in planar grasping models. All five digits bend at every joint, but within the drawing plane: the fingers curl towards the thumb and the thumb towards the fingers.
- **Bones:** phalanx lengths and widths, and the knuckle positions on the palm, from Cerruti et al. (IEEE ICRA 2015), scaled to a 19 cm hand.
- **Masses:** each phalanx is a cylinder and the palm a block, at Dempster's hand density (the method of Tsakonas et al., 2026).
- **Joint stops:** each joint has a volar plate, a tension-only spring that stops it bending backwards. The wrist's collateral ligaments limit its sideways tilt.
- **Damping:** loading the hand sets the air drag to 2 s⁻¹ to stand in for soft-tissue damping. Other examples set it back to 0.

The sources and every assumption are in the comment above `humanHand`. Bars lighter than 5 g, such as ligaments and the anchor tabs for the volar plates, are drawn thin.

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
| Draw rack | K | Drag to draw a rack, a toothed bar. If it passes along a gear, it snaps tangent and meshes with it. Put its joints on rails (Slider, X) to make it slide. |
| Draw spring | S | Drag between joints. An end in empty space is anchored to the ground. Set the stiffness and rest length in the inspector, and tick **Tension only** for a cable, ligament or tendon that goes slack when short. |
| Gears | G | Click a centre, then drag out the radius. If the gear touches another gear or a rack, it snaps into mesh with it. |
| Pin | F | Click a joint, then click a bar, rack or gear to pin the joint onto it, or empty space to pin it to the ground. |
| Slider | X | Click a joint, then the body its rail is on (or empty space for a ground rail), then click to set the rail's direction. Then click where the rail ends on one side and on the other, or press Enter for no ends. |
| Free joint | N | Removes a joint's constraint: a pin, a rail, or a pin on a body. |
| Mass | M | Click a joint to hang a 1 kg mass on it. Click on a gear to pin a weight to it, or in empty space for a free mass. |
| Lock gear | L | Click a gear to stop it turning. Click again to unlock it. |
| Motor | O | Click a gear to put a motor on it. Click again to remove it. Set its torque and no-load speed in the gear's inspector. |
| Fix angle | A | Click one bar, then another, to hold the angle between them at its drawn value. Click the same pair again to free it. |
| Join bodies | J | Click two bodies (bars, racks or gears) to make them one rigid body. Click the same pair again to separate them. |
| Delete | D | Click a joint, bar, rack, spring or gear to delete it. |

Bars and springs that start or end on a gear are pinned to the gear and turn with it.

**Pins and rails.** The Pin and Slider tools work in steps. While a tool is waiting for your next click, the hint bar says what to do; Esc cancels. Clicking the chosen joint again removes its constraint.
- **Pin:** a joint pinned onto a body moves rigidly with it, at the place it was drawn: on a bar, a rack or a gear, or on the ground.
- **Slider:** puts a joint on a straight rail through where it was drawn. The direction snaps to 15° steps, and to the bar's own direction or square to it when close. You can type an exact angle in the joint's inspector.
- **Ground rails** stay put and can be at any angle. A rail at 0° or 90° behaves as the old lock y / lock x.
- **Rails on a body** turn and move with that body. That's how a crank pin slides along a slotted lever, as in the **Quick-return mechanism** example.
- **In the equations:** a pin on a bar adds two equations, and a rail adds one. If the mechanism already holds the joint that way, the extra equations are dropped with a warning.
- **Range:** after the direction, click where the rail ends on one side, then on the other. The ends snap to 5 cm when snap is on. Press Enter after one click for a stop on one side only, or straight away for a rail without ends. If both clicks fall on one side of the joint, the nearer end is set at the joint, so the joint always starts inside its range. You can also type the range and the stops' restitution in the joint's inspector.
- **End stops:** past an end, the joint hits a stop, with the same contact model as the ground. The stop is a spring-damper scaled by the effective mass along the rail, applied as a generalized force along ∂s/∂q, where s is the joint's travel along the rail. On a body's rail, the stop pushes the body back as well. The equations panel lists each slider's stops. The **Bead on a swinging rod, with end stops** example shows them working.

A rack is a rigid body like a bar, with the same mass and inertia, and bars attach to its two end joints. A gear meshing with it rolls along its pitch line without slipping, which turns rotation into sliding: the gear centre's position ξ along the rack and its rotation φ satisfy ξ + R(φ − θ_rack) = const. Its centre also stays a distance R from the rack. You can tick **Toothed** in a bar's inspector to turn it into a rack, or untick it to turn a rack back into a bar.

A fixed angle adds the constraint θ_b − θ_a = α, with α the angle as drawn. Two bars that share a joint are welded there into one rigid piece, for example an L-shaped or bent arm. A purple double arc at the joint shows the angle in degrees. Bars that don't touch are joined by a dashed line and stay at their drawn angle to each other wherever they move. If the mechanism already holds that angle, as for two sides of a rigid triangle, the constraint is dropped with a warning, because a duplicate would make the equations singular.

Joining two bodies makes them one rigid body: they can no longer turn or move relative to each other. This works for any pair of bars, racks and gears, whether or not they touch. Examples are a crank arm fixed to a gear, a pinion fixed to a rack, or a bent lever made of two bars. Joined bodies are drawn in the same colour: the colour of the group's lowest-numbered bar, or of its lowest-numbered gear if the group has no bars. Each body has a frame: a bar's frame is its first joint and its angle, and a gear's frame is its centre and its rotation. Joining body b to body a then adds up to three constraints: θ_b − θ_a = α, and r_b − r_a = Rot(θ_a)(u, v), with α, u and v taken from the drawing. Constraints the mechanism already enforces are left out automatically. For example, a gear joined to a bar on its own axle only needs to turn with the bar. If all three are already enforced, a warning says the bodies already move as one.

**Selecting, copying and pasting groups.** In Select mode, drag on empty space to draw a box. This selects every joint inside it, every bar, rack and spring with both ends inside, and every gear whose centre is inside. Ctrl/⌘+A selects everything. Then:

- Drag any selected part to move the whole group.
- Ctrl/⌘+C copies the group, Ctrl/⌘+X cuts it, and Delete removes it. Esc deselects.
- Ctrl/⌘+V pastes a copy centred on the cursor. You can paste as many times as you like.

A paste keeps every constraint among the copied parts: ground pins and rails, joints pinned on copied gears, gear and rack meshes, fixed angles and joined bodies. Constraints to parts that weren't copied are left out. For example, a joint pinned on a gear you didn't copy becomes a plain joint. The pasted parts stay selected, ready to drag into place. Copy and paste also work on a single selected part.

The inspector lets you change masses, positions, spring stiffness and rest length, gear radius, tooth count and mass, which side a rack's teeth are on, which gears and racks mesh, which bars have a fixed angle, and which bodies are joined.

A **tension-only** spring pulls when it is stretched beyond its rest length and goes slack when it is shorter, with no force and no energy: V = ½ k max(0, ℓ − ℓ₀)². While slack, it is drawn as a dashed line.

Other controls:
- Undo with Ctrl/Cmd+Z.
- Right-drag or middle-drag to pan, and use the mouse wheel to zoom. You can zoom in to millimetre scale; the grid and the scale bar follow.
- **Fit view** re-centres the mechanism.

### Running it

- **Play** / **Pause** (Space) start and stop the simulation. **Step** advances one frame. **Reset** (R) goes back to editing.
- While it runs, **drag any bar, joint or gear** to pull it with a spring force. A red arrow shows the spring's pull, which points at the cursor and grows with the stretch. The status bar shows the same value as |F|. The grab also damps the point's motion, like a steady hand, and that damping isn't part of the arrow. You can click a spring to change its stiffness without stopping.
- **Select a joint** to draw the path it moves along, as an orange trail. Select it before pressing Play, or click it while the simulation runs. The trail keeps about the last 24 000 points (a point is added each time the joint moves about a pixel), and its oldest end fades out. Selecting another joint, or pressing Reset, starts a new trail.
- The settings let you turn gravity on or off, add air drag, change how strongly a drag pulls, and change the playback speed. Untick **Show labels** to hide the names of joints, bars, springs and gears, which helps with big mechanisms like the 30-link ring chain. Tick **Follow the mechanism** to keep moving parts in view.
- The status bar shows the time, the total energy E = T + V, and the constraint error max|Φ|. With no air drag and no friction, E should stay constant.

### Equations panel

The top-right panel updates as you edit. It shows the generalized coordinates and degrees of freedom, one equation of motion per coordinate, and the constraints. Collapsible sections list the kinetic and potential energy and every parameter with its value. For a big mechanism, deriving and typesetting can take a moment. The panel then fills with green to show progress, and the new equations appear when it is done. An equation too long to typeset quickly (over about 4 kB of LaTeX, as in the 30-link ring chain) shows a **Show** button instead.

### Friction and heating

Tick **Friction & heating** to add viscous friction in pin joints, rails and gear teeth. It is modelled with a Rayleigh dissipation function. The energy friction removes heats each joint, which glows from red to white and cools by Newton's law of cooling. The status bar shows the heat produced, W, and E + W, which should stay constant when air drag is off.

### Motors

A motor turns its gear with a torque τ, given in N·m; a positive torque turns it anticlockwise. The motor itself is fixed to the ground. With a no-load speed Ω, the torque falls in a straight line from τ at rest to zero at Ω, as it does for a DC motor. Above Ω the motor brakes. A no-load speed of 0 gives a constant torque whatever the speed.

The torque enters the gear's equation of motion as the generalized force Q_φ = τ − b φ̇, with b = τ/Ω, so the equations panel shows it. To drive a bar, such as a crank, join a gear to it with **Join bodies**. You can change the torque and the speed while the simulation runs. The status bar adds the motor's work, W_m, to the energy budget, so E + W − W_m stays constant.

Example: **Strandbeest leg driven by a crank motor**. Select the foot, joint G, before pressing Play to draw its walking path.

Another example: **Strandbeest with four legs, gear-coupled and motor-driven**. Four Jansen legs stand one after another, each crank a quarter turn from the next, so the legs step in turn. Each crank carries a gear, and coupling rods join pins at the same place on neighbouring gears. Each pair of gears then forms a parallelogram, so all four cranks turn together, like a locomotive's side rods. A motor on the last gear drives them all. The frame is held still, so the legs step in place.

### Ground contact

Tick **Ground contact** to add a floor. Joints, bar ends and gear rims land on it, bounce and slide.

The ground is a terrain line through waypoints, level beyond the first and the last. A new ground starts flat, just below the machine, with waypoints across it and a metre or more beyond.
- **Select it** by clicking it in Select mode. Its waypoints show as handles.
- **Drag a waypoint** up or down to shape hills and dips, or set its x and height in the inspector.
- **Double-click** the ground to add a waypoint. **Delete** removes the selected one; a ground keeps at least two.

The ground always stays under the machine while you edit. A waypoint dragged up stops where the ground would touch a joint or gear rim. Moving a part below the ground, by dragging a joint for example, lowers the waypoints around it until the part is above the ground again.

**Contact on a terrain.** Each joint and gear rim is checked against the terrain segment beneath it, using that segment's normal and direction. Friction tracks distance along the terrain line, so its hold doesn't jump at a corner. A bar can't cross a straight stretch before one of its ends does, but it can rest across a peak with both ends in the air. So each peak (a waypoint where the terrain turns downhill) is also checked against the bars, and pushes a bar up where it pokes above it.

- **Restitution** sets how bouncy impacts are: 1 bounces back to the full height, 0 stops dead. A dropped mass rebounds to e² of its drop height.
- **Friction μ** holds a point where it landed (static friction) until the sideways force reaches μ times the normal force, then lets it slide. A block on a slope stays put when μ > tan α and otherwise slides at g(sin α − μ cos α).

Contact is a stiff spring and damper at each touching point, applied as a generalized force Q = Jᵀ F. The spring's stiffness is scaled by the effective mass the contact pushes against, worked out from the mass matrix and the constraints. The same idea sets contact softness in the MuJoCo physics engine. So every contact responds at the same rate (250 rad/s) whatever it carries. A 5 g fingertip and a 70 kg leg are equally stable, and anything resting on the ground sinks only about 0.16 mm.

The damper is calibrated so the restitution you set is the one you get. It never pulls the point down, so an uncalibrated damper would bounce higher than the textbook formula predicts. Near the ground each 1 ms step is split into four (or eight for very dead impacts), because contact forces switch on and off within a few milliseconds. Energy lost in impacts and sliding is counted in the heat W, so E + W stays constant. The equations panel describes the contact model in its own section. These forces are computed each step and are not part of the symbolic equations.

Tick **Follow the mechanism** to have the view pan along with something that travels, like the rimless wheel.

Examples:
- **Rimless wheel walking down a slope:** McGeer's (1990) simplest passive walker. It walks with no motor, at a steady pace set by the slope.
- **Box dropped onto the floor**.
- **Chain dropped onto the floor**.

### Limitations

- Everything is in a plane, and bars are uniform rigid rods.
- Gears are external spur gears and straight racks. There are no ring gears.
- Parts only collide with the ground, not with each other. The ground is a terrain line through waypoints, and very steep bars, over 60°, only touch it at their ends. Contact is slightly soft: resting parts sink about 0.16 mm, and fast impacts a few millimetres.
- Springs have no mass, and friction is viscous (proportional to speed).
- Motors push against the ground. A motor on a gear that rides on a moving part does not push back on that part.

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
