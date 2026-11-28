# MechanicalSimulations

**Linkage Lab** is a browser app for building planar mechanisms out of bars, springs, gears and racks. It derives their equations of motion automatically with Lagrangian mechanics and simulates them in real time.

- You draw the mechanism. The app picks generalized coordinates, builds the kinetic and potential energy symbolically, and shows Lagrange's equations, M(q) q̈ = f(q, q̇) + Q + Φ_qᵀλ, as typeset math.
- Closed loops, locked axes, meshing gears, rack-and-pinion pairs and joined bodies become constraints, handled with Lagrange multipliers.
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

The first time you open the app, it shows one leg of Theo Jansen's Strandbeest. After that, it remembers your last design in the browser. Use **Examples…** to browse other mechanisms, or **Clear** to start from scratch.

**The examples gallery.** **Examples…** opens a panel in the middle of the screen with every example as a card, its drawing above its name, grouped into famous mechanisms and basics. A box at the top filters them by name or description.
- **Choosing a card** shows it on the right: a bigger drawing (drag with any mouse button to move it, scroll to zoom, double-click to fit; the *Labels* box in its corner shows or hides the names, and is remembered), its name, what it is and what to look for, and what it is made of. **Open** loads it; so do double-clicking a card and Enter.
- **Unsaved changes:** if your drawing has changes not saved to a file, opening an example asks first. You can save the design first, open without saving (Undo can still bring your drawing back until you reload), or cancel. Tick *Don't show this again* to stop the question for this session or for good.
- **What counts as saved:** saving or loading a design or project, or opening an example, counts as saved; any edit after that counts as a change. This is remembered across reloads.

 Examples marked *top view* lie in the horizontal plane, such as the two car steering linkages, so loading one turns gravity off; loading any other example turns it back on.

### Building a mechanism

Pick a tool in the left toolbar (or press its key) and click or drag on the canvas.

**Moving and the grid.** Dragging a selected joint, a group of selected parts or a ground waypoint moves it continuously, following the cursor's travel. A press only becomes a move once the cursor has gone 0.3 cm from where it was pressed, so a click never nudges anything.
- **Grid mode:** hold **Alt** (**Option** on a Mac) to move on the grid instead. A joint lands on a grid point, a group moves by whole grid steps, and a waypoint's height lands on a grid line.
- **Showing the grid:** the grid shows only while Alt / Option is held during something that snaps: a drag, drawing a part, placing a duplicate or a rail's ends. Its spacing follows the zoom, from 1 m down to 1 mm, and snapping uses the spacing you see.
- **Drawing and pasting** work the same way: free by default, snapped to the grid while Alt / Option is held. That covers joints, radii (to half the spacing), a propeller's blade line (to 15° steps), rail ends, and paste and duplicate offsets.

| Tool | Key | How to use it |
|---|:-:|---|
| Select | V | Click a part to edit it in the inspector (bottom right). Where parts overlap, clicking again in the same spot steps through everything there, from the top down, then wraps (the hint names what is picked, say "bar 1: 2 of 3 here"). Drag joints to reshape the mechanism. Drag on empty space to draw a selection box. |
| Dimension | U | Click a bar, rack or spring to show its length, a gear, wheel or propeller for its radius, or a joint for its position (x, y). Click it again to hide it. |
| Initial state | I | Click a part, then pick one of its coordinates (or a rate) from the list beside the mouse. Move the mouse to set it, with everything else following, and click to settle. Esc cancels. |
| Draw bar | B | Drag, or click twice. Start or end on an existing joint to pin bars together. |
| Draw rack | K | Drag to draw a rack, a toothed bar. If it passes along a gear, it snaps tangent and meshes with it. Put its joints on rails (Slider, X) to make it slide. |
| Draw spring | S | Drag between joints. An end in empty space is anchored to the ground. Set the stiffness and rest length in the inspector, and tick **Tension only** for a cable, ligament or tendon that goes slack when short. |
| Gears | G | Click a centre, then drag out the radius. If the gear touches another gear or a rack, it snaps into mesh with it. |
| Propeller | P | Click the centre, then the edge of a blade. The blades spin out of the page about an axis square to that line, and push along it. In empty space it's a free body; start on a joint to mount it there. |
| Wheel | W | Click a centre, then drag out the radius. A wheel is a gear without teeth: it meshes with nothing, but rolls on the ground. Start on a joint to mount it there; in empty space its axle is free, so it drops and rolls. |
| Pin | F | Click a joint, then click a bar, rack or gear to pin the joint onto it, or empty space to pin it to the ground. |
| Slider | X | Click a joint, then the body its rail is on (or empty space for a ground rail), then click to set the rail's direction. Then click where the rail ends on one side and on the other, or press Enter for no ends. |
| Free joint | N | Removes a joint's constraint: a pin, a rail, or a pin on a body. |
| Mass | M | Click a joint to hang a 1 kg mass on it. Click on a gear to pin a weight to it, or in empty space for a free mass. |
| Lock gear | L | Click a gear or wheel to stop it turning (a braked wheel). Click again to unlock it. |
| Lever | E | Click a motor to give it a control lever, worked from the dock at the bottom right. On a gear without a motor, a motor is added too. Click the motor again to remove the lever. |
| PID controller | C | Click a motor to put a PID controller on it, in place of its lever. Then click what it should measure and pick the quantity from the menu. Click the motor again to remove it. |
| Motor | O | Click a gear or wheel to put a motor on it. Click again to remove it. Set its torque and no-load speed in the gear's inspector. |
| Join bodies | J | Click two bodies (bars, racks or gears) to make them one rigid body, for example two bars welded into an L-shaped arm. Click the same pair again to separate them. (This replaces the old Fix angle tool: designs saved with fixed angles open with those bars joined.) |
| Delete | D | Click a joint, bar, rack, spring or gear to delete it. |

**Save & load.** The *Save & load* block in the side bar saves and opens files, in the browser's save dialog where it has one, otherwise as a download.
- **Design** (the drawing: bars, joints, gears, springs, joins, the ground):
  - **Save** writes a design file.
  - **Load** opens one in place of the drawing (one undo step).
  - **Import** adds one to the drawing: like a duplicate, its outline follows the mouse, a click places it (selected, ready to drag) and Esc cancels. Its ground isn't imported.
- **Project** (the drawing plus every side-panel setting, the beam colouring and the view):
  - **Save** writes a project file (Ctrl/⌘+S).
  - **Load** opens one, restoring the drawing, gravity, drag, pull stiffness, speed, labels, dimensions, follow, sound, friction, beam colouring, deflection with its E and I, breaking, and the pan and zoom.
- **Opening a project as a design:** Load and Import design also open project files, taking just their drawing.
- **Formats:** files are JSON tagged `linkage-lab-design` or `linkage-lab-project`. Opening one runs the same checks as the design kept in the browser, so files from older versions open too. The file name defaults to the example the drawing came from.

**Select is the default tool.** The app starts with it, and Reset, loading an example and Clear all go back to it. Esc cancels whatever is under way, such as a bar being drawn, a join or pin half done, a duplicate being placed or a box selection; pressed again with nothing under way, it switches back to Select.

Bars and springs that start or end on a gear are pinned to the gear and turn with it.

**Pins and rails.** The Pin and Slider tools work in steps. While a tool is waiting for your next click, the hint bar says what to do; Esc cancels. Clicking the chosen joint again removes its constraint.
- **Pin:** a joint pinned onto a body moves rigidly with it, at the place it was drawn: on a bar, a rack or a gear, or on the ground.
- **Slider:** puts a joint on a straight rail through where it was drawn. The direction snaps to 15° steps, and to the bar's own direction or square to it when close. You can type an exact angle in the joint's inspector.
- **Ground rails** stay put and can be at any angle. A rail at 0° or 90° behaves as the old lock y / lock x.
- **Rails on a body** turn and move with that body. That's how a crank pin slides along a slotted lever, as in the **Quick-return mechanism** example.
- **In the equations:** a pin on a bar adds two equations, and a rail adds one. If the mechanism already holds the joint that way, the extra equations are dropped with a warning.
- **Range:** after the direction, click where the rail ends on one side, then on the other. In grid mode (Alt / Option held) the ends snap to half the grid's spacing. Press Enter after one click for a stop on one side only, or straight away for a rail without ends. If both clicks fall on one side of the joint, the nearer end is set at the joint, so the joint always starts inside its range. You can also type the range and the stops' restitution in the joint's inspector.
- **End stops:** past an end, the joint hits a stop, with the same contact model as the ground. The stop is a spring-damper scaled by the effective mass along the rail, applied as a generalized force along ∂s/∂q, where s is the joint's travel along the rail. On a body's rail, the stop pushes the body back as well. The equations panel lists each slider's stops. The **Bead on a swinging rod, with end stops** example shows them working.

A rack is a rigid body like a bar, with the same mass and inertia, and bars attach to its two end joints. A gear meshing with it rolls along its pitch line without slipping, which turns rotation into sliding: the gear centre's position ξ along the rack and its rotation φ satisfy ξ + R(φ − θ_rack) = const. Its centre also stays a distance R from the rack. You can tick **Toothed** in a bar's inspector to turn it into a rack, or untick it to turn a rack back into a bar.

A **wheel** is a uniform disk like a gear, turning about its centre joint, with the same mass, inertia and motor options, and joints can be pinned on it. It has no teeth, so it can't mesh with a gear or rack, and a mesh with one is ignored with a warning. Its tyre touches the ground like any rim: it rolls, gripping with the ground's friction. A locked wheel is braked and skids. Wheels are drawn with a dark tyre and spokes, labelled w1, w2 and so on.

Joining two bodies makes them one rigid body: they can no longer turn or move relative to each other. This works for any pair of bars, racks and gears, whether or not they touch. Examples are a crank arm fixed to a gear, a pinion fixed to a rack, or a bent lever made of two bars. Joined bodies are drawn in the same colour: the colour of the group's lowest-numbered bar, or of its lowest-numbered gear if the group has no bars. Joined bodies, joined directly or through others, are placed as one rigid body. The group gets a single rotation coordinate: the angle θ of its first bar, or the rotation φ of its first gear if it has no bars. Every joint of the group then sits at r_origin + Rot(θ)(u, v), with the offsets u and v taken from the drawing.

So a group needs at most three coordinates and adds no constraints. Its other bars turn at θ plus a constant, and its gears turn with θ. Compared with joining bodies through constraints, the human hand example drops from 39 coordinates and 23 constraints to 16 coordinates and none, and it simulates about seven times faster.

**Dimensions.** The Dimension tool (U) shows a part's size on the drawing:
- **Bars, racks and springs:** the length, as a dimension line beside the part with extension lines, arrowheads and the value along it. A spring's is its current length, ℓ.
- **Gears, wheels and propellers:** the radius, as an arrow from the centre to the rim.
- **Joints:** the position, (x, y).

**Moving a dimension:** as in CAD, press on a dimension's line or text, or on its part, and drag. It only moves once the cursor has gone 0.3 cm, so a plain click still shows or hides it.
- **A length's dimension line** slides to pass through the cursor, nearer the part or farther from it, and to either side; its extension lines grow to reach it.
- **A radius** swings round the centre to point at the cursor, its text following on a leader beyond the rim.
- **A joint's label** moves to the cursor, with a thin leader back to the joint once it is away.

Dragging a part whose dimension isn't shown pulls one out. Each drag is one undo step.

Dimensions stay on the view, follow the parts while they move, and are saved with the design, including where they were dragged to. Copying parts copies their dimensions, and deleting a part removes its dimension.
- **Decimals:** two by default; hold **Alt** (**Option** on a Mac) for three, or **Shift** for one.
- **Show dimensions,** under Show labels in the settings, turns them all on or off. By default they show while editing and hide while running. The box remembers each choice separately, so ticking it while running keeps them showing on every run, even after a reload.

**Initial state.** A simulation starts from the drawing, at rest unless initial velocities are set. The Initial state tool (I) sets both without changing the mechanism's geometry:
1. **Click a part:** a joint, a bar or rack, a gear, wheel or propeller. A list beside the mouse shows the coordinates the part moves with directly, for example the angles of the bars it hangs from or a free joint's x and y, with their current values. Below them it lists the same coordinates' rates (a propeller's spin rate included).
2. **Pick one and move the mouse.**
   - **A coordinate:** the part is pulled by a spring from the point you clicked towards the cursor, shown as a red line, and moves by the equations of motion. RK4 integrates its real inertia, velocity terms and constraints, in 1 ms steps, each frame.
     - **What moves:** only the chosen coordinate and the coordinates the constraints tie to it, directly or through others. Every other coordinate is held still, its rate zero. So a crank pulled round a four-bar carries the coupler and rocker with it, while pulling a double pendulum's lower bar by its angle leaves the upper bar where it is.
     - **Only your pull acts:** gravity, springs and motors are left out while setting. The pull is scaled by the point's effective mass, so a light part and a heavy one follow alike, at about 3 Hz, damped so they settle rather than swing.
     - **Limits:** a part can only go where the mechanism lets it. A pull straight along a bar it hangs from does nothing, and at a limit of the motion it stops as close as it can get.
   - **A rate:** the mouse sets the part's velocity, shown as an orange arrow of the motion over half a second. Teal arrows show every joint's velocity, the other rates following so the constraints still hold.
3. **Click to settle,** or press Esc to cancel.
   - **A new pose becomes the drawing.** Every joint moves, but every length stays the same, so it is the same mechanism in a different position. Slider ranges are shifted with their joints, and a propeller's blade line turns with its hub.
   - **Rates are kept** with the design (the status bar says it starts moving), drawn as faint teal arrows while editing, and made consistent with the constraints when Play starts.
   - **"Clear all initial velocities"** in the list sets every rate back to zero.

Under the hood, the free coordinates F (the chosen one and those tied to it) follow M_FF q̈_F = f_F + J_Fᵀ F_pull − c M_FF q̇_F + Φ_Fᵀλ, with Φ_F q̈_F = γ, solved as one KKT system. Here F_pull = m (ω² (r_cursor − r) − 2ω ṙ), and m = 1/λ_max(J A Jᵀ) is the point's effective mass in its most mobile direction, A being the constrained inverse inertia. After each step the free coordinates are projected back onto the constraints, with the held ones untouched. Setting a rate works directly instead: the chosen rate follows the mouse, and the others are corrected minimally so the constraints still hold.

**Selecting, copying and pasting groups.** In Select mode, drag on empty space to draw a box. This selects every joint inside it, every bar, rack and spring with both ends inside, and every gear whose centre is inside. Ctrl/⌘+A selects everything. Then:

- Drag any selected part to move the whole group.
- Ctrl/⌘+C copies the group, Ctrl/⌘+X cuts it, and Delete removes it. Esc deselects.
- Ctrl/⌘+V pastes a copy centred on the cursor. You can paste as many times as you like.
- Ctrl/⌘+D (or **Duplicate** in the inspector) duplicates the selection. The copy follows the cursor as a dashed outline (on the grid while Alt / Option is held) until a left click places it there. Esc cancels, as do switching tools and pressing Play.

A paste keeps every constraint among the copied parts: ground pins and rails, joints pinned on copied gears, gear and rack meshes and joined bodies. Constraints to parts that weren't copied are left out. For example, a joint pinned on a gear you didn't copy becomes a plain joint. The pasted parts stay selected, ready to drag into place. Copy and paste also work on a single selected part.

**Number boxes** work as in Blender. Press on one and drag sideways to change its value continuously; a click without a drag opens it for typing.
- **Plain drag:** steps of 0.01, at 0.01 per pixel.
- **Alt (Option on a Mac):** shows the third decimal and moves 10× slower, 0.001 per pixel.
- **Shift:** rounds to one decimal and moves 10× faster, 0.1 per pixel.
- **Whole-number boxes** (teeth, blades) move by whole numbers.
- **While dragging:** the part updates live, even while it runs (for the values that can change then). The whole drag is one undo step, and Esc puts the value back.

The inspector lets you change masses, positions, spring stiffness and rest length, gear radius, tooth count and mass, which side a rack's teeth are on, which gears and racks mesh, and which bodies are joined.

A **tension-only** spring pulls when it is stretched beyond its rest length and goes slack when it is shorter, with no force and no energy: V = ½ k max(0, ℓ − ℓ₀)². While slack, it is drawn as a dashed line.

Other controls:
- Undo with Ctrl/Cmd+Z.
- Right-drag or middle-drag to pan, and use the mouse wheel to zoom. You can zoom in to millimetre scale; the scale bar, and the grid when it shows, follow.
- **The side bar** is a column of separate floating blocks, with the drawing showing and clickable between them.
  - **Always open:** the top block, with Play, Step, Reset, Undo, Fit view, Clear and the Examples gallery.
  - **Folded at first:** Tools, Simulation (gravity, air drag, pull stiffness, speed), View (labels, dimensions, follow), Beam forces, Friction & ground, and Sound. Click a heading to open or fold it; which are open is remembered.
  - **Summaries:** a folded heading still says what matters: the tool in use, the beam colouring, deflection and breaking that are on, and whether friction or the ground is on.
  - **Still reachable:** tool keys and the right-click list work whether or not their block is open.
- **Panels that get out of the way.** The equations panel starts folded (its open state is remembered).
  - **Sliding away:** every panel slides off its edge of the screen once the mouse has been away from it for 3 seconds, and a thin tab at that edge marks where it went:
    - the side bar to the left;
    - the equations panel to the right;
    - the inspector to the right;
    - the message bar up;
    - the status bar down.
  - **Bringing them back:** move the mouse to the panel's part of its edge and it slides back in, going again 3 seconds after the mouse leaves. Each part:
    - the side bar and the equations panel: the top of their edge, down to the panel's height or 40% of the screen;
    - the inspector: the bottom of the right edge;
    - the message bar and the status bar: the stretch of the top or bottom edge alongside them.
  - **Coming out by themselves:** the message bar comes out whenever there's something new to say, and the inspector whenever you pick a different part to inspect.
  - **Messages:** every message goes through one place, the message system (`src/ui/messages.ts`). There is the standing instruction for what you are doing (the tool, a pick half done, running or paused) and passing notices (copied, a bar broke, warnings), shown long enough to read before the instruction returns. Simulation errors also appear there in red, besides in the status bar.
  - **Never while in use:** a panel stays while the mouse is over it, while a press that began on it is held (dragging a slider or number box out past its edge), and while the keyboard is in one of its boxes or menus.
- **Right-click** the drawing for a search list of everything in the side panel: tools, Play, Step and Reset, every setting and switch (beam colouring, deflection, breaking, friction, ground…), sliders and number boxes, and the examples.
  - **Opening:** the list opens at the mouse with its search box ready, so just type. Every word must appear in an entry, and entries that start with what you typed come first. A check mark shows what is on, and shortcuts and current values show on the right.
  - **Keys:** ↑ ↓ choose and Enter runs, or click an entry; Esc, or a click elsewhere, closes it.
  - **Actions** (a tool, Play, an example, Fit view) run and close the list.
  - **Checkboxes and switch options** flip in place and the list stays open, so you can set several in a row. The list then shows things as they are now: ticking Friction & heating brings its sliders into it.
  - **Sliders and number boxes** are worked right in their row, with a slider or number box of the list's own that moves the side panel's along with it. A number box can be typed into or dragged sideways like every number box. Enter on one of these rows gives its control the keyboard, and Enter again goes back to searching.
  - **Panning:** right-dragging, even when it starts on the list, closes the list and pans as usual.
  - **On a part,** right-clicking selects the part, and before anything is typed the list shows only that part's own entries, under its name. Typing still searches everything, the part's entries included.
    - **While editing:** what the tools do to it, without picking the tool first. That covers joining it to another body, pinning, putting it on a rail, freeing it, a mass, a motor, locking, a lever and a PID controller. Its numbers can be set right there: mass, a bar's E and I, a joint's x and y, a motor's torque and speed, a spring's k and rest length. It also offers its dimension, making a bar a rack, its initial motion, duplicate and delete. Entries that need another click, such as choosing the body to join to, leave that tool active.
    - **While running:** measuring it and graphing any of its quantities and forces. For a bar there are also its bending moment, shear, axial force and deflection along it, its own E and I, and the beam colouring, deflection and breaking options.
- **Fit view** re-centres the mechanism.

### Running it

- **Play** / **Pause** (Space) start and stop the simulation. **Step** advances one frame. **Reset** (R) goes back to editing.
- While it runs, **drag any bar, joint or gear** to pull it with a spring force. A red arrow shows the spring's pull, which points at the cursor and grows with the stretch. The status bar shows the same value as |F|. The grab also damps the point's motion, like a steady hand, and that damping isn't part of the arrow. You can click a spring to change its stiffness without stopping.
- **Select a joint** to draw the path it moves along, as an orange trail. Select it before pressing Play, or click it while the simulation runs. The trail goes back the last 20 seconds (a point is added each time the joint moves about a pixel), and its oldest end fades out. Selecting another joint, or pressing Reset, starts a new trail.
- The settings let you turn gravity on or off, add air drag, change how strongly a drag pulls, and change the playback speed. Untick **Show labels** to hide the names of joints, bars, springs and gears, which helps with big mechanisms like the 30-link ring chain. Tick **Follow the mechanism** to keep moving parts in view.
- The status bar shows the time, the total energy E = T + V, and the constraint error max|Φ|. With no air drag and no friction, E should stay constant.

### Measuring while it runs

While the simulation runs or is paused, the inspector measures the selected part instead of showing its settings. Pause and click a part to measure it; click it again, or empty space, to let go. While running, pulling a part selects it too. Every value is live and given to three decimals.
- **Bars, racks, gears, wheels and propellers** (rigid bodies):
  - the centre of mass x_G, y_G, and the angle θ (a gear's turn φ, a propeller's hub yaw);
  - their rates ẋ_G, ẏ_G and the angular velocity;
  - momentum p_x, p_y = m v_G;
  - angular momentum about the centre of mass, L_G = I ω, and about the origin, L_O = r_G × p + I ω;
  - kinetic energy ½ m v_G² + ½ I ω², and gravitational potential energy m g y_G.

  A propeller also shows its blade spin rate, and its kinetic energy is in its 3D form.
- **Joints:** x, y, ẋ, ẏ and speed; for a joint carrying a mass, also its momentum, angular momentum about the origin, kinetic energy and potential energy.
- **Springs:** length, rate of stretch, extension, tension (or push) and elastic energy.

**Joint forces.** The measurements also show the forces in the joints, worked out from the motion by inverse dynamics.
- **For a bar, rack, gear, wheel or propeller:** the force at each of its connections, as its size and its direction (°). That covers its joints, a slider pressing on a rail on it, gear teeth, a rack's teeth and welds to bodies it is joined with. It also shows the couple of a weld or of a locked gear.
- **For a joint:** the force each body puts on it, and its support's reaction (the ground, or a rail).
- **Every other load** on the part is listed and drawn too, where it acts:
  - weight, at the centre of mass;
  - air drag (with Air drag on): −c m v_G at the centre of mass and the couple −c I ω, which is what the simulation's mass-proportional drag amounts to;
  - pin friction couples, and rail friction;
  - ground contact, where it touches (a wheel at the bottom of its rim);
  - springs, at their ends;
  - a motor's torque on its gear, and its reaction on the housing;
  - slider end stops;
  - propeller air;
  - your pull.

  Several loads of one kind on a part, such as two contact points, are added up in the list. The list updates when a load comes or goes, for example when a wheel lands.
- **Arrows:** joint forces are drawn as purple arrows from where they act; the loads use their own colours: weight green, drag blue, friction red, contact brown, springs grey, motor blue, your pull red. All are sized to the largest, with their size in newtons; couples are curled arrows.
- **Graphs:** every force is a quantity like any other: graph it over time, or put it in a phase plot.

*How it works.* The simulation's own Lagrange multipliers exist only for the constraints that close loops, because most joints are built into the coordinates. So the forces are found afterwards from q, q̇ and q̈, which are recorded every frame along with the simulator's point loads:
- **Every body** obeys m a_G = ΣF and I_G α = ΣM_G; a two-bladed propeller also includes its changing yaw inertia.
- **Every joint** (a massless pin or a point mass) obeys ΣF = m a.
- **Known loads:** gravity, air drag, springs, motor torques (and their reaction on the housing), pin and rail friction, ground contact, slider stops, propeller air and the mouse pull.
- **Unknowns:**

  | Connection | Unknowns | Where and which way |
  |---|---|---|
  | Pin | 2 | at the joint |
  | Frictionless rail | 1 | square to the rail |
  | Gear mesh | 1 | along the tooth tangent, at the pitch point |
  | Rack mesh | 1–2 | along the rack, and square to it when the rack holds the gear's centre distance |
  | Weld | 1–3 | a couple, plus a force where the bodies share no joint |
  | Fixed joint | 2 | at the joint |
  | Locked gear | 1 | a couple |

- **Solving:** all of it is solved together by least squares.

*Checks and limits.*
- **Residual:** the panel shows the model residual, which is about 1e-11 for a pendulum and essentially zero for every example. It is not zero when gear-mesh friction is on, since that is left out.
- **Over-constrained mechanisms,** such as a parallelogram braced by a third parallel bar, have redundant supports. A rigid model can't say how they share the load, so the even (minimum-norm) split is shown and flagged.
- **Gear teeth** carry only their tangential force, since the meshes have no pressure angle.
- **Impacts:** joint forces spike for a few milliseconds during them.

**Internal forces along a bar.** A measured bar (or rack) also lists its internal forces under *Internal forces along it*, each row showing the largest value along the bar with its sign:
- **N**, the axial force, positive in tension;
- **V**, the shear force;
- **M**, the bending moment.

Click one to see its diagram along the bar in the graph panel, from end A (`l.a`) to end B, which replaces whatever the panel showed.
- **The diagram** is filled to the zero line. Point loads make steps, and their places are dashed.
- **Hovering** shows the distance s and the value there, and marks that point on the bar.
- **The bar is coloured** by the size of the diagram: green for none, through yellow, to red at the largest value seen since the diagram opened (or at the value axis's reach, if you set the axis yourself).
- **Axes:** the value axis zooms and slides as in other graphs, and double-clicking starts its range, and the colour scale, again from now. The length axis is fixed.
- **No phase plots:** these diagrams don't join phase plots. Shift-clicking another quantity while one is shown just graphs that quantity instead.

**Beam forces as colour.** The side panel's *Beam forces as colour* switch colours every bar by one internal force while the mechanism runs: **Off**, **Shear** (V), **Tension** (N, the colour shows its size, so compression counts too) or **Bending** (M). A second switch picks how:
- **Maximum:** each bar is one colour for its largest value.
- **Distribution:** each bar is coloured along its length, so it shows where in each bar the load is.

Both modes use one scale for all bars, so bars compare with each other, and a third switch sets what is red (the legend shows the value):
- **Now** (the default): the largest value across all bars at this moment.
- **Run peak:** the largest any bar has reached in this run. It starts again when you choose another force or reset. A jolt at start-up can leave everything green afterwards.
- **Set:** a value you type or drag in the number box. It is kept separately for forces (N) and bending (N·m), and the first time it starts from what is red at that moment.

The bar whose diagram is open in the graph panel keeps that diagram's colouring. The choice is remembered between visits.

**Example: Tower Bridge.** The *Tower Bridge: bascules opening* example (Famous mechanisms) is a 1:25 model of London's 1894 bascule bridge, made to show this analysis. Loading it turns on Bending colouring (Distribution, Run peak) and deflection drawn ×100.
- **The leaves:** each leaf is a deck welded to a tail with a counterweight below the road, turning on a trunnion at its pier. A motor whose PID controller holds the leaf's angle raises it to 80°; the levers in the dock set how far.
- **The rest:** the towers and the two high-level walkways stand still, and the side spans' suspension chains hang from the tower tops to the shore.
- **What to watch:** pause and click a leaf to see its diagram.
  - Closed, a leaf is a cantilever: its bending moment at the trunnion is m g L/2, plus the push that starts it lifting.
  - As it rises, that moment drains into compression along the leaf: at 80° there is little bending and about 29 N of compression.
  - The walkways carry their own weight as beams pinned at both towers, M = wL²/8, and are drawn sagging.

**Deflection.** Tick *Draw deflection ×* in the side panel to draw every bar bent by its bending moment while the mechanism runs. Racks stay straight.
- **Formula:** Euler–Bernoulli, EI w″ = M, with w measured square to the bar (positive to the left of A → B) and w = 0 at both joints. The bar bends between its joints and stays connected to them. The motion is still that of rigid bars, so the deflection is drawn, not simulated.
- **Integration:** w is integrated twice along the bar from the bending-moment diagram. Between stations M is taken as the cubic that matches its value and slope (dM/ds = V) at each end. That is exact here, because a bar's distributed load is linear along it, so the textbook PL³/48EI and 5wL⁴/384EI come out to round-off.
- **E and I:** the side panel sets default values for E (GPa) and I (cm⁴). A bar's own inspector can override them; leave its fields empty to use the defaults. They only change the drawing, so they can be edited while it runs.
- **Exaggeration:** real deflections are small, so they are drawn the side panel's factor times larger (100 by default).
- **True values:** the largest true deflection (mm) is listed among a measured bar's internal forces, and its diagram along the bar opens like the others.

**Bars that break.** Tick *Bars break at |M| ≥* in the side panel and set a bending moment in N·m. While the mechanism runs, any bar whose bending moment reaches that value snaps:
- **Where:** at the largest |M| in each stretch of the bar where |M| reaches the threshold, so a bar can break in several places at once. A break is kept at least 4% of the bar's length from its ends, leaving a short stub at the joint rather than cutting through it.
- **The pieces** are rigid bars with the bar's mass shared by length. The first piece keeps the bar's number. Each cut makes two free joints, one at the end of each piece. Joints pinned onto the bar and rails on it go with the piece they sit on. A weld goes with the piece at the joint the two bodies share, otherwise with the longest piece. Racks don't break.
- **Carrying on:** the equations are derived again for the broken mechanism, which the equations panel then shows. The state carries over exactly: every joint stays where it was and moves as it was, and every gear, motor housing and propeller stays turned as it was. Each piece simply continues the motion it had as part of the bar, so kinetic energy is unchanged. The equations leave out V's constant terms, which differ between mechanisms, so an offset keeps E and the energy budget continuous.
- **Repeated breaks:** pieces can break again later, so a low threshold can shatter a mechanism.
- **What stays the same:** joint heat stays with its joints. Graphs start again from the break, because the coordinates have changed.
- **Reset** puts the bars back as drawn. Changes made while it ran to springs, motors and the ground are kept. The saved design and the undo history never contain broken bars.
- **Limit:** bars are checked once per animation frame, at the recorded state, so a peak shorter than a frame (about 16 ms) can be missed.

*How it works.* Cut the bar at distance s from A. The piece [0, s] is held in balance, d'Alembert style, by what the rest of the bar does on it across the cut:
- **The piece's loads:** every point force on [0, s] where it acts (joint forces, a slider on it, teeth, welds, contact, friction, the pull), plus its share of the distributed load w(x) = μ (g − c v(x) − a(x)), with μ = m / L.
- **The distributed load** is the bar's own weight, air drag and inertia, spread along it rather than lumped at the centre. Since the bar is rigid, v and a are linear along it.
- **The cut** carries R(s) = −[ΣF + ∫₀ˢ w dx] and the couple M(s) = −[Σ((r_i − P(s)) × F_i + M_i) + ∫₀ˢ (x − s) t × w dx].
- **The components:** N = R·t and V = −R·n, with t running from A to B and n a quarter turn anticlockwise from it. This is the usual beam convention with A on the left: dM/ds = V, and M > 0 means sagging.
- **A load off the bar's line** (a wheel's rim contact, say) adds its moment at the step.
- **Closure check:** past the far end everything must balance. The tests check this for every bar of every example; it closes to round-off.

**Settings…** switches to the part's usual inspector, for values that can change while it runs, such as a motor's torque. **← Measurements** switches back.

**Graphs.** Click a quantity to graph it against time; one quantity is graphed at a time. The energy figures on the status bar work the same way, in the same graph: E = T + V, heat W, motor work W_m, air work W_a, and the energy budget. So does the largest constraint error max|Φ|, in scientific notation, since it is usually round-off around 1e-13. Graphing the budget shows how well the energy is accounted for over the run, and graphing max|Φ| how well the constraints hold. Click the figure again to close its graph. The simulation records its state every frame from the start, so the whole run so far can be looked back over. Angles are unwrapped, so they run on past ±180°.
- **Time window:** the graph shows the most recent 10 seconds and scrolls along as it runs, rather than squeezing the whole run in. Early in a run the window starts at the beginning, with the rest still to fill.
- **Scrolling back:** drag the plot or the time axis sideways to move through time. Over the plot, the mouse wheel or a sideways trackpad swipe also scrolls. Scrolling back stops the graph following, and it says "scrolled back" at the top; scroll to the end again, or double-click, and it follows the current time once more.
- **Scales:** hover the time axis (its labels) and turn the wheel to zoom the time scale, from 0.05 s to 10 minutes, about the time under the mouse. Hover the value axis and turn the wheel to zoom the value scale about the value under the mouse, or drag it to slide the range. The value range fits what is visible until you set it.
- **Resetting:** double-click the time axis to come back to the current time, keeping the zoom, or the value axis to fit the values again. Double-clicking the plot does both.
- **Loading as you look:** a graph works out its values only for the time in view, plus half a window either side ready for scrolling. Each joint force or load costs a force solve per frame, so a long run is never worked through all at once.
  - **Coarse to fine:** the curve shows at once and sharpens over the next few frames, using at most about four values per pixel. It says "working out…" until it's done.
  - **Time budget:** the work takes at most 6 ms of each frame, so the app stays smooth.
  - **Kept:** values once worked out are kept, so scrolling back over them is instant.
  - **Phase plots:** these only work out their path, as far back as a joint's trail.

**Phase plots.** With a graph open, **Shift-click** another quantity, on the same part, another part or the status bar.
- **Two quantities:** the graph becomes a phase plot, the first quantity across (x) and the second up (y).
- **Three quantities:** a 3D phase plot, the third along z. Drag it to turn it, and double-click to turn it back. Each axis is scaled to the path shown and labelled with its quantity and largest value.
- **The path:** it shows the path the quantities have traced over the last 20 seconds, the same as a selected joint's trail, fading with age, and a dot at the state now. Hover the path for the time and the values at that point.
- **Changing it:** shift-click a quantity already in the plot to take it out (the first out makes the next one first), and a fourth replaces the third. A plain click goes back to a single quantity against time.
- **Cursor line:** move the mouse over the graph for a vertical line at that time, a dot where it crosses the curve, and the time and value there.
- **⤢ Enlarge:** moves the graph to the middle of the screen, much bigger; **⤡** shrinks it back, and **×** closes it.
- **History length:** about the last 5 minutes are kept (20,000 frames). Reset clears the record.

### Equations panel

The top-right panel updates as you edit. It shows the generalized coordinates and degrees of freedom, one equation of motion per coordinate, and the constraints. Collapsible sections list the kinetic and potential energy and every parameter with its value. For a big mechanism, deriving and typesetting can take a moment. The panel then fills with green to show progress, and the new equations appear when it is done. An equation too long to typeset quickly (over about 4 kB of LaTeX, as in the 30-link ring chain) shows a **Show** button instead.

### Friction and heating

Tick **Friction & heating** to add viscous friction in pin joints, rails and gear teeth. It is modelled with a Rayleigh dissipation function. The energy friction removes heats each joint, which glows from red to white and cools by Newton's law of cooling. The status bar shows the heat produced, W, and E + W, which should stay constant when air drag is off.

### Motors

A motor turns its gear with a torque τ, given in N·m; a positive torque turns it anticlockwise. With a no-load speed Ω, the torque falls in a straight line from τ at rest to zero at Ω, as it does for a DC motor. Above Ω the motor brakes. A no-load speed of 0 gives a constant torque whatever the speed.

**The housing.** A motor is a body of its own: a housing, drawn as a box on the gear's axle, that turns about the axle. The motor turns the gear by τ and its own housing back by −τ, as a real motor does. What happens to that reaction depends on how the housing is held:
- **Mounted on the ground:** this is the default when the axle is pinned to the ground. The housing can't turn, and the ground takes the reaction.
- **Joined to other bodies:** with **Join bodies**, click the part of the housing outside the gear, or use the gear inspector's **Join to** under the motor. The housing then pushes back on what it's joined to, such as a frame or an arm.
- **Free:** neither mounted nor joined. The housing spins back the other way.

In the equations, the torque acts on the gear's turn inside its housing, ρ = φ − ψ, as the generalized force Q = (τ − b ρ̇) ∂ρ/∂q with b = |τ|/Ω (so the torque falls to zero at the no-load speed turning the way it pushes, clockwise for a negative torque). This puts +τ on the gear and −τ on whatever the housing is part of, with nothing pushing from nowhere. The housing has a mass, 0.5 kg by default and set in the inspector, and turns as a disk of 0.6 times the gear's radius.

*Very stiff motors.* A slow motor driving almost nothing has a damping b far stiffer than the 1 ms step can follow. With b/m beyond 2/h, where m is the inertia it drives (from the constrained mass matrix), RK4 would blow up. An example is a bascule's motor once its leaf has broken off. Such a motor reaches its no-load speed within a step anyway, so its speed–torque line is made as steep as the step allows, rechecked every 50 steps as the inertia changes:
- **Without a PID controller,** τ and b are scaled together, so the no-load speed stays exact.
- **Under a PID controller,** which sets τ itself, only b is scaled.

The joint forces use the torque as applied.

The equations panel shows the torque. To drive a bar, such as a crank, join a gear to it with **Join bodies**. You can change the torque and the speed while the simulation runs. The status bar adds the motor's work, W_m, to the energy budget, so E + W − W_m stays constant.

**Levers.** The Lever tool (E) gives a motor a control lever, so you can drive it by hand. The lever is drawn on the motor's housing with a coloured knob. A control in the same colour appears in the **lever dock** at the bottom right of the screen: a vertical slot with a handle.
- **Working it:** drag the handle up for forward (anticlockwise), down for reverse, or leave it in the middle for none. Double-click the slot to centre it. It works while the simulation runs, and the lever on the housing tilts to match.
- **Torque:** τ = position × scale, where the position runs from −1 to +1 and the **scale** is the torque at full throw. Select the lever, by clicking it on the housing or the label above its control, to set the scale and colour, centre it or remove it. While a motor has a lever, the lever sets its torque in place of the gear inspector's torque.
- **Speed:** with a no-load speed Ω, the falloff stays b = scale/Ω as the lever moves, as in a DC motor whose voltage the lever sets. So the lever sets the top speed too, position × Ω, and in the middle the motor brakes against being turned.
- **Many levers:** their controls line up side by side. When there are more than fit, the dock scrolls sideways (with a trackpad, the scroll bar, or the mouse wheel), and the edges with controls scrolled out of view fade out.
- **Under the hood:** moving a lever while the simulation runs only changes the torque parameter, so nothing is derived again and the motion carries on smoothly.

**PID controllers.** The PID tool (C) puts a controller on a motor, drawn as a small box marked PID on the motor. A motor has either a lever or a PID controller: adding one replaces the other.
- **The sensor:** after placing the controller, click what it should measure: a joint, bar, rack, gear, wheel or propeller. A menu then lists what can be measured there:
  - joints: x, y, ẋ, ẏ;
  - bars and racks: the centre's x and y, the angle θ, and their rates;
  - gears and wheels: the centre's x and y, the turn φ, and their rates;
  - propellers: the same, plus the spin rate in rpm.

  The sensor is drawn on that part with a flashing light in the controller's colour, wired to the controller. **Choose…** or **Change…** in the controller's inspector picks again.
- **The target:** it starts at the quantity's value as drawn, so the controller first holds things where they are. Set the **target offset** in the controller's inspector. To move the target by hand, give the controller a lever: click its PID box with the Lever tool, or press **Add a lever for the target**. Its control then joins the dock, labelled "PID", and moves the target by its position × scale (in the quantity's units) about the offset. A lever clicked on the motor itself replaces the controller instead.
- **The control law:** u = s (K_p e + K_i ∫e dt − K_d dy/dt), with e = target − y.
  - The output u is the motor's torque, kept within ±the motor's **max torque**.
  - The derivative acts on the measurement (exactly, from the jacobians, for positions and angles; smoothed for rates), so moving the target gives no kick.
  - The integral pauses while the output is at its limit, so it doesn't wind up.
- **Pushing direction:** s = ±1, chosen in the controller's inspector, and changeable while it runs.
  - **Auto (default):** s is the way the motor actually moves the quantity, found when the simulation starts from rowᵀ A g, with A the constrained inverse inertia, row = ∂y/∂q and g the motor's generalized-force direction. So the controller can't push the wrong way. If the motor can't move the quantity there, the controller says so and switches off. That happens when nothing connects the motor to the quantity, or when the machine starts at a dead point for it, such as a crank drawn pointing right with its tip's x measured.
  - **Forward:** s = +1, so a positive torque raises the quantity.
  - **Reverse:** s = −1, so a positive torque lowers it.

  Forward and Reverse override the check, so they drive the controller from a dead point anyway. The menu shows what the check found. Changing the direction clears the integral, because what it had built up pushed the other way.
- **Units:** quantities are in m, m/s, °, °/s and rpm, and the gains are in N·m per unit. The starting gains suit the quantity and scale with the motor's torque. Gains, target and limit can all be changed while it runs; the inspector shows the measurement, target, error and torque live. The controller runs every 1 ms step.

Examples:
- **Robot-arm joint held by a PID controller:** holds its upper arm at the target set by its lever, against gravity and a swinging forearm.
- **Motor car with cruise control:** holds the chassis's speed over the hills; its motor is found to drive with a negative torque.

Example: **Strandbeest leg driven by a crank motor**. Select the foot, joint G, before pressing Play to draw its walking path.

Another example: **Strandbeest with four legs, gear-coupled and motor-driven**. Four Jansen legs stand one after another, each crank a quarter turn from the next, so the legs step in turn. Each crank carries a gear, and coupling rods join pins at the same place on neighbouring gears. Each pair of gears then forms a parallelogram, so all four cranks turn together, like a locomotive's side rods. A motor on the last gear drives them all. The frame is held still, so the legs step in place.

### Propellers

A propeller is a 3D rigid body kept to the plane. Its spin axis lies in the plane, so the rotor disk is seen edge-on, and the blades are drawn as their projection onto the page: each blade shows foreshortened as it turns out of the page, and blades behind the page are paler.
- **Drawing:** with the Propeller tool (P), click the centre, then the edge of a blade. The line from the centre to the edge is the blade line, its length the radius. The spin axis is square to it, and an arrow shows which way it pushes.
- **Coordinates:** a free propeller has four, x and y of its centre, the hub's yaw θ in the plane, and the blades' spin φ about the axis. They are Euler angles: turn by θ about the axis out of the page, then by φ about the spin axis. Its centre is a joint, so bars can be pinned to it; join it to a body (Join bodies, J) to carry it, which leaves only φ.
- **Kinetic energy:** in body axes (spin axis, blade 1, third axis) its angular velocity is ω = (φ̇, θ̇ sin φ, θ̇ cos φ). The 3D rigid-body energy ½ m v² + ½ ωᵀ I ω, without the out-of-page coordinate and the tilt out of the plane, is T = ½ m (ẋ² + ẏ²) + ½ (I₁ φ̇² + I₂ θ̇² sin²φ + I₃ θ̇² cos²φ). The blades are rods with a chord of 0.16 R, holding 80% of the mass, and the hub is a small disk. With three or more blades, I₂ = I₃. With two, the yaw inertia I₂ sin²φ + I₃ cos²φ swings widely as the blades turn: a free two-bladed rotor yaws in bursts whenever its blades stand out of the page.
- **Motor:** by default a propeller has none and is a free body. A motor (O) spins its blades, and its housing is the hub: mounted on the ground, free, or joined to other bodies, as with a gear's housing. Its default is fast, 0.5 N·m and 3000 rpm no-load. A lever (E) works on it too.
- **No collisions:** a propeller touches nothing.
- **Braked:** ticking it in the inspector stops the blades spinning.

**The air.** With n the spin in revolutions per second, D the diameter and ρ the air density, the standard propeller model [(NASA)](https://ntrs.nasa.gov/api/citations/20205010582/downloads/Simmons_Propeller_SID_v2.pdf) gives thrust T = C_T ρ n² D⁴ and torque Q = C_Q ρ n² D⁵. The coefficients depend mainly on the advance ratio J = V/(nD). Here:
- **Thrust along the axis:** T = ±ρD⁴C_T n|n| − ρD³(C_T/J₀)|n| V_axial. The coefficient falls linearly with J and reaches zero at J₀, so a propeller moving along its axis pushes less. It reverses with the spin; tick **Pitch reversed** for a propeller of the other hand.
- **Air torque against the spin:** Q = −ρD⁵C_Q n|n|.
- **Rotor drag** across the axis: H = −ρD³C_H |n| v. This is the H-force, linear in both speed and spin rate, as found for quadrotors by [Martin and Salaün](https://arxiv.org/pdf/1712.02402v3.pdf).

The forces act at the centre as Q = J_cᵀ(T n + H e), and the torque acts on φ. They are worked out numerically each step, like ground contact. Their work W_a enters the energy budget, so E + W − W_m − W_a stays constant. The defaults are C_T = 0.1, C_Q = 0.007 (C_P = 2πC_Q ≈ 0.044), J₀ = 0.7, C_H = 0.05 and ρ = 1.225 kg/m³. They are typical of small fixed-pitch propellers, and all of them can be set in the inspector.

Example: **Twin-rotor drone with a lever per motor**. A frame carries a motor and propeller at each end, joined into one rigid body: five coordinates (x, y, θ of the frame and the two spins). Its levers are set a little above hover, so it lifts off and climbs. Pull one lever back and it tilts and flies sideways.

### Ground contact

Tick **Ground contact** to add a floor. Joints, bar ends, and gear and wheel rims land on it, bounce, slide and roll.

The ground is a terrain line through waypoints, level beyond the first and the last. A new ground starts flat, just below the machine, with waypoints across it and a metre or more beyond.
- **Select it** by clicking it in Select mode. Its waypoints show as handles.
- **Drag a waypoint** up or down to shape hills and dips, or set its x and height in the inspector.
- **Double-click** the ground to add a waypoint. **Delete** removes the selected one; a ground keeps at least two.

The ground always stays under the machine while you edit. A waypoint dragged up stops where the ground would touch a joint or gear rim. Moving a part below the ground, by dragging a joint for example, lowers the waypoints around it until the part is above the ground again.

**Contact on a terrain.** Each joint is checked against the terrain segment beneath it, using that segment's normal and direction. A gear or wheel rim is checked against the nearest points of the terrain: each segment its centre is above, and each crest waypoint it is rolling over. So a wheel pivots smoothly over a peak, and in a dip it can rest on both sides at once. A rim's friction acts on its slip, the speed of the tyre against the ground, so a free wheel rolls without slipping and a braked one skids. Friction tracks distance along the terrain line, so its hold doesn't jump at a corner. A bar can't cross a straight stretch before one of its ends does, but it can rest across a peak with both ends in the air. So each peak (a waypoint where the terrain turns downhill) is also checked against the bars, and pushes a bar up where it pokes above it.

- **Restitution** sets how bouncy impacts are: 1 bounces back to the full height, 0 stops dead. A dropped mass rebounds to e² of its drop height.
- **Friction μ** holds a point where it landed (static friction) until the sideways force reaches μ times the normal force, then lets it slide. A block on a slope stays put when μ > tan α and otherwise slides at g(sin α − μ cos α).

Contact is a stiff spring and damper at each touching point, applied as a generalized force Q = Jᵀ F. The spring's stiffness is scaled by the effective mass the contact pushes against, worked out from the mass matrix and the constraints. The same idea sets contact softness in the MuJoCo physics engine. So every contact responds at the same rate (250 rad/s) whatever it carries. A 5 g fingertip and a 70 kg leg are equally stable, and anything resting on the ground sinks only about 0.16 mm.

The damper is calibrated so the restitution you set is the one you get. It never pulls the point down, so an uncalibrated damper would bounce higher than the textbook formula predicts. Near the ground each 1 ms step is split into four (or eight for very dead impacts), because contact forces switch on and off within a few milliseconds. Energy lost in impacts and sliding is counted in the heat W, so E + W stays constant. The equations panel describes the contact model in its own section. These forces are computed each step and are not part of the symbolic equations.

Tick **Follow the mechanism** to have the view pan along with something that travels, like the rimless wheel or the motor car.

Examples:
- **Motor car on wheels over hills:** two wheels on a chassis, the rear one driven by a motor whose housing is joined to the chassis. It climbs and rolls over the hills, nosing up as it pulls away because the chassis takes the motor's reaction, then cruises at the motor's no-load speed.
- **Rimless wheel walking down a slope:** McGeer's (1990) simplest passive walker. It walks with no motor, at a steady pace set by the slope.
- **Box dropped onto the floor**.
- **Chain dropped onto the floor**.

### Sound

Tick **Sound effects** in the settings (on by default) and set the **Volume**; both are remembered. Every sound is synthesised in the browser with the Web Audio API, so nothing is downloaded. Audio starts with your first click or key press, as browsers require.

**While editing**, actions make short sounds:
- **Bars:** a wooden knock.
- **Racks:** a ratchet.
- **Springs:** a boing.
- **Gears:** a metallic ting.
- **Wheels:** a rubbery bump.
- **Propellers:** a whoosh.
- **Teeth snapping into mesh:** a clink.
- **Pins and rails:** a snap; freeing a joint plays it in reverse.
- **Joining bodies:** a clunk; separating them plays it in reverse.
- **Fixing an angle:** two ticks.
- **Locking a gear:** a latch.
- **Hanging a mass:** a thunk.
- **Deleting:** a pop.
- **Undo:** a falling blip.
- **Copy and paste:** blips.
- **Loading an example:** a chime.
- **Adding or removing a motor:** it whirs up or down.
- **Lever:** clicks like a ratchet every 5% of throw as you move it.
- **Play, pause and reset:** each has its own chime.
- **Solver failure:** a buzz.

**While running**, the machine is heard as it moves:
- **Motors** whine with their speed, and strain louder and brighter as their torque rises.
- **Propellers** whoosh with their spin, chopping the air at the blade-pass rate (blades × rev/s) and humming at it once it is fast.
- **Gear and rack teeth** tick at their tooth-passing rate, the pitch speed divided by the tooth pitch. Slowly turning gears click tooth by tooth; fast ones whir.
- **Landings** thud louder for faster impacts, and lower and fuller for heavier ones, using the effective mass the contact stopped. A **slider hitting an end stop** clacks.
- **Wheels** rumble as they roll and squeal when they skid. **Joints sliding along the ground** scrape.
- **Fast-moving parts** swish through the air, and **hot friction** sizzles when friction and heating is on.

The simulator reports what these sounds need: impacts with their speed and effective mass, how fast rims roll and slip, how fast joints scrape, motor speeds and torques, and mesh pitch speeds.

### Limitations

- Everything is in a plane, and bars are uniform rigid rods.
- Gears are external spur gears and straight racks. There are no ring gears.
- Parts only collide with the ground, not with each other. The ground is a terrain line through waypoints, and very steep bars, over 60°, only touch it at their ends. Contact is slightly soft: resting parts sink about 0.16 mm, and fast impacts a few millimetres.
- Springs have no mass, and friction is viscous (proportional to speed).

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
