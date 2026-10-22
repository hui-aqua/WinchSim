# WinchSim input file guide

This guide explains how to create a new lifting case in WinchSim’s AquaSim-compatible XML format. It is written for users who want to build a model by editing XML or copying one of the examples in `input/`.

For the run and viewer workflow, see [the user guide](USER_GUIDE.md). The current solver limits and planned validation work are in [the audit](AUDIT.md) and [the TODO list](TODO.md).

## 1. Start from a supplied case

Copy the closest model from `input/` to a new file before editing:

- `model.xml`: direct rope connection to the lifted assembly.
- `model_roller.xml`: rope passes over a fixed roller.
- `model_roller_buoy.xml`: fixed roller, buoy body, and lower slings.

Keep your own component properties, node coordinates, element IDs, and output paths together. Use unique node and element IDs within the model. A supplied case is the safest way to preserve the expected XML spelling and required attributes.

Run the new case with:

```powershell
.\.venv\Scripts\python.exe solver/solve.py input/my_case.xml
```

## 2. File structure

The XML root is `<model>`. A typical file has these sections in order:

1. Model metadata and units.
2. `<environment>` settings.
3. `<Nodes>` with point IDs and coordinates.
4. `<Components>` with beam/truss properties and connected elements.
5. A model-level `<winch>` schedule.
6. `<solver>` duration, output and numerical settings.

WinchSim accepts `.xml` and `.amodel` files and retains a legacy JSON input path. AquaSim XML is broader than the subset WinchSim reads: unrecognized attributes may be ignored. Do not assume that every AquaSim field affects the WinchSim calculation merely because the file opens in AmodelViewer.

## 3. Units and coordinates

The examples use metres, seconds, kilograms, and newtons. Node coordinates are in metres, and `waterline_z_m` is the global Z coordinate of the still-water surface. Positive Z is upward.

The XML header can label units with `units_length`, `units_time`, `units_mass`, and `units_force`; these labels do not convert numeric values. Enter values in the units expected by the fields.

One legacy AquaSim field needs particular care: `<materials netLineLoadInWater="...">` for a buoy is read as the buoy’s signed underwater line-load equivalent and multiplied by gravity by the solver. Follow the magnitude and sign convention in the supplied buoy example and verify it against the buoy data. Do not substitute a force-per-length number in newton units without converting it to the convention expected by this field.

## 4. Metadata and environment

```xml
<model version="2.5"
       name="My lifting case"
       description="Short description of the arrangement"
       model_type="global_quasi_static_equilibrium"
       units_length="m" units_time="s" units_mass="kg" units_force="N">
  <environment waterline_z_m="0" gravity_m_per_s2="9.81" />
  <!-- Nodes, Components, winch, and solver go here. -->
</model>
```

`name`, `description`, and the unit labels help identify the file. The solver reads the numeric waterline and gravity from `<environment>`; it does not convert values based on the header labels.

## 5. Define nodes

Each node needs a unique integer `id` and `x`, `y`, `z` coordinates:

```xml
<Nodes>
  <node id="1" x="-5" y="-5" z="-5" name="beam_corner_1" />
  <node id="5" x="0" y="0" z="-3" name="upper_joint" role="upper_joint" />
  <node id="6" x="0" y="0" z="20" name="fixed_winch" role="winch" />
</Nodes>
```

`name` is for people and viewer labels. `role` changes how selected points are interpreted:

| Node role | Meaning in the global analysis |
| --- | --- |
| `upper_joint` | Moving connection between the main rope and lifted structure. A winch-controlled main-rope leg must connect to this point. |
| `roller` | Fixed roller point for a routed main rope. |
| `winch` | Fixed winch/rope anchor point. |
| `fixed`, `fixed_position`, `anchor` | Fixed-position point. |
| `lower_joint` | Buoy/lower-sling connection point; also used in result fields and viewer mapping. |
| `buoy_top` | Upper point of the buoy body; used for result and viewer mapping. |
| Other or omitted role | Ordinary structural point unless it belongs to a rigid beam component. |

The beam mesh is determined from the connectivity of beam elements, not from the order of nodes or a special `beam_corner` role. Every node ID referenced by an element must exist.

## 6. Define beam components

A beam component groups one or more beam elements that share section and material properties. For a typical rigid lifting structure:

```xml
<beam id="beam1" name="Lifting frame" property_set="beam" structural_model="rigid">
  <wizard type="circular" outerDiameter="500" eModulus="1.1E9" gModulus="3.84E8" />
  <materials young="1.1E9" shear="3.84E8"
             crossectionareal="0.0649" iy="0.00169" iz="0.00169" it="0.00339"
             weightInAir="283.98" effectiveLineMassInWater="82.72" />
  <elements>
    <element id="1" StartNode_ID="1" EndNode_ID="2" name="frame_edge_1" />
    <element id="2" StartNode_ID="2" EndNode_ID="3" name="frame_edge_2" />
  </elements>
</beam>
```

Important fields:

- `id`: component identifier, unique among components.
- `property_set`: property dictionary name; the examples use `beam` for a lifting beam, `rope` for shared rope/slings, and `buoy` for a buoy. Components with the same property-set name share the same material/section values; give components different names if their properties differ.
- `structural_model`: use `rigid` for the supported rigid-body example workflow. `elastic` selects an OpenSees beam model and requires complete section properties; validate this path separately before using it for engineering decisions.
- `rotation_behavior="fixed"`: keeps that rigid component’s orientation fixed while allowing translation. The buoy example uses this to keep its axis vertical.
- `wizard outerDiameter`: circular section diameter in **millimetres** in the AquaSim XML convention. WinchSim uses it to establish the section size and waterline transition width for ordinary beams.
- `young`, `shear`, `crossectionareal`, `iy`, `iz`, `it`: elastic modulus, shear modulus, cross-sectional area, second moments, and torsional constant. Use Pa and SI section units (m², m⁴) where applicable.
- `weightInAir`: beam mass per length in kg/m.
- `effectiveLineMassInWater`: effective submerged mass per length in kg/m; this can account for buoyancy.

Beam elements connect nodes with `StartNode_ID` and `EndNode_ID`. Their connectivity defines the rigid component shape. Build a connected structure, and avoid assigning a node to two separate beam components.

## 7. Define truss and rope components

Truss elements represent the main rope and slings. The current global solver treats them as tension-only axial members; they carry no compression. Example:

```xml
<truss id="rope1" name="Main rope" property_set="truss">
  <mooring weightInAir="0.07615"
           netLineMassInWater="-0.00043125"
           areal="7.85E-5" young="6.0E10"
           noCompressionForces="true" />
  <elements>
    <element id="10" StartNode_ID="6" EndNode_ID="5"
             name="winch_to_upper_joint" role="main_rope"
             subdivisions="8" />
  </elements>
</truss>
```

| Field | Meaning |
| --- | --- |
| `weightInAir` | Rope mass per length in air, kg/m; must be non-negative. |
| `netLineMassInWater` | Signed net submerged mass per length, kg/m. A negative value represents net buoyancy. |
| `areal` | Cross-sectional area in m². |
| `young` | Young’s modulus in Pa. |
| `noCompressionForces` | AquaSim-compatible description of tension-only behavior. WinchSim’s global rope model is tension-only. |
| `subdivisions` | Number of solver segments along this element. More segments describe a curved/slack shape with more points but increase solve cost. |
| `rest_length_m` | Optional total unstretched length of this element in metres. If omitted, the initial endpoint distance is used. |

The initial endpoint distance must not exceed `rest_length_m`. If a line is subdivided, the total rest length is divided evenly among its solver segments. Use enough subdivisions to resolve the shape you care about, then check that increasing the count does not materially change the result.

Components that use the same `property_set` name share one property dictionary in the parsed model. In the XML parser, a later component using the same name replaces that property entry, so do not reuse a name for components that need different values.

Element `role` values used by the examples are:

- `main_rope`: winch-controlled rope. The active leg is the main-rope line connected to `upper_joint`.
- `upper_sling` and `lower_sling`: identify sling families for output, viewer presentation, and model interpretation. They use the same tension-only truss formulation.

## 8. Build the rope route

### Direct winch connection

Use one `main_rope` line from the fixed `winch` node to the `upper_joint` node:

```xml
<element id="10" StartNode_ID="6" EndNode_ID="5" role="main_rope" />
```

### Route over a fixed roller

Use two connected `main_rope` elements: winch to roller, then roller to upper joint. Give the route points the `winch` and `roller` node roles. The roller and winch positions remain fixed. The leg ending at `upper_joint` is the leg whose unstretched length changes under the motion schedule.

The global solver needs a main-rope leg connected to an `upper_joint`. When the schedule commands nonzero winch motion, exactly one such active leg must be identifiable.

Model the roller as a beam component with `role="roller_body"`. Exactly two truss elements must connect to the beam endpoint marked `role="roller"`, and those trusses must have the same component `name` as the roller beam. (The XML keeps beam and truss component IDs separate because they are different element types.) Set `allowed_rope_components` on the beam component to a comma-separated list of truss component IDs permitted to pass through the roller. A rope attached at the roller head but omitted from this list is not an allowed pass-through route. The beam's other endpoint is an ordinary node: it can be fixed by assigning a fixed role or connected to rope or other beam elements. Use `structural_model="elastic"` to calculate beam end loads. For example:

```xml
<beam id="roller1" name="Roller support" property_set="roller_beam"
      structural_model="elastic" allowed_rope_components="rope1">
  <!-- section and material properties -->
  <elements><element id="11" StartNode_ID="8" EndNode_ID="6" role="roller_body" /></elements>
</beam>
```

Elastic beam results include local end axial force, two shear forces, torsion, and two bending moments. Fixed nodes report global reaction forces and moments.

### Mechanical model behind the roller representation

The rope does not terminate at the roller. It is represented by two tension-only truss elements that share the roller head node. Each truss element applies its axial force along its own current direction. The resultant load transferred to the roller head is the vector sum of those two rope forces (plus any other loads explicitly connected at that node). For rope tensions \(T_1\), \(T_2\) and unit vectors \(\mathbf{u}_1\), \(\mathbf{u}_2\) pointing away from the head along the rope legs, the rope load on the head is:

\[
\mathbf{F}_{rope} = -T_1\mathbf{u}_1 - T_2\mathbf{u}_2.
\]

This vector view matters: two equal tensions can still produce a nonzero resultant when the rope changes direction. For a symmetric reversal through angle \(\theta\), the resultant magnitude is \(2T\sin(\theta/2)\). The angle and the two independently calculated tensions determine the load direction and magnitude; no separate point-force approximation is needed.

The `roller_body` beam joins the rope head to the support or connected structure. Its end forces and moments are the internal actions required to transfer the head load through that member. In the beam's local axes, the solver writes axial force \(N\), two shear components \(V_y,V_z\), torsion \(M_x\), and bending moments \(M_y,M_z\) at both ends. Their signs follow the beam's element node order and local-axis convention. The opposite end is an ordinary structural node, so its equilibrium is determined by whatever is attached there: a fixed boundary, a rope, another beam, or a combination. Reactions at fixed nodes are support reactions, while the beam end forces describe the member action at its connections.

The current roller rule is a connectivity rule: one `roller_body` beam has exactly one endpoint with role `roller`, and exactly two truss elements meet at that head. It does not model a finite-diameter rotating sheave, contact pressure, wrap-angle evolution, bearing details, or frictional tension ratio. The two rope legs remain separate tension-only elements, so their tensions may differ because of their material lengths and equilibrium. The elastic beam solver calculates small-deformation beam response using the supplied section and material properties; use appropriate properties and independently check results before design use.

### Connect a lifted structure with slings

Create a truss line from `upper_joint` to each supported lifting point and assign `role="upper_sling"`. For lower slings, connect the beam lifting points to a shared `lower_joint` and use `role="lower_sling"`. These labels help output and scene interpretation; the geometry and connectivity still determine the mechanics.

## 9. Add a buoy

The supplied buoy is represented by a beam component with a single vertical axis element. The lower endpoint is the `lower_joint`; the upper endpoint is `buoy_top`. Give the element `role="buoy_body"` and use `property_set="buoy"` so the viewer can obtain buoy dimensions:

```xml
<beam id="buoy1" name="Buoy" property_set="buoy"
      structural_model="rigid" rotation_behavior="fixed">
  <wizard type="circular" outerDiameter="355" eModulus="2.1E12" gModulus="7.5E11" />
  <materials young="2.1E12" shear="7.5E11" crossectionareal="0.01805"
             iy="0.000258" iz="0.000258" it="0.000517"
             weightInAir="16.9685" effectiveLineMassInWater="82.72"
             netLineLoadInWater="-136.9128" />
  <elements>
    <element id="15" StartNode_ID="6" EndNode_ID="9"
             name="buoy_body_axis" role="buoy_body" />
  </elements>
</beam>
```

This is the current line-based buoy approximation. The buoy axis is vertical, and immersion is calculated along that axis. Check submerged buoyancy and dry weight against the buoy’s actual displacement and mass. A buoy attached to lifting slings can be pulled above the water if sling tension exceeds its buoyant support; the solver does not enforce the waterline as a solid boundary.

For the supplied 50 m retrieval schedule, the lower slings have an explicit 45 m rest length. The maximum distance from the freely floating buoy bottom to a lifting-beam corner is about 40.9 m, so this leaves slack for the buoy to remain at the surface. If `rest_length_m` is omitted, WinchSim uses the initial endpoint spacing (about 6–9 m here); those short slings become taut and hoist the buoy out of the water as the beam rises. Set this value from the real sling length and the full winch travel for other models.

## 10. Set winch motion

Define motion at the model level. `duration_s` belongs to `<solver>`, not `<winch>`:

```xml
<winch mode="prescribed_speed">
  <motion_schedule>
    <point time_s="0" speed_m_per_s="0.1" direction="in" />
    <point time_s="150" speed_m_per_s="0.1" direction="out" />
  </motion_schedule>
</winch>
```

The schedule is piecewise constant:

- The first point must be at `time_s="0"`.
- Times must be strictly increasing and within the simulation duration.
- Speed is a non-negative magnitude in m/s.
- `in` retrieves rope and shortens the active line’s unstretched length.
- `out` pays out rope and lengthens it.
- A point’s command remains active until the next point or the end of the run.

For a simple constant-speed case, legacy attributes are still accepted:

```xml
<winch mode="prescribed_speed"
       payout_or_retrieval_speed_m_per_s="0.1"
       direction="upward" />
```

New cases should use `motion_schedule` with `direction="in"` or `"out"`; `upward`/`downward` are legacy direction names. Avoid old winch-level duration fields: set the global duration in `<solver>`.

## 11. Configure the solver

Example for a new global quasi-static case:

```xml
<solver duration_s="300"
        analysis_method="global_quasi_static_equilibrium"
        time_step_s="1"
        continuation_min_step_s="0.01"
        default_truss_subdivisions="8"
        output_bin="results/my_case.bin"
        output_csv="results/my_case.csv"
        dynamic_effects_enabled="false"
        equilibrium_max_iterations="3000"
        equilibrium_force_acceptance_n="25"
        equilibrium_relative_force_tolerance="0.005"
        aquasim_convergence_criteria="1" />
```

Paths that are not absolute are resolved from the WinchSim project folder, not from the XML file’s folder.

| Setting | Use |
| --- | --- |
| `analysis_method` | For a new connected model, use `global_quasi_static_equilibrium`. Other available methods are `quasi_static_mesh_preview` and the legacy `quasi_static_lower_sling_profiles`. |
| `duration_s` | Total simulation duration, seconds. |
| `time_step_s` | Requested output interval, seconds. Failed intervals may be retried internally at smaller intervals. |
| `continuation_min_step_s` | Smallest allowed retry interval. Smaller values can improve continuation near abrupt geometry/load changes but may increase solve time. |
| `default_truss_subdivisions` | Mesh segments for truss elements that do not set their own `subdivisions`. |
| `output_bin`, `output_csv` | Binary viewer result and CSV result paths. |
| `dynamic_effects_enabled` | Keep `false`; dynamic analysis is not implemented. Setting it to `true` stops with an explicit error. |
| `equilibrium_max_iterations` | Iteration limit for rigid energy minimization. Increasing it can cost more time and does not guarantee a physical solution. |
| `equilibrium_force_acceptance_n` | Absolute floor for accepted force residual. |
| `equilibrium_relative_force_tolerance` | Relative residual threshold based on the model’s total member-load scale. |
| `aquasim_convergence_criteria` | AquaSim-style force/displacement change norm. It supplements the force-residual check and cannot accept a stalled solve by itself. |
| `force_tolerance_n` | Optimizer gradient tolerance in newtons. It is not a material strength limit. |

The XML parser also accepts advanced and legacy solver attributes. Unless they are described in the user guide or a supplied model, leave them unchanged; an accepted XML attribute is not necessarily used by the selected analysis method.

## 12. Complete small example

This direct-lift case shows the essential connectivity. Copy a supplied input for a production case so that you can retain complete material data and solver settings.

```xml
<?xml version="1.0" encoding="utf-8"?>
<model version="2.5" name="Small frame lift"
       description="Direct main-rope lift with four upper slings"
       model_type="global_quasi_static_equilibrium"
       units_length="m" units_time="s" units_mass="kg" units_force="N">
  <environment waterline_z_m="0" gravity_m_per_s2="9.81" />
  <Nodes>
    <node id="1" x="-5" y="-5" z="-5" />
    <node id="2" x="5" y="-5" z="-5" />
    <node id="3" x="5" y="5" z="-5" />
    <node id="4" x="-5" y="5" z="-5" />
    <node id="5" x="0" y="0" z="-3" name="upper_joint" role="upper_joint" />
    <node id="6" x="0" y="0" z="20" name="winch" role="winch" />
  </Nodes>
  <Components>
    <beam id="frame" name="Lifting frame" property_set="beam" structural_model="rigid">
      <wizard type="circular" outerDiameter="500" eModulus="1.1E9" gModulus="3.84E8" />
      <materials young="1.1E9" shear="3.84E8" crossectionareal="0.0649"
                 iy="0.00169" iz="0.00169" it="0.00339"
                 weightInAir="283.98" effectiveLineMassInWater="82.72" />
      <elements>
        <element id="1" StartNode_ID="1" EndNode_ID="2" />
        <element id="2" StartNode_ID="2" EndNode_ID="3" />
        <element id="3" StartNode_ID="3" EndNode_ID="4" />
        <element id="4" StartNode_ID="4" EndNode_ID="1" />
      </elements>
    </beam>
    <truss id="main" name="Main rope" property_set="rope">
      <mooring weightInAir="0.07615" netLineMassInWater="-0.00043125"
               areal="7.85E-5" young="6.0E10" noCompressionForces="true" />
      <elements>
        <element id="10" StartNode_ID="6" EndNode_ID="5" role="main_rope"
                 name="winch_to_upper_joint" subdivisions="8" />
      </elements>
    </truss>
    <truss id="slings" name="Upper slings" property_set="rope">
      <mooring weightInAir="0.07615" netLineMassInWater="-0.00043125"
               areal="7.85E-5" young="6.0E10" noCompressionForces="true" />
      <elements>
        <element id="11" StartNode_ID="5" EndNode_ID="1" role="upper_sling" />
        <element id="12" StartNode_ID="5" EndNode_ID="2" role="upper_sling" />
        <element id="13" StartNode_ID="5" EndNode_ID="3" role="upper_sling" />
        <element id="14" StartNode_ID="5" EndNode_ID="4" role="upper_sling" />
      </elements>
    </truss>
  </Components>
  <winch mode="prescribed_speed">
    <motion_schedule>
      <point time_s="0" speed_m_per_s="0.05" direction="in" />
      <point time_s="10" speed_m_per_s="0.05" direction="out" />
    </motion_schedule>
  </winch>
  <solver duration_s="20" analysis_method="global_quasi_static_equilibrium"
          time_step_s="1" default_truss_subdivisions="8"
          output_bin="results/small_frame.bin" output_csv="results/small_frame.csv"
          dynamic_effects_enabled="false" />
</model>
```

This example uses sample properties for illustration. Replace them with verified properties for the actual structure and rigging. It intentionally omits design strength checks and dynamic effects.

## 13. Pre-run checklist

Before starting a long run, check:

- [ ] Coordinates, dimensions, gravity, and waterline use the expected units and sign convention.
- [ ] Every node and line ID is unique; every element endpoint references an existing node.
- [ ] Beam connectivity matches the actual lifted structure and each beam component is connected.
- [ ] The active rope path reaches the `upper_joint`; roller and winch locations use the corresponding fixed roles.
- [ ] Rope rest lengths are positive and at least as long as their initial endpoint distance.
- [ ] Rope and beam mass, stiffness, area, and buoyancy values match trusted source data.
- [ ] Winch schedule starts at zero, has increasing times within `duration_s`, and uses the intended `in`/`out` command.
- [ ] `output_bin` and `output_csv` point to the intended files.
- [ ] A short case completes and its geometry, reactions, and line tensions are plausible before increasing duration or mesh density.
- [ ] The final result is reviewed for high tension/extension and physical plausibility; numerical completion alone is not a safety assessment.

## 14. Common input problems

| Symptom | Likely cause |
| --- | --- |
| Model says no upper joint or main rope | Missing `role="upper_joint"` node or `role="main_rope"` line. |
| Winch-controlled leg cannot be identified | Main-rope connectivity does not include exactly one active leg connected to `upper_joint`. |
| Unknown component/property set | A line’s `component_id` does not match a `<beam>`/`<truss>` component, or its `property_set` is missing. |
| Initial cable geometry is infeasible | Endpoint distance exceeds `rest_length_m`, or the assigned material length is too short. |
| Schedule validation error | First point is not at zero, times are duplicated/out of order/outside the run, direction is not `in` or `out`, or speed is negative. |
| Unexpected buoy/line movement at the surface | Check submerged and dry line loads, connection geometry, sling tension, and whether the body is free to be lifted. The waterline is a fluid boundary, not a solid support. |
| Run completes but loads look unrealistic | Check input properties, connectivity, rope lengths, and model scope. The solver has no breaking-load rule and no independent design-standard validation. |

