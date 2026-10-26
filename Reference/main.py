"""
Simplified prescribed-speed winch lifting simulation.

Model:
- Fixed winch/fairlead at (0, 0, 20) m.
- Rigid 10 m x 10 m square beam, initially at z=-10 m.
- Four straight slings from each corner to a common joint initially at z=-5 m.
- Main rope connects the fixed winch to the common joint.
- Prescribed upward payout/reel-in speed = 0.1 m/s.
- Simulation duration = 150 s.
- Waterline at z=0 m.

IMPORTANT:
This is a kinematic visualization, NOT a transient structural dynamics solver.
It does not solve rope elasticity, inertia, damping, hydrodynamic drag, or winch torque.
The supplied E and A are used to report EA only.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SIDE = 10.0
Z_BEAM_INITIAL = -10.0
Z_JOINT_INITIAL = -5.0
Z_WINCH = 20.0
WINCH_SPEED = 0.1
DURATION = 150.0
DT = 1.0
WATERLINE = 0.0
ROPE_E = 6.0e10
ROPE_A = 7.85e-5
BEAM_MASS_PER_M_WATER = 82.721721
BEAM_MASS_PER_M_AIR = 283.98

EA = ROPE_E * ROPE_A
CORNERS = np.array([
    [-SIDE/2, -SIDE/2],
    [ SIDE/2, -SIDE/2],
    [ SIDE/2,  SIDE/2],
    [-SIDE/2,  SIDE/2]
])
EDGES = [(0, 1), (1, 2), (2, 3), (3, 0)]

def state_at_time(t):
    dz = WINCH_SPEED * np.clip(t, 0.0, DURATION)
    return Z_BEAM_INITIAL + dz, Z_JOINT_INITIAL + dz

def main():
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set(xlim=(-8, 8), ylim=(-8, 8), zlim=(-12, 22),
           xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    ax.view_init(elev=17, azim=-60)
    ax.set_title("Prescribed-speed winch lifting simulation")

    xx, yy = np.meshgrid(np.linspace(-8, 8, 2), np.linspace(-8, 8, 2))
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.12, color="cyan")
    ax.scatter([0], [0], [Z_WINCH], color="green", s=100, label="Fixed winch")

    beam_lines = [ax.plot([], [], [], lw=4, color="yellowgreen")[0] for _ in EDGES]
    sling_lines = [ax.plot([], [], [], lw=2, color="mediumpurple")[0] for _ in range(4)]
    main_rope = ax.plot([], [], [], lw=2, color="mediumpurple")[0]
    joint = ax.scatter([0], [0], [Z_JOINT_INITIAL], color="black", s=20)
    time_label = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
    info_label = ax.text2D(0.02, 0.86, "", transform=ax.transAxes)
    ax.legend(loc="upper right")

    def update(frame):
        t = frame * DT
        z_beam, z_joint = state_at_time(t)
        main_rope.set_data_3d([0, 0], [0, 0], [Z_WINCH, z_joint])

        for i, (x, y) in enumerate(CORNERS):
            sling_lines[i].set_data_3d([0, x], [0, y], [z_joint, z_beam])

        for i, (a, b) in enumerate(EDGES):
            beam_lines[i].set_data_3d(
                [CORNERS[a, 0], CORNERS[b, 0]],
                [CORNERS[a, 1], CORNERS[b, 1]],
                [z_beam, z_beam]
            )

        joint._offsets3d = ([0], [0], [z_joint])
        time_label.set_text(f"t = {t:.0f} s | beam elevation z = {z_beam:.1f} m")
        info_label.set_text(
            f"Winch speed: {WINCH_SPEED:.2f} m/s upward\n"
            f"Main rope length: {Z_WINCH-z_joint:.1f} m\n"
            f"Rope EA: {EA:,.0f} N"
        )
        return beam_lines + sling_lines + [main_rope, joint, time_label, info_label]

    anim = FuncAnimation(fig, update, frames=int(DURATION/DT)+1, interval=50, blit=False)
    plt.show()

if __name__ == "__main__":
    main()
