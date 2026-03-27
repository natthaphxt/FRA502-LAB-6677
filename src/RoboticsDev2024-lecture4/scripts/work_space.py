#!/usr/bin/env python3

"""
LAB 4 Part 1: HEMISPHERE Workspace Analysis - CUT ALONG X-Y PLANE
This creates a proper DOME by cutting horizontally at the base (z = base_height)
UPPER hemisphere: z >= base_height (above the base)
LOWER hemisphere: z <= base_height (below the base)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3


def create_robot():
    """Create the 3R robot model with DH parameters"""
    robot = rtb.DHRobot(
        [
            rtb.RevoluteMDH(alpha=0.0, a=0.0, d=0.2, offset=0.0),
            rtb.RevoluteMDH(alpha=pi / 2, a=0.0, d=0.02, offset=0.0),
            rtb.RevoluteMDH(alpha=0, a=0.25, d=0.0, offset=0.0),
        ],
        tool=SE3.Tx(0.28),
        name="RRR_Robot",
    )
    return robot


def calculate_workspace_sampling(robot, num_samples=30):
    """Calculate FULL workspace first"""
    q1_samples = np.linspace(-pi, pi, num_samples)
    q2_samples = np.linspace(-pi / 2, pi / 2, num_samples)
    q3_samples = np.linspace(-pi / 2, pi / 2, num_samples)

    x_positions = []
    y_positions = []
    z_positions = []

    print("Sampling full workspace points...")
    total_points = num_samples**3
    point_count = 0

    for q1 in q1_samples:
        for q2 in q2_samples:
            for q3 in q3_samples:
                q = [q1, q2, q3]
                T_0e = robot.fkine(q)
                pos = T_0e.t

                x_positions.append(pos[0])
                y_positions.append(pos[1])
                z_positions.append(pos[2])

                point_count += 1
                if point_count % 1000 == 0:
                    print(f"  Processed {point_count}/{total_points} points...")

    return np.array(x_positions), np.array(y_positions), np.array(z_positions)


def filter_hemisphere_xy_plane(x_pos, y_pos, z_pos, hemisphere_type="upper"):
    """
    Filter points to create HEMISPHERE by cutting along X-Y PLANE (horizontal cut)

    Args:
        hemisphere_type: 'upper' (z >= base_height) or 'lower' (z <= base_height)
    """
    base_height = 0.2  # Base at z = 0.2m

    print(
        f"\nFiltering for {hemisphere_type} hemisphere (CUT AT X-Y PLANE z={base_height})..."
    )

    if hemisphere_type == "upper":
        # Upper hemisphere: keep points ABOVE base plane
        mask = z_pos >= base_height
        print(f"  Filter: z >= {base_height} (UPPER hemisphere - above base)")
    elif hemisphere_type == "lower":
        # Lower hemisphere: keep points BELOW base plane
        mask = z_pos <= base_height
        print(f"  Filter: z <= {base_height} (LOWER hemisphere - below base)")
    else:
        # Default: upper
        mask = z_pos >= base_height
        print(f"  Filter: z >= {base_height} (default UPPER)")

    # Apply filter
    x_filtered = x_pos[mask]
    y_filtered = y_pos[mask]
    z_filtered = z_pos[mask]

    print(f"  Original points: {len(x_pos)}")
    print(
        f"  Filtered points: {len(x_filtered)} ({100*len(x_filtered)/len(x_pos):.1f}%)"
    )

    return x_filtered, y_filtered, z_filtered


def analyze_workspace(x_positions, y_positions, z_positions):
    """Analyze workspace characteristics"""
    base_height = 0.2
    radii_from_base = np.sqrt(
        x_positions**2 + y_positions**2 + (z_positions - base_height) ** 2
    )
    radii_horizontal = np.sqrt(x_positions**2 + y_positions**2)

    max_radius = np.max(radii_from_base)
    min_radius = np.min(radii_from_base[radii_from_base > 0.01])

    z_max = np.max(z_positions)
    z_min = np.min(z_positions)
    horizontal_max = np.max(radii_horizontal)

    x_max = np.max(x_positions)
    x_min = np.min(x_positions)
    y_max = np.max(y_positions)
    y_min = np.min(y_positions)

    print("\n" + "=" * 60)
    print("HEMISPHERE WORKSPACE ANALYSIS (CUT AT X-Y PLANE)")
    print("=" * 60)
    print(f"Maximum reach from base: {max_radius:.3f} m")
    print(f"Minimum reach from base: {min_radius:.3f} m")
    print(f"Maximum horizontal reach: {horizontal_max:.3f} m")
    print(f"X-axis range: {x_min:.3f} to {x_max:.3f} m")
    print(f"Y-axis range: {y_min:.3f} to {y_max:.3f} m")
    print(f"Z-axis range: {z_min:.3f} to {z_max:.3f} m")
    print(f"Cutting plane: z = 0.200 m (base height)")
    print("=" * 60)

    return max_radius, min_radius, radii_from_base


def plot_workspace(
    x_positions, y_positions, z_positions, radii, hemisphere_type="upper"
):
    """
    Create visualization showing HEMISPHERE cut at X-Y plane
    """
    base_height = 0.2

    fig = plt.figure(figsize=(18, 12))

    # 3D scatter plot - Shows hemisphere dome
    ax1 = fig.add_subplot(2, 3, 1, projection="3d")
    scatter = ax1.scatter(
        x_positions, y_positions, z_positions, c=radii, cmap="viridis", s=2, alpha=0.6
    )
    ax1.set_xlabel("X (m)", fontsize=12)
    ax1.set_ylabel("Y (m)", fontsize=12)
    ax1.set_zlabel("Z (m)", fontsize=12)

    title = f"3D Workspace - {hemisphere_type.upper()} Hemisphere\n"
    title += f"(Cut at X-Y plane, z = {base_height}m)"
    ax1.set_title(title, fontsize=14, fontweight="bold")

    # Add base reference
    base_marker = ax1.plot([0], [0], [base_height], "ro", markersize=12, zorder=100)

    # Add X-Y cutting plane visualization (no legend - avoids matplotlib bug)
    xx, yy = np.meshgrid(np.linspace(-0.6, 0.6, 10), np.linspace(-0.6, 0.6, 10))
    zz = np.ones_like(xx) * base_height
    ax1.plot_surface(xx, yy, zz, alpha=0.2, color="red")

    # Simple legend without surface
    ax1.text2D(
        0.05,
        0.95,
        f"● Base (z={base_height}m)\n▬ X-Y cutting plane",
        transform=ax1.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )
    plt.colorbar(scatter, ax=ax1, label="Radius (m)", pad=0.1)

    # Set equal aspect ratio
    max_range = 0.6
    ax1.set_xlim([-max_range, max_range])
    ax1.set_ylim([-max_range, max_range])
    if hemisphere_type == "upper":
        ax1.set_zlim([base_height - 0.05, base_height + max_range])
    else:
        ax1.set_zlim([base_height - max_range, base_height + 0.05])

    # XY projection (top view) - Shows FULL CIRCLE (not cut here)
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.scatter(
        x_positions, y_positions, c=z_positions, cmap="coolwarm", s=2, alpha=0.6
    )
    ax2.plot(0, 0, "ro", markersize=10, label="Base (z=0.2)", zorder=100)
    ax2.set_xlabel("X (m)", fontsize=12)
    ax2.set_ylabel("Y (m)", fontsize=12)
    ax2.set_title(
        f"Top View (X-Y plane)\nFull circle - Cut is horizontal",
        fontsize=12,
        fontweight="bold",
    )
    ax2.axis("equal")
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlim([-0.6, 0.6])
    ax2.set_ylim([-0.6, 0.6])

    # Draw circles at different radii
    for r in [0.1, 0.2, 0.3, 0.4, 0.5]:
        circle = plt.Circle(
            (0, 0), r, fill=False, color="gray", linestyle="--", alpha=0.3
        )
        ax2.add_patch(circle)

    # XZ projection (side view) - Shows HALF CIRCLE (dome profile)
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.scatter(
        x_positions, z_positions, c=y_positions, cmap="coolwarm", s=2, alpha=0.6
    )
    ax3.plot(0, base_height, "ro", markersize=10, label="Base")
    ax3.axhline(
        y=base_height,
        color="r",
        linestyle="-",
        linewidth=2,
        alpha=0.7,
        label=f"X-Y cutting plane (z={base_height})",
    )
    ax3.axhline(y=0, color="g", linestyle="-", linewidth=2, alpha=0.7, label="Ground")
    ax3.set_xlabel("X (m)", fontsize=12)
    ax3.set_ylabel("Z (m)", fontsize=12)
    ax3.set_title(
        f"Side View (X-Z plane)\nShows {hemisphere_type.upper()} dome profile",
        fontsize=12,
        fontweight="bold",
    )
    ax3.axis("equal")
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    ax3.set_xlim([-0.6, 0.6])
    if hemisphere_type == "upper":
        ax3.set_ylim([-0.1, 0.8])
    else:
        ax3.set_ylim([-0.4, 0.4])

    # YZ projection (side view) - Shows HALF CIRCLE (dome profile)
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.scatter(
        y_positions, z_positions, c=x_positions, cmap="coolwarm", s=2, alpha=0.6
    )
    ax4.plot(0, base_height, "ro", markersize=10, label="Base")
    ax4.axhline(
        y=base_height,
        color="r",
        linestyle="-",
        linewidth=2,
        alpha=0.7,
        label=f"X-Y cutting plane (z={base_height})",
    )
    ax4.axhline(y=0, color="g", linestyle="-", linewidth=2, alpha=0.7, label="Ground")
    ax4.set_xlabel("Y (m)", fontsize=12)
    ax4.set_ylabel("Z (m)", fontsize=12)
    ax4.set_title(
        f"Side View (Y-Z plane)\nShows {hemisphere_type.upper()} dome profile",
        fontsize=12,
        fontweight="bold",
    )
    ax4.axis("equal")
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    ax4.set_xlim([-0.6, 0.6])
    if hemisphere_type == "upper":
        ax4.set_ylim([-0.1, 0.8])
    else:
        ax4.set_ylim([-0.4, 0.4])

    # Radius distribution
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.hist(radii, bins=50, edgecolor="black", alpha=0.7)
    ax5.set_xlabel("Radius (m)", fontsize=12)
    ax5.set_ylabel("Count", fontsize=12)
    ax5.set_title("Radius Distribution", fontsize=12)
    ax5.axvline(x=0.53, color="r", linestyle="--", linewidth=2, label="r_max=0.53")
    ax5.axvline(x=0.03, color="g", linestyle="--", linewidth=2, label="r_min=0.03")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # Z-height distribution
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.hist(z_positions, bins=50, edgecolor="black", alpha=0.7)
    ax6.set_xlabel("Z height (m)", fontsize=12)
    ax6.set_ylabel("Count", fontsize=12)
    ax6.set_title("Z-height Distribution", fontsize=12)
    ax6.axvline(
        x=base_height,
        color="r",
        linestyle="-",
        linewidth=2,
        label=f"Cutting plane z={base_height}",
    )
    ax6.axvline(x=0, color="g", linestyle="-", linewidth=2, alpha=0.7, label="Ground")
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    suptitle = f"HEMISPHERE WORKSPACE - {hemisphere_type.upper()}\n"
    suptitle += f"(Cut along X-Y plane at z = {base_height}m)"
    plt.suptitle(suptitle, fontsize=16, fontweight="bold", y=0.995)
    plt.tight_layout()

    filename = f"workspace_hemisphere_XY_cut_{hemisphere_type}.png"
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    print(f"\n✓ Hemisphere visualization saved as '{filename}'")
    plt.show()


def create_dome_wireframe(hemisphere_type="upper"):
    """
    Create a clean wireframe dome cut at X-Y plane
    """
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Parameters
    r_max = 0.53
    base_height = 0.2

    # Create hemisphere surface
    if hemisphere_type == "upper":
        # Upper hemisphere: elevation from 0 (horizontal) to 90° (top)
        u = np.linspace(0, np.pi / 2, 30)
    else:
        # Lower hemisphere: elevation from 90° (horizontal) to 180° (bottom)
        u = np.linspace(np.pi / 2, np.pi, 30)

    v = np.linspace(0, 2 * np.pi, 40)  # Full rotation around z-axis
    u, v = np.meshgrid(u, v)

    # Convert to Cartesian coordinates (sphere centered at base)
    x = r_max * np.sin(u) * np.cos(v)
    y = r_max * np.sin(u) * np.sin(v)
    z = r_max * np.cos(u) + base_height

    # Plot wireframe
    ax.plot_wireframe(x, y, z, color="black", linewidth=0.5, alpha=0.6)

    # Add base circle at cutting plane
    theta_base = np.linspace(0, 2 * np.pi, 100)
    x_base = r_max * np.cos(theta_base)
    y_base = r_max * np.sin(theta_base)
    z_base = np.ones_like(theta_base) * base_height
    ax.plot(x_base, y_base, z_base, "r-", linewidth=2, label="X-Y cutting plane")

    # Add center point
    ax.plot([0], [0], [base_height], "ro", markersize=10, label="Base")

    # Add vertical line
    if hemisphere_type == "upper":
        ax.plot([0, 0], [0, 0], [base_height, base_height + r_max], "k-", linewidth=2)
    else:
        ax.plot([0, 0], [0, 0], [base_height - r_max, base_height], "k-", linewidth=2)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    title = f"Hemisphere Dome - {hemisphere_type.upper()}\n"
    title += f"(Cut at X-Y plane, z = {base_height}m)"
    ax.set_title(title, fontsize=14, fontweight="bold")
    ax.legend()

    # Set equal aspect
    ax.set_xlim([-0.6, 0.6])
    ax.set_ylim([-0.6, 0.6])
    if hemisphere_type == "upper":
        ax.set_zlim([0, 0.8])
    else:
        ax.set_zlim([-0.4, 0.4])

    # Set viewing angle for better visualization
    ax.view_init(elev=20, azim=45)

    filename = f"hemisphere_wireframe_XY_cut_{hemisphere_type}.png"
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    print(f"✓ Wireframe dome saved as '{filename}'")
    plt.show()


def main():
    """Main function"""
    print("\n" + "=" * 60)
    print("LAB 4 - HEMISPHERE WORKSPACE (CUT AT X-Y PLANE)")
    print("Horizontal cut at base height (z = 0.2m)")
    print("=" * 60)

    # Choose hemisphere type: 'upper' or 'lower'
    # UPPER = above base (typical for robot reaching upward)
    # LOWER = below base (if robot can reach downward)
    hemisphere_type = "upper"

    print(f"\nGenerating {hemisphere_type.upper()} hemisphere")
    print(f"Cut at X-Y plane (z = 0.2m)")
    print("=" * 60)

    # Create robot
    robot = create_robot()

    # Sample FULL workspace
    x_full, y_full, z_full = calculate_workspace_sampling(robot, num_samples=20)

    # Filter for HEMISPHERE (cut at X-Y plane)
    x_hemi, y_hemi, z_hemi = filter_hemisphere_xy_plane(
        x_full, y_full, z_full, hemisphere_type
    )

    # Analyze
    base_height = 0.2
    radii = np.sqrt(x_hemi**2 + y_hemi**2 + (z_hemi - base_height) ** 2)
    r_max, r_min, _ = analyze_workspace(x_hemi, y_hemi, z_hemi)

    # Plot hemisphere points
    plot_workspace(x_hemi, y_hemi, z_hemi, radii, hemisphere_type)

    # Create clean wireframe dome
    create_dome_wireframe(hemisphere_type)

    print("\n" + "=" * 60)
    print("✓ HEMISPHERE ANALYSIS COMPLETE (X-Y PLANE CUT)")
    print("=" * 60)
    print(f"Hemisphere type: {hemisphere_type.upper()}")
    print(f"Cutting plane: X-Y plane at z = {base_height}m")
    print(f"Shape: Dome (hemisphere)")
    print(f"r_max = {r_max:.3f} m")
    print(f"r_min = {r_min:.3f} m")
    print("=" * 60)


if __name__ == "__main__":
    main()
