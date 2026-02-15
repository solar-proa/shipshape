"""
Generate structural diagrams for validation scenarios.

Uses Matplotlib to create simple schematic diagrams illustrating
the mechanical models used in each validation test.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, Polygon, Arc, Wedge
import numpy as np
from pathlib import Path


# Style constants
BEAM_COLOR = 'steelblue'
SUPPORT_COLOR = 'lightgray'
FORCE_COLOR = 'red'
REACTION_COLOR = 'green'
MOMENT_COLOR = 'purple'
DIMENSION_COLOR = 'gray'
TITLE_SIZE = 14
LABEL_SIZE = 12
SMALL_LABEL_SIZE = 10
TINY_LABEL_SIZE = 9


def setup_diagram(figsize=(10, 5)):
    """Create a figure with clean styling for technical diagrams."""
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect('equal')
    ax.axis('off')
    return fig, ax


def draw_fixed_support(ax, x, y, width=0.15, height=0.3):
    """Draw a fixed (cantilever) support symbol."""
    rect = patches.Rectangle((x - width, y - height/2), width, height,
                               linewidth=1.5, edgecolor='black',
                               facecolor=SUPPORT_COLOR, hatch='///')
    ax.add_patch(rect)


def draw_pin_support(ax, x, y, size=0.15):
    """Draw a pin support (triangle)."""
    triangle = Polygon([(x, y), (x - size, y - size*1.2), (x + size, y - size*1.2)],
                       closed=True, linewidth=1.5, edgecolor='black', facecolor='white')
    ax.add_patch(triangle)
    ax.plot([x - size*1.5, x + size*1.5], [y - size*1.2, y - size*1.2], 'k-', linewidth=1.5)


def draw_roller_support(ax, x, y, size=0.12):
    """Draw a roller support (triangle on circle)."""
    triangle = Polygon([(x, y), (x - size, y - size), (x + size, y - size)],
                       closed=True, linewidth=1.5, edgecolor='black', facecolor='white')
    ax.add_patch(triangle)
    circle = plt.Circle((x, y - size - size*0.4), size*0.4,
                        linewidth=1.5, edgecolor='black', facecolor='white')
    ax.add_patch(circle)
    ax.plot([x - size*1.5, x + size*1.5], [y - size - size*0.8, y - size - size*0.8], 'k-', linewidth=1.5)


def draw_spring(ax, x1, y1, x2, y2, n_coils=5, width=0.1):
    """Draw a spring between two points."""
    length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    angle = np.arctan2(y2-y1, x2-x1)

    # Generate spring path
    t = np.linspace(0, 1, n_coils * 20 + 1)
    spring_x = t * length
    spring_y = width * np.sin(t * n_coils * 2 * np.pi)

    # Rotate and translate
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    plot_x = x1 + spring_x * cos_a - spring_y * sin_a
    plot_y = y1 + spring_x * sin_a + spring_y * cos_a

    ax.plot(plot_x, plot_y, 'k-', linewidth=1.5)


def draw_beam(ax, x1, y, x2, height=0.1, color=BEAM_COLOR, label=None):
    """Draw a beam (rectangle)."""
    rect = patches.Rectangle((x1, y - height/2), x2 - x1, height,
                               linewidth=1.5, edgecolor='black', facecolor=color)
    ax.add_patch(rect)
    if label:
        ax.text((x1 + x2)/2, y, label, fontsize=LABEL_SIZE, ha='center', va='center',
                color='white', fontweight='bold')


def draw_tube(ax, x, y1, y2, width=0.08, color=BEAM_COLOR, label=None):
    """Draw a vertical tube/pipe."""
    rect = patches.Rectangle((x - width/2, y1), width, y2 - y1,
                               linewidth=1.5, edgecolor='black', facecolor=color)
    ax.add_patch(rect)
    if label:
        ax.text(x, (y1 + y2)/2, label, fontsize=SMALL_LABEL_SIZE, ha='center', va='center',
                color='white', fontweight='bold', rotation=90)


def draw_force_arrow(ax, x, y, dx=0, dy=0, label=None, color=FORCE_COLOR, label_offset=(0.1, 0)):
    """Draw a force arrow."""
    ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
               arrowprops=dict(arrowstyle='->', color=color, lw=2))
    if label:
        label_x = x + dx/2 + label_offset[0]
        label_y = y + dy/2 + label_offset[1]
        ax.text(label_x, label_y, label, fontsize=LABEL_SIZE, color=color, ha='left', va='center')


def draw_distributed_load(ax, x1, x2, y, intensity=0.4, label=None, color=FORCE_COLOR, n_arrows=5):
    """Draw a distributed load (multiple arrows with connecting line).

    Positive intensity = arrows pointing down (from above the beam).
    Negative intensity = arrows pointing up (from below the beam).
    """
    xs = np.linspace(x1, x2, n_arrows)
    ax.plot([x1, x2], [y + intensity, y + intensity], color=color, lw=1.5)
    for x in xs:
        ax.annotate('', xy=(x, y), xytext=(x, y + intensity),
                   arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
    if label:
        label_offset = 0.1 if intensity > 0 else -0.15
        va = 'bottom' if intensity > 0 else 'top'
        ax.text((x1 + x2)/2, y + intensity + label_offset, label, fontsize=LABEL_SIZE,
                color=color, ha='center', va=va)


def draw_dimension(ax, x1, x2, y, label, offset=-0.3):
    """Draw a dimension line with label."""
    ax.annotate('', xy=(x1, y + offset), xytext=(x2, y + offset),
               arrowprops=dict(arrowstyle='<->', color=DIMENSION_COLOR, lw=1))
    ax.text((x1 + x2)/2, y + offset - 0.08, label, fontsize=SMALL_LABEL_SIZE,
            ha='center', va='top', color=DIMENSION_COLOR)


def draw_moment_arc(ax, x, y, radius=0.3, label=None, clockwise=True, color=MOMENT_COLOR):
    """Draw a moment arc with arrow."""
    if clockwise:
        theta1, theta2 = 45, 315
        arr_angle = 315
    else:
        theta1, theta2 = 45, 315
        arr_angle = 45

    arc = Arc((x, y), radius*2, radius*2, angle=0,
              theta1=theta1, theta2=theta2, color=color, lw=2)
    ax.add_patch(arc)

    # Arrow head
    arr_rad = np.radians(arr_angle)
    arr_x = x + radius * np.cos(arr_rad)
    arr_y = y + radius * np.sin(arr_rad)
    if clockwise:
        dx, dy = 0.05, -0.05
    else:
        dx, dy = -0.05, 0.05
    ax.annotate('', xy=(arr_x + dx, arr_y + dy), xytext=(arr_x, arr_y),
               arrowprops=dict(arrowstyle='->', color=color, lw=2))

    if label:
        ax.text(x, y - radius - 0.12, label, fontsize=LABEL_SIZE, color=color,
                ha='center', va='top')


def draw_box(ax, x, y, width, height, color='lightgray', label=None, label_size=SMALL_LABEL_SIZE):
    """Draw a labeled box."""
    box = FancyBboxPatch((x - width/2, y - height/2), width, height,
                         boxstyle="round,pad=0.02", linewidth=1.5,
                         edgecolor='black', facecolor=color)
    ax.add_patch(box)
    if label:
        ax.text(x, y, label, fontsize=label_size, ha='center', va='center')


def draw_circle(ax, x, y, radius, color='lightyellow', label=None):
    """Draw a labeled circle."""
    circle = plt.Circle((x, y), radius, linewidth=1.5, edgecolor='black', facecolor=color)
    ax.add_patch(circle)
    if label:
        ax.text(x, y, label, fontsize=SMALL_LABEL_SIZE, ha='center', va='center')


def draw_ellipse(ax, x, y, width, height, color='lightyellow', label=None):
    """Draw a labeled ellipse."""
    ellipse = patches.Ellipse((x, y), width, height, linewidth=1.5,
                               edgecolor='black', facecolor=color)
    ax.add_patch(ellipse)
    if label:
        ax.text(x, y, label, fontsize=SMALL_LABEL_SIZE, ha='center', va='center')


def save_diagram(fig, output_path):
    """Save diagram with consistent settings."""
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close()


# =============================================================================
# DIAGRAM FUNCTIONS FOR EACH SCENARIO
# =============================================================================

def diagram_suspended_ama(output_path: Path):
    """Suspended ama: aka as cantilever beam with tip and distributed loads."""
    fig, ax = setup_diagram(figsize=(10, 5))

    # Layout
    vaka_x, beam_y = 1.0, 2.5
    aka_start, aka_end = 1.5, 5.0

    # Vaka (support)
    draw_box(ax, vaka_x, beam_y, 0.8, 1.0, color=SUPPORT_COLOR, label='Vaka')
    draw_fixed_support(ax, aka_start, beam_y, width=0.1, height=0.3)

    # Aka beam
    draw_beam(ax, aka_start, beam_y, aka_end, height=0.12, label='Aka')

    # Pillar and ama at tip
    pillar_x = aka_end
    draw_box(ax, pillar_x, beam_y - 0.4, 0.15, 0.6, color='lightblue', label=None)
    ax.text(pillar_x + 0.2, beam_y - 0.6, 'Pillar', fontsize=SMALL_LABEL_SIZE, ha='left', va='center')
    draw_ellipse(ax, pillar_x, beam_y - 0.9, 0.7, 0.25, color='lightyellow', label='Ama')

    # Loads (pointing DOWN for gravity)
    draw_force_arrow(ax, pillar_x, beam_y - 0.06, dy=-0.5, label='Ftip', color=FORCE_COLOR, label_offset=(0.15, 0.15))
    draw_distributed_load(ax, aka_start + 0.3, aka_end - 0.3, beam_y - 0.06,
                         intensity=-0.35, label='w (self-weight)', n_arrows=6)

    # Dimension
    draw_dimension(ax, aka_start, aka_end, beam_y, 'L (cantilever)', offset=0.4)

    # Moment at support (with better positioning)
    draw_moment_arc(ax, aka_start + 0.25, beam_y + 0.4, radius=0.28, label='Mmax', clockwise=True)

    # Title
    ax.set_title('Suspended Ama: Aka as Cantilever Beam', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0, 6)
    ax.set_ylim(0.5, 4.0)
    save_diagram(fig, output_path)


def diagram_aka_point_load(output_path: Path):
    """Aka point load: simply supported beam with crew at center."""
    fig, ax = setup_diagram(figsize=(10, 5))

    # Layout
    beam_y = 2.0
    beam_start, beam_end = 1.0, 5.0
    mid_x = (beam_start + beam_end) / 2

    # Vaka (below aka, left edge aligns with left edge of aka - aka spans across)
    vaka_width = 0.6
    vaka_height = 0.7
    vaka_center_x = beam_start + vaka_width / 2  # left edge at beam_start
    draw_box(ax, vaka_center_x, beam_y - 0.4, vaka_width, vaka_height, color=SUPPORT_COLOR, label='Vaka')
    draw_pin_support(ax, vaka_center_x, beam_y - 0.06, size=0.12)

    # Pillar (below aka, right edge aligns with right edge of aka - aka spans across)
    pillar_width = 0.2
    pillar_height = 0.5
    pillar_center_x = beam_end - pillar_width / 2  # right edge at beam_end
    draw_box(ax, pillar_center_x, beam_y - 0.35, pillar_width, pillar_height, color='lightblue')
    ax.text(pillar_center_x + 0.25, beam_y - 0.35, 'Pillar', fontsize=SMALL_LABEL_SIZE, ha='left', va='center')
    draw_roller_support(ax, pillar_center_x, beam_y - 0.06, size=0.1)

    # Aka beam (sitting on top of vaka and pillar)
    draw_beam(ax, beam_start, beam_y, beam_end, height=0.1, label='Aka')

    # Crew (stick figure)
    head_y = beam_y + 0.55
    draw_circle(ax, mid_x, head_y, 0.08, color='lightblue')
    ax.plot([mid_x, mid_x], [head_y - 0.08, beam_y + 0.2], 'darkblue', lw=2)
    ax.plot([mid_x - 0.1, mid_x, mid_x + 0.1], [beam_y + 0.12, beam_y + 0.22, beam_y + 0.12], 'darkblue', lw=2)
    ax.text(mid_x + 0.2, beam_y + 0.4, 'Crew', fontsize=SMALL_LABEL_SIZE, ha='left')

    # Force arrow (pointing DOWN for weight, offset to left of person)
    draw_force_arrow(ax, mid_x - 0.25, beam_y + 0.6, dy=-0.45, label='P', color=FORCE_COLOR, label_offset=(-0.2, 0.1))

    # Reactions (pointing UP)
    draw_force_arrow(ax, beam_start, beam_y - 0.55, dy=0.35, label='R', color=REACTION_COLOR, label_offset=(0.1, -0.15))
    draw_force_arrow(ax, beam_end, beam_y - 0.55, dy=0.35, label='R', color=REACTION_COLOR, label_offset=(0.1, -0.15))

    # Dimension (above the person)
    draw_dimension(ax, beam_start, beam_end, beam_y, 'L (span)', offset=0.85)

    ax.set_title('Aka Point Load: Simply Supported Beam', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0, 6)
    ax.set_ylim(0.5, 3.5)
    save_diagram(fig, output_path)


def diagram_one_end_supported(output_path: Path):
    """One end supported: spine as continuous beam on elastic aka supports."""
    fig, ax = setup_diagram(figsize=(10, 5))

    # Layout - side view of outrigger
    spine_y = 2.2
    spine_start, spine_end = 1.0, 5.5

    # Spine beam
    draw_beam(ax, spine_start, spine_y, spine_end, height=0.1, label='Spine')

    # Fixed support (beach/dock) directly under the left end of spine
    support_x = spine_start + 0.15
    support_size = 0.12
    draw_pin_support(ax, support_x, spine_y - 0.06, size=support_size)
    # Box directly under the pin support triangle (aligned with triangle base)
    box_y = spine_y - 0.35 - support_size * 1.2
    draw_box(ax, support_x, box_y, 0.45, 0.28, color='burlywood', label=None)
    ax.text(support_x, box_y - 0.28, 'Beach/Dock', fontsize=SMALL_LABEL_SIZE, ha='center')

    # Aka positions with spring supports
    aka_positions = [1.8, 2.8, 3.8, 4.8]
    for i, x in enumerate(aka_positions):
        # Spring symbol
        draw_spring(ax, x, spine_y - 0.05, x, spine_y - 0.4, n_coils=4, width=0.06)
        # Small box for aka connection
        draw_box(ax, x, spine_y - 0.5, 0.12, 0.15, color=BEAM_COLOR)
        if i == 0:
            ax.text(x + 0.3, spine_y - 0.45, 'Aka (elastic\nsupport)', fontsize=SMALL_LABEL_SIZE, ha='left', va='center')

    # Distributed load (outrigger weight) - pointing DOWN
    draw_distributed_load(ax, spine_start + 0.2, spine_end - 0.2, spine_y + 0.05,
                         intensity=-0.35, label='Outrigger weight', n_arrows=7)

    # Deflected shape (dashed) - more visible
    x_defl = np.linspace(spine_start, spine_end, 50)
    y_defl = spine_y - 0.02 - 0.18 * ((x_defl - spine_start) / (spine_end - spine_start))**1.5
    ax.plot(x_defl, y_defl, 'b--', lw=1.5, alpha=0.6)
    ax.text(spine_end + 0.15, y_defl[-1] - 0.1, 'Deflected\nshape', fontsize=SMALL_LABEL_SIZE,
            color='blue', alpha=0.8, va='top', ha='left')

    ax.set_title('One End Supported: Spine on Elastic Aka Supports', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0, 6.5)
    ax.set_ylim(0.5, 3.5)
    save_diagram(fig, output_path)


def diagram_mast_wind(output_path: Path):
    """Mast wind loading: cantilever mast with wind force on sail."""
    fig, ax = setup_diagram(figsize=(8, 6))

    # Layout
    mast_x = 2.5
    deck_y = 1.0
    mast_top = 4.5
    partner_y = 1.5

    # Deck
    draw_beam(ax, 0.5, deck_y, 4.5, height=0.15, color=SUPPORT_COLOR, label=None)
    ax.text(2.5, deck_y, 'Deck', fontsize=SMALL_LABEL_SIZE, ha='center', va='center')

    # Mast
    draw_tube(ax, mast_x, deck_y + 0.08, mast_top, width=0.12, color=BEAM_COLOR, label='Mast')

    # Partner (support point)
    draw_box(ax, mast_x, partner_y, 0.35, 0.2, color='brown')
    ax.text(mast_x + 0.3, partner_y, 'Partner', fontsize=SMALL_LABEL_SIZE, ha='left', va='center')

    # Sail (tall rectangle) - on lee side of mast (left side, away from wind)
    sail_bottom = 2.3
    sail_top = 4.2
    sail_width = 0.8
    sail = patches.Rectangle((mast_x - 0.1 - sail_width, sail_bottom), sail_width, sail_top - sail_bottom,
                               linewidth=1, edgecolor='darkgray', facecolor='lightyellow', alpha=0.7)
    ax.add_patch(sail)
    ax.text(mast_x - 0.1 - sail_width/2, (sail_bottom + sail_top)/2, 'Sail',
            fontsize=SMALL_LABEL_SIZE, ha='center', va='center')
    ax.text(mast_x - 0.1 - sail_width/2, sail_bottom - 0.1, '(lee side)',
            fontsize=SMALL_LABEL_SIZE - 1, ha='center', va='top', style='italic', color='gray')

    # Wind force at CE (from the right/windward side)
    ce_y = (sail_bottom + sail_top) / 2
    draw_force_arrow(ax, mast_x + 1.0, ce_y, dx=-0.6, label='Fwind', color=FORCE_COLOR, label_offset=(0.1, 0.1))
    ax.plot(mast_x + 0.06, ce_y, 'ko', markersize=5)
    ax.text(mast_x + 0.2, ce_y + 0.15, 'CE', fontsize=SMALL_LABEL_SIZE, ha='left', va='bottom')

    # Moment at partner (repositioned for clarity)
    draw_moment_arc(ax, mast_x - 0.5, partner_y + 0.5, radius=0.3, label=None, clockwise=False)
    ax.text(mast_x - 0.9, partner_y + 0.3, 'Mmax', fontsize=LABEL_SIZE, color=MOMENT_COLOR,
            ha='right', va='center')

    # Dimensions
    ax.annotate('', xy=(mast_x - 0.7, partner_y), xytext=(mast_x - 0.7, ce_y),
               arrowprops=dict(arrowstyle='<->', color=DIMENSION_COLOR, lw=1))
    ax.text(mast_x - 0.85, (partner_y + ce_y)/2, 'h', fontsize=LABEL_SIZE,
            ha='right', va='center', color=DIMENSION_COLOR)

    ax.set_title('Mast Wind Loading: Bending at Partner', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5.5)
    save_diagram(fig, output_path)


def diagram_diagonal_braces(output_path: Path):
    """Diagonal braces: lateral loading when boat is tilted."""
    fig, ax = setup_diagram(figsize=(10, 5))

    # Layout - side view of outrigger structure tilted
    aka_y = 2.5
    pillar_bottom = 1.2
    pillar_x = 4.9  # pillar at end of aka
    aka_start, aka_end = 1.5, pillar_x  # aka ends at pillar (no overhang)

    # Vaka attachment
    draw_box(ax, aka_start - 0.3, aka_y, 0.5, 0.6, color=SUPPORT_COLOR, label='Vaka')

    # Aka (horizontal beam, ends at pillar)
    draw_beam(ax, aka_start, aka_y, aka_end, height=0.1, label='Aka')

    # Pillar (vertical)
    ax.plot([pillar_x, pillar_x], [aka_y - 0.05, pillar_bottom], 'k-', lw=3)
    ax.text(pillar_x + 0.15, (aka_y + pillar_bottom)/2 + 0.2, 'Pillar', fontsize=SMALL_LABEL_SIZE, ha='left', va='center')

    # Ama at bottom
    draw_ellipse(ax, pillar_x, pillar_bottom - 0.2, 0.8, 0.3, color='lightyellow', label='Ama')

    # Diagonal brace (single, from aka toward vaka down to pillar bottom)
    brace_attach = aka_start + 2.3  # attachment point on aka (toward vaka)
    brace_bottom = pillar_bottom + 0.3
    ax.plot([brace_attach, pillar_x], [aka_y - 0.05, brace_bottom], 'b-', lw=3)
    ax.text(brace_attach - 0.15, aka_y + 0.1, 'Brace',
            fontsize=SMALL_LABEL_SIZE, ha='right', va='bottom', color='blue')

    # Lateral force (outrigger weight when tilted)
    draw_force_arrow(ax, pillar_x + 0.15, pillar_bottom - 0.2, dx=0.8,
                    label='Flateral', color=FORCE_COLOR, label_offset=(0.05, 0.12))

    # Compression annotation (clearer)
    mid_brace_x = (brace_attach + pillar_x) / 2
    mid_brace_y = (aka_y - 0.05 + brace_bottom) / 2
    ax.annotate('', xy=(mid_brace_x + 0.12, mid_brace_y + 0.12),
               xytext=(mid_brace_x - 0.12, mid_brace_y - 0.12),
               arrowprops=dict(arrowstyle='->', color=MOMENT_COLOR, lw=2))
    ax.text(mid_brace_x - 0.35, mid_brace_y + 0.05, 'Compression',
            fontsize=SMALL_LABEL_SIZE, color=MOMENT_COLOR, ha='right', va='center')

    # Note about scenario
    ax.text(5.6, 3.0, 'Boat tilted\n(ama lifted)', fontsize=SMALL_LABEL_SIZE,
            ha='left', va='top', style='italic', color='gray')

    ax.set_title('Diagonal Braces: Lateral Loading (Boat Tilted)', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0.5, 6.5)
    ax.set_ylim(0, 3.8)
    save_diagram(fig, output_path)


def diagram_wave_slam_vertical(output_path: Path):
    """Wave slam vertical: ama impact from below, load path through structure."""
    fig, ax = setup_diagram(figsize=(10, 5.5))

    # Layout
    aka_y = 2.8
    pillar_top = aka_y - 0.05
    pillar_bottom = 1.5
    pillar_x = 4.9  # pillar at end of aka
    aka_start, aka_end = 1.2, pillar_x  # aka ends at pillar (no overhang)

    # Vaka
    draw_box(ax, aka_start - 0.3, aka_y, 0.5, 0.6, color=SUPPORT_COLOR, label='Vaka')

    # Aka beam (ends at pillar)
    draw_beam(ax, aka_start, aka_y, aka_end, height=0.1, label='Aka')

    # Pillar
    ax.plot([pillar_x, pillar_x], [pillar_top, pillar_bottom], 'k-', lw=3)
    ax.text(pillar_x + 0.12, (pillar_top + pillar_bottom)/2, 'Pillar',
            fontsize=SMALL_LABEL_SIZE, ha='left', va='center')

    # Diagonal brace
    brace_attach = aka_start + 2.3
    ax.plot([brace_attach, pillar_x], [aka_y - 0.05, pillar_bottom + 0.3], 'b-', lw=2.5)
    ax.text(brace_attach + 0.35, (aka_y + pillar_bottom)/2 + 0.25, 'Brace',
            fontsize=SMALL_LABEL_SIZE, ha='left', color='blue')

    # Ama
    draw_ellipse(ax, pillar_x, pillar_bottom - 0.2, 1.0, 0.35, color='lightyellow', label='Ama')

    # Water surface (wavy line)
    water_y = 0.8
    x_water = np.linspace(0.5, 6.0, 100)
    y_water = water_y + 0.08 * np.sin(x_water * 4)
    ax.fill_between(x_water, 0, y_water, color='lightblue', alpha=0.4)
    ax.plot(x_water, y_water, 'b-', lw=1, alpha=0.7)
    ax.text(0.7, 0.4, 'Water', fontsize=SMALL_LABEL_SIZE, color='blue')

    # Wave slam force (upward from water into ama)
    draw_force_arrow(ax, pillar_x, pillar_bottom - 0.9, dy=0.5,
                    label='Fslam', color=FORCE_COLOR, label_offset=(0.15, -0.1))

    # Load path arrows
    ax.annotate('', xy=(pillar_x, pillar_top - 0.1), xytext=(pillar_x, pillar_bottom + 0.1),
               arrowprops=dict(arrowstyle='->', color='orange', lw=2, ls='--'))
    ax.annotate('', xy=(brace_attach, aka_y - 0.15), xytext=(pillar_x - 0.1, pillar_bottom + 0.4),
               arrowprops=dict(arrowstyle='->', color='orange', lw=2, ls='--'))
    ax.annotate('', xy=(aka_start + 0.1, aka_y), xytext=(brace_attach - 0.1, aka_y),
               arrowprops=dict(arrowstyle='->', color='orange', lw=2, ls='--'))

    ax.text(3.0, 3.3, 'Load path', fontsize=SMALL_LABEL_SIZE, color='orange', style='italic')

    ax.set_title('Wave Slam (Vertical): Impact Load Path', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0.3, 6.0)
    ax.set_ylim(-0.1, 4.0)
    save_diagram(fig, output_path)


def diagram_frontal_wave_slam(output_path: Path):
    """Frontal wave slam: fore-aft impact resisted by X-braces."""
    fig, ax = setup_diagram(figsize=(10, 5))

    # Layout - side view showing X-braces between pillars
    base_y = 1.0
    pillar_height = 1.2

    # Two pillars
    p1_x, p2_x = 1.5, 4.0
    ax.plot([p1_x, p1_x], [base_y, base_y + pillar_height], 'k-', lw=3)
    ax.plot([p2_x, p2_x], [base_y, base_y + pillar_height], 'k-', lw=3)
    ax.text(p1_x, base_y + pillar_height + 0.1, 'Pillar', fontsize=SMALL_LABEL_SIZE, ha='center')
    ax.text(p2_x, base_y + pillar_height + 0.1, 'Pillar', fontsize=SMALL_LABEL_SIZE, ha='center')

    # X-braces
    ax.plot([p1_x, p2_x], [base_y + 0.1, base_y + pillar_height - 0.1], 'b-', lw=2)
    ax.plot([p1_x, p2_x], [base_y + pillar_height - 0.1, base_y + 0.1], 'b--', lw=2, alpha=0.5)
    ax.text((p1_x + p2_x)/2 + 0.3, base_y + pillar_height/2 + 0.2, 'Tension',
            fontsize=SMALL_LABEL_SIZE, color='blue')
    ax.text((p1_x + p2_x)/2 - 0.1, base_y + pillar_height/2 - 0.3, 'Buckled',
            fontsize=SMALL_LABEL_SIZE, color='blue', alpha=0.5)

    # Spine at top
    draw_beam(ax, p1_x - 0.3, base_y + pillar_height, p2_x + 0.3, height=0.08, color='gray', label=None)
    ax.text((p1_x + p2_x)/2, base_y + pillar_height, 'Spine', fontsize=SMALL_LABEL_SIZE,
            ha='center', va='center', color='white')

    # Ama at bottom - elongated ellipse for side view, spanning pillar width
    ama_center_x = (p1_x + p2_x) / 2
    ama_width = p2_x - p1_x + 0.4  # Covers full pillar span plus a bit
    draw_ellipse(ax, ama_center_x, base_y - 0.25, ama_width, 0.3, color='lightyellow', label='Ama')

    # Frontal slam force
    draw_force_arrow(ax, p1_x - 0.8, base_y - 0.25, dx=0.5,
                    label='Fslam', color=FORCE_COLOR, label_offset=(-0.5, 0.15))

    # Wave symbol
    ax.text(0.3, base_y - 0.25, 'Wave', fontsize=SMALL_LABEL_SIZE, color='blue', ha='center')

    ax.set_title('Frontal Wave Slam: X-Braces in Tension', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(-0.2, 5)
    ax.set_ylim(0, 3.2)
    save_diagram(fig, output_path)


def diagram_sideways_wave_slam(output_path: Path):
    """Sideways wave slam: lateral impact resisted by diagonal braces."""
    fig, ax = setup_diagram(figsize=(10, 5))

    # Layout
    aka_y = 2.5
    pillar_bottom = 1.2
    pillar_x = 4.8  # pillar at end of aka
    aka_start, aka_end = 1.5, pillar_x  # aka ends at pillar (no overhang)

    # Aka
    draw_beam(ax, aka_start, aka_y, aka_end, height=0.1, label='Aka')

    # Vaka
    draw_box(ax, aka_start - 0.3, aka_y, 0.5, 0.6, color=SUPPORT_COLOR, label='Vaka')

    # Pillar
    ax.plot([pillar_x, pillar_x], [aka_y - 0.05, pillar_bottom], 'k-', lw=3)
    ax.text(pillar_x + 0.12, (aka_y + pillar_bottom)/2, 'Pillar',
            fontsize=SMALL_LABEL_SIZE, ha='left', va='center')

    # Diagonal brace (only one, going toward vaka)
    brace_attach_x = aka_start + 2.0
    ax.plot([brace_attach_x, pillar_x], [aka_y - 0.05, pillar_bottom + 0.3], 'b-', lw=3)
    ax.text(brace_attach_x - 0.15, (aka_y + pillar_bottom)/2 + 0.4, 'Brace\n(compression)',
            fontsize=SMALL_LABEL_SIZE, ha='right', va='center', color='blue')

    # Ama
    draw_ellipse(ax, pillar_x, pillar_bottom - 0.2, 1.0, 0.35, color='lightyellow', label='Ama')

    # Water/wave on side
    water_x = 5.6
    y_water = np.linspace(0.5, 1.5, 20)
    x_water = water_x + 0.1 * np.sin(y_water * 8)
    ax.fill_betweenx(y_water, water_x + 0.3, x_water, color='lightblue', alpha=0.5)
    ax.plot(x_water, y_water, 'b-', lw=1)
    ax.text(water_x + 0.4, 1.0, 'Wave', fontsize=SMALL_LABEL_SIZE, color='blue')

    # Sideways slam force
    draw_force_arrow(ax, pillar_x + 0.7, pillar_bottom - 0.2, dx=-0.5,
                    label='Fslam', color=FORCE_COLOR, label_offset=(0.1, 0.15))

    ax.set_title('Sideways Wave Slam: Diagonal Braces in Compression', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0.5, 6.5)
    ax.set_ylim(0, 3.5)
    save_diagram(fig, output_path)


def diagram_lifting_sling(output_path: Path):
    """Lifting sling: V-sling configuration with 4 hooks to neighboring akas."""
    fig, ax = setup_diagram(figsize=(10, 6))

    # Layout - front view of boat being lifted
    # Vaka (main hull) on LEFT, Ama (outrigger) hangs BELOW right end of akas
    # Akas extend horizontally across full width of vaka and beyond to ama
    boat_y = 2.0
    hook_y_front = 4.2
    hook_y_rear = 4.6  # Slightly higher to show depth

    # Vaka position and size
    vaka_left = 0.3
    vaka_right = 1.5
    vaka_center_x = (vaka_left + vaka_right) / 2

    # Aka extends from left of vaka to right where ama hangs
    aka_left = vaka_left
    aka_right = 5.2

    # Ama hangs below the right end of the akas
    ama_x = aka_right - 0.1  # Directly under aka tips
    ama_y = boat_y - 1.0  # Below the akas

    # Vaka (main hull) - larger box
    draw_box(ax, vaka_center_x, boat_y - 0.3, vaka_right - vaka_left, 0.9, color=SUPPORT_COLOR, label='Vaka')

    # Four akas spanning from vaka left edge to ama (shown as 2 pairs: front and rear)
    # Front pair (outer and inner) - solid blue
    aka_outer_y = boat_y + 0.25  # outer aka
    aka_inner_y = boat_y - 0.05  # inner aka
    draw_beam(ax, aka_left, aka_outer_y, aka_right, height=0.1, color=BEAM_COLOR)
    draw_beam(ax, aka_left, aka_inner_y, aka_right, height=0.1, color=BEAM_COLOR)

    # Rear pair (outer and inner) - lighter to show depth
    aka_outer_rear_y = boat_y + 0.1
    aka_inner_rear_y = boat_y - 0.2
    draw_beam(ax, aka_left, aka_outer_rear_y, aka_right, height=0.1, color='lightsteelblue')
    draw_beam(ax, aka_left, aka_inner_rear_y, aka_right, height=0.1, color='lightsteelblue')

    # Ama (outrigger) - smaller, elliptical, below right end of akas
    draw_ellipse(ax, ama_x, ama_y, 0.5, 0.35, color='lightyellow', label='Ama')

    # Connection lines from akas to ama (pillars/iakos)
    ax.plot([aka_right, ama_x], [aka_inner_y, ama_y + 0.3], 'k-', lw=1.5)
    ax.plot([aka_right, ama_x], [aka_inner_rear_y, ama_y + 0.3], 'k-', lw=1, alpha=0.6)

    # V-sling x-offset between the two legs
    v_offset = 0.4

    # === VAKA SIDE: Front V-sling (solid) ===
    # Left leg: far left end of lowest (outer) aka
    # Right leg: slightly to the right, on second-lowest (inner) aka
    vaka_left_attach_x = aka_left  # Far left end
    vaka_right_attach_x = aka_left + v_offset  # Slightly to the right
    # Hook positioned above
    hook_vaka_front_x = (vaka_left_attach_x + vaka_right_attach_x) / 2

    ax.plot(hook_vaka_front_x, hook_y_front, 'ks', markersize=10)
    ax.plot([hook_vaka_front_x, vaka_left_attach_x], [hook_y_front, aka_outer_y], 'g-', lw=2.5)  # to outer aka
    ax.plot([hook_vaka_front_x, vaka_right_attach_x], [hook_y_front, aka_inner_y], 'g-', lw=2.5)  # to inner aka

    # === AMA SIDE: Front V-sling (solid) - symmetrical ===
    # Right leg: far right end of lowest (outer) aka
    # Left leg: slightly to the left, on second-lowest (inner) aka
    ama_right_attach_x = aka_right  # Far right end
    ama_left_attach_x = aka_right - v_offset  # Slightly to the left
    # Hook positioned above
    hook_ama_front_x = (ama_left_attach_x + ama_right_attach_x) / 2

    ax.plot(hook_ama_front_x, hook_y_front, 'ks', markersize=10)
    ax.plot([hook_ama_front_x, ama_right_attach_x], [hook_y_front, aka_outer_y], 'g-', lw=2.5)  # to outer aka
    ax.plot([hook_ama_front_x, ama_left_attach_x], [hook_y_front, aka_inner_y], 'g-', lw=2.5)  # to inner aka

    # === REAR V-slings (dashed) - placeholder, will refine later ===
    hook_vaka_rear_x = hook_vaka_front_x + 0.2
    ax.plot(hook_vaka_rear_x, hook_y_rear, 'ks', markersize=10, alpha=0.6)
    ax.plot([hook_vaka_rear_x, vaka_left_attach_x], [hook_y_rear, aka_outer_rear_y],
            color='green', ls='--', lw=2, alpha=0.7)
    ax.plot([hook_vaka_rear_x, vaka_right_attach_x], [hook_y_rear, aka_inner_rear_y],
            color='green', ls='--', lw=2, alpha=0.7)

    hook_ama_rear_x = hook_ama_front_x - 0.2
    ax.plot(hook_ama_rear_x, hook_y_rear, 'ks', markersize=10, alpha=0.6)
    ax.plot([hook_ama_rear_x, ama_right_attach_x], [hook_y_rear, aka_outer_rear_y],
            color='green', ls='--', lw=2, alpha=0.7)
    ax.plot([hook_ama_rear_x, ama_left_attach_x], [hook_y_rear, aka_inner_rear_y],
            color='green', ls='--', lw=2, alpha=0.7)

    # Crane lines from all 4 hooks
    ax.plot([hook_vaka_front_x, hook_vaka_front_x], [hook_y_front + 0.1, hook_y_front + 0.5], 'k-', lw=2.5)
    ax.plot([hook_vaka_rear_x, hook_vaka_rear_x], [hook_y_rear + 0.1, hook_y_rear + 0.5], 'k-', lw=2, alpha=0.6)
    ax.plot([hook_ama_front_x, hook_ama_front_x], [hook_y_front + 0.1, hook_y_front + 0.5], 'k-', lw=2.5)
    ax.plot([hook_ama_rear_x, hook_ama_rear_x], [hook_y_rear + 0.1, hook_y_rear + 0.5], 'k-', lw=2, alpha=0.6)

    # Crane arrow (centered between the two hook sets)
    center_x = (hook_vaka_front_x + hook_ama_front_x) / 2
    ax.annotate('', xy=(center_x, hook_y_rear + 0.8),
               xytext=(center_x, hook_y_rear + 0.4),
               arrowprops=dict(arrowstyle='->', color='black', lw=2))
    ax.text(center_x, hook_y_rear + 0.95, 'Crane', fontsize=LABEL_SIZE, ha='center')

    # V-angle annotation on ama side front hook
    ax.annotate('', xy=(hook_ama_front_x - 0.4, hook_y_front - 0.35),
               xytext=(hook_ama_front_x - 0.05, hook_y_front - 0.05),
               arrowprops=dict(arrowstyle='-', color=DIMENSION_COLOR, lw=1.5,
                             connectionstyle='arc3,rad=0.15'))
    ax.text(hook_ama_front_x - 0.55, hook_y_front - 0.2, 'θ', fontsize=LABEL_SIZE + 2, color=DIMENSION_COLOR)

    # Labels
    ax.text(hook_vaka_front_x - 0.4, (hook_y_front + hook_y_rear) / 2, 'Vaka\nside',
            fontsize=SMALL_LABEL_SIZE, ha='right', va='center')
    ax.text(hook_ama_front_x + 0.4, (hook_y_front + hook_y_rear) / 2, 'Ama\nside',
            fontsize=SMALL_LABEL_SIZE, ha='left', va='center')

    # Aka label - simple label in the middle
    ax.text(center_x, boat_y + 0.5, 'Akas', fontsize=SMALL_LABEL_SIZE, ha='center', va='bottom')

    # Legend for front/rear distinction
    ax.plot([], [], 'g-', lw=2.5, label='Front slings')
    ax.plot([], [], 'g--', lw=2, alpha=0.7, label='Rear slings')
    ax.legend(loc='upper right', fontsize=SMALL_LABEL_SIZE, framealpha=0.9)

    # Note
    ax.text(center_x, 0.35, '(Front view: 4 hooks total, 2 per side, each V connects outer+inner akas)',
            fontsize=SMALL_LABEL_SIZE, ha='center', style='italic', color='gray')

    ax.set_title('Lifting Sling: V-Configuration to Neighboring Akas', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(-0.3, 6.0)
    ax.set_ylim(0, 5.8)
    save_diagram(fig, output_path)


def diagram_gunwale_loads(output_path: Path):
    """Gunwale loads: load distribution from aka to hull through gunwale."""
    fig, ax = setup_diagram(figsize=(10, 5.5))

    # Layout - cross-section view
    hull_y = 1.2
    gunwale_y = 2.2
    aka_y = 2.5

    # Hull (curved bottom)
    hull_x = np.linspace(1, 5, 50)
    hull_bottom = hull_y - 0.6 + 0.35 * (hull_x - 3)**2 / 4
    ax.fill_between(hull_x, hull_bottom, hull_y, color='lightgray', alpha=0.5)
    ax.plot(hull_x, hull_bottom, 'k-', lw=2)
    ax.text(3, hull_y - 0.35, 'Hull', fontsize=SMALL_LABEL_SIZE, ha='center')

    # Freeboard (vertical hull sides with thickness)
    freeboard_thickness = 0.12
    freeboard_top = gunwale_y + 0.05

    # Left freeboard - outer edge at x=1, inner edge at x=1+thickness
    left_outer = 1.0
    left_inner = left_outer + freeboard_thickness
    # Draw filled rectangle for left freeboard
    ax.fill([left_outer, left_inner, left_inner, left_outer],
            [hull_y, hull_y, freeboard_top, freeboard_top],
            color='lightgray', edgecolor='black', lw=1.5)

    # Right freeboard - outer edge at x=5, inner edge at x=5-thickness
    right_outer = 5.0
    right_inner = right_outer - freeboard_thickness
    # Draw filled rectangle for right freeboard
    ax.fill([right_inner, right_outer, right_outer, right_inner],
            [hull_y, hull_y, freeboard_top, freeboard_top],
            color='lightgray', edgecolor='black', lw=1.5)

    # Gunwales (3"x2" wood)
    # Outer edge aligns with freeboard outer edge, extends inward past freeboard inner edge
    gunwale_width = 0.35  # Wider than freeboard thickness
    # Left gunwale: outer edge at left_outer, extends inward
    draw_beam(ax, left_outer, gunwale_y, left_outer + gunwale_width, height=0.18, color='burlywood')
    # Right gunwale: outer edge at right_outer, extends inward
    draw_beam(ax, right_outer - gunwale_width, gunwale_y, right_outer, height=0.18, color='burlywood')
    ax.text(0.5, gunwale_y, 'Gunwale\n(3"×2")', fontsize=SMALL_LABEL_SIZE, ha='right', va='center')

    # Aka
    draw_beam(ax, 0.4, aka_y, 5.6, height=0.12, color=BEAM_COLOR, label='Aka')

    # Force from aka (pointing DOWN - gravity from suspended ama)
    draw_force_arrow(ax, 1.05, aka_y + 0.5, dy=-0.35, label='Faka', color=FORCE_COLOR, label_offset=(0.12, 0.1))
    draw_force_arrow(ax, 4.95, aka_y + 0.5, dy=-0.35, color=FORCE_COLOR)

    # Distribution arrows (spreading load into hull)
    for dx in [-0.25, -0.12, 0, 0.12, 0.25]:
        ax.annotate('', xy=(1.05 + dx, hull_y + 0.15), xytext=(1.05 + dx, gunwale_y - 0.12),
                   arrowprops=dict(arrowstyle='->', color='orange', lw=1.5, alpha=0.8))

    # Distribution length (characteristic length lambda) - larger font
    ax.annotate('', xy=(0.75, gunwale_y - 0.35), xytext=(1.35, gunwale_y - 0.35),
               arrowprops=dict(arrowstyle='<->', color=DIMENSION_COLOR, lw=1.5))
    ax.text(1.05, gunwale_y - 0.55, 'λ (char. length)', fontsize=LABEL_SIZE, ha='center', color=DIMENSION_COLOR)

    # Glass bond annotation
    ax.annotate('Fiberglass\nbond', xy=(1.25, gunwale_y - 0.1), xytext=(1.9, gunwale_y - 0.5),
               fontsize=SMALL_LABEL_SIZE, ha='left',
               arrowprops=dict(arrowstyle='->', color='gray', lw=1))

    ax.set_title('Gunwale Loads: Distribution from Aka to Hull', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0, 6)
    ax.set_ylim(0, 3.8)
    save_diagram(fig, output_path)


def diagram_ama_lift_wind(output_path: Path):
    """Ama lift wind speed: heeling moment vs righting moment."""
    fig, ax = setup_diagram(figsize=(10, 5.5))

    # Layout - front view of boat heeling to the left (ama on right, lifted)
    water_y = 1.2

    # Water surface
    ax.fill_between([0, 6], [0, 0], [water_y, water_y], color='lightblue', alpha=0.3)
    ax.plot([0, 6], [water_y, water_y], 'b-', lw=1)

    # Heel angle (negative = tilting left)
    heel_angle = -12  # degrees

    # Vaka - draw as a boat shape (better representation)
    vaka_center_x, vaka_center_y = 2.0, water_y + 0.15
    # Create a simple hull cross-section shape
    hull_pts = [
        (vaka_center_x - 0.4, vaka_center_y + 0.25),  # deck left
        (vaka_center_x + 0.4, vaka_center_y + 0.25),  # deck right
        (vaka_center_x + 0.25, vaka_center_y - 0.2),  # bottom right
        (vaka_center_x, vaka_center_y - 0.3),         # keel
        (vaka_center_x - 0.25, vaka_center_y - 0.2),  # bottom left
    ]
    # Rotate hull points (negate angle so hull tilts left with deck aligned to aka)
    cos_a, sin_a = np.cos(np.radians(-heel_angle)), np.sin(np.radians(-heel_angle))
    rotated_pts = []
    for px, py in hull_pts:
        dx, dy = px - vaka_center_x, py - vaka_center_y
        rx = vaka_center_x + dx * cos_a - dy * sin_a
        ry = vaka_center_y + dx * sin_a + dy * cos_a
        rotated_pts.append((rx, ry))
    hull = Polygon(rotated_pts, closed=True, linewidth=2, edgecolor='black', facecolor=SUPPORT_COLOR)
    ax.add_patch(hull)
    ax.text(vaka_center_x, vaka_center_y - 0.6, 'Vaka', fontsize=SMALL_LABEL_SIZE, ha='center')

    # Mast (tilted with heel, perpendicular to deck)
    mast_base_x = vaka_center_x
    mast_base_y = vaka_center_y + 0.25
    mast_height = 2.2
    # Mast tilts left with the boat
    mast_top_x = mast_base_x + mast_height * np.sin(np.radians(heel_angle))
    mast_top_y = mast_base_y + mast_height * np.cos(np.radians(heel_angle))
    ax.plot([mast_base_x, mast_top_x], [mast_base_y, mast_top_y], color=BEAM_COLOR, lw=4)
    ax.text(mast_top_x - 0.3, mast_top_y, 'Mast', fontsize=SMALL_LABEL_SIZE, ha='right')

    # Sail (rectangular, on lee side - left of mast, away from wind)
    sail_height = 1.6
    sail_width = 0.7
    sail_bottom = mast_base_y + 0.5
    # Calculate sail corners rotated with mast
    sail_base_x = mast_base_x + (sail_bottom - mast_base_y) * np.sin(np.radians(heel_angle))
    sail_base_y = mast_base_y + (sail_bottom - mast_base_y) * np.cos(np.radians(heel_angle))
    sail_top_x = sail_base_x + sail_height * np.sin(np.radians(heel_angle))
    sail_top_y = sail_base_y + sail_height * np.cos(np.radians(heel_angle))
    # Offset to lee side (left, perpendicular to mast)
    lee_offset_x = -sail_width * np.cos(np.radians(heel_angle))
    lee_offset_y = -sail_width * np.sin(np.radians(heel_angle))
    sail_pts = [
        (sail_base_x, sail_base_y),
        (sail_top_x, sail_top_y),
        (sail_top_x + lee_offset_x, sail_top_y + lee_offset_y),
        (sail_base_x + lee_offset_x, sail_base_y + lee_offset_y),
    ]
    sail = Polygon(sail_pts, closed=True, linewidth=1, edgecolor='darkgray',
                   facecolor='lightyellow', alpha=0.7)
    ax.add_patch(sail)
    sail_center_x = (sail_base_x + sail_top_x) / 2 + lee_offset_x / 2
    sail_center_y = (sail_base_y + sail_top_y) / 2 + lee_offset_y / 2
    ax.text(sail_center_x - 0.2, sail_center_y, 'Sail', fontsize=SMALL_LABEL_SIZE, ha='right')

    # Aka (perpendicular to mast, extending to ama on right)
    # Aka attaches at deck level, goes horizontally to ama
    aka_start_x = vaka_center_x + 0.3
    aka_start_y = vaka_center_y + 0.2
    ama_x = 4.8
    # Ama is lifted (right side up when tilting left)
    ama_y = water_y + 0.9
    # Draw aka as horizontal beam (perpendicular to tilted mast)
    ax.plot([aka_start_x, ama_x], [aka_start_y, ama_y], 'k-', lw=2.5)
    ax.text((aka_start_x + ama_x) / 2, (aka_start_y + ama_y) / 2 + 0.2, 'Aka',
            fontsize=SMALL_LABEL_SIZE, ha='center')
    draw_ellipse(ax, ama_x, ama_y, 0.55, 0.28, color='lightyellow')
    ax.text(ama_x, ama_y + 0.4, 'Ama', fontsize=SMALL_LABEL_SIZE, ha='center')

    # Wind force at center of effort (from the right/ama side)
    ce_x = (sail_base_x + sail_top_x) / 2
    ce_y = (sail_base_y + sail_top_y) / 2
    draw_force_arrow(ax, ce_x + 1.2, ce_y, dx=-0.7, label='Fwind', color=FORCE_COLOR, label_offset=(0.1, 0.1))

    # Heeling moment arc (counter-clockwise, pushing boat to tilt left)
    ax.annotate('', xy=(1.3, water_y + 1.0), xytext=(1.6, water_y + 1.8),
               arrowprops=dict(arrowstyle='->', color=MOMENT_COLOR, lw=2.5,
                             connectionstyle='arc3,rad=0.35'))
    ax.text(1.0, water_y + 1.4, 'Mheel', fontsize=LABEL_SIZE, color=MOMENT_COLOR)

    # Righting moment (from ama weight as counterweight, clockwise)
    ax.annotate('', xy=(4.5, ama_y + 0.5), xytext=(4.2, ama_y - 0.3),
               arrowprops=dict(arrowstyle='->', color=REACTION_COLOR, lw=2.5,
                             connectionstyle='arc3,rad=0.35'))
    ax.text(4.7, ama_y + 0.1, 'Mright', fontsize=LABEL_SIZE, color=REACTION_COLOR)

    # Weight arrow on ama (pointing DOWN)
    draw_force_arrow(ax, ama_x, ama_y - 0.15, dy=-0.5, label='Wama', color='darkblue', label_offset=(0.15, 0.1))

    # Explanatory note
    ax.text(0.3, 0.4, 'At ama lift wind speed:\nMheel = Mright',
            fontsize=SMALL_LABEL_SIZE, ha='left', va='bottom', style='italic', color='gray',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    ax.set_title('Ama Lift Wind Speed: Heeling vs Righting Moment', fontsize=TITLE_SIZE, fontweight='bold', pad=20)

    ax.set_xlim(0, 6)
    ax.set_ylim(0, 4.2)
    save_diagram(fig, output_path)


def generate_all_diagrams(output_dir: Path):
    """Generate all structural diagrams."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    diagrams = [
        ('suspended_ama.png', diagram_suspended_ama),
        ('aka_point_load.png', diagram_aka_point_load),
        ('one_end_supported.png', diagram_one_end_supported),
        ('mast_wind.png', diagram_mast_wind),
        ('diagonal_braces.png', diagram_diagonal_braces),
        ('wave_slam_vertical.png', diagram_wave_slam_vertical),
        ('frontal_wave_slam.png', diagram_frontal_wave_slam),
        ('sideways_wave_slam.png', diagram_sideways_wave_slam),
        ('lifting_sling.png', diagram_lifting_sling),
        ('gunwale_loads.png', diagram_gunwale_loads),
        ('ama_lift_wind.png', diagram_ama_lift_wind),
    ]

    generated = []
    for filename, func in diagrams:
        output_path = output_dir / filename
        func(output_path)
        print(f"Generated: {output_path}")
        generated.append(output_path)

    return generated


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Generate structural diagrams')
    parser.add_argument('--output-dir', default='docs/diagrams',
                       help='Output directory for diagrams')
    args = parser.parse_args()

    generate_all_diagrams(Path(args.output_dir))
