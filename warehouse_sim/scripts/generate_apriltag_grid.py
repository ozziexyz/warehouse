#!/usr/bin/env python3
"""Generates an SDF XML snippet placing one AprilTag model per path-planner grid
intersection, face-up on the floor. Paste the output into worlds/warehouse.sdf
inside the <world> element.

Grid defaults mirror warehouse_control/src/path_planner.cpp's declared
parameters (grid_width, grid_height, resolution, origin_x, origin_y), so tags
land exactly on the grid lines the robot plans along.
"""

import argparse
from pathlib import Path

MODEL_NAME = 'apriltag'
TEXTURE_DIR = Path(__file__).resolve().parent.parent / 'models' / MODEL_NAME / 'materials' / 'textures'

MODEL_TEMPLATE = """    <model name="floor_tag_{tag_id}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{size} {size} {thickness}</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0 0 0 1</specular>
            <pbr>
              <metal>
                <albedo_map>model://{model_name}/materials/textures/tag36h11-{tag_id}.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>{collision}
      </link>
    </model>"""

COLLISION_TEMPLATE = """
        <collision name="collision">
          <geometry>
            <box>
              <size>{size} {size} {thickness}</size>
            </box>
          </geometry>
        </collision>"""


def build_snippet(args):
    expected = args.grid_width * args.grid_height
    if args.tag_count != expected:
        raise ValueError(
            f'grid is {args.grid_width}x{args.grid_height} ({expected} cells) '
            f'but tag_count is {args.tag_count} — pass matching --grid-width/--grid-height/--tag-count'
        )

    missing = [i for i in range(args.tag_count) if not (TEXTURE_DIR / f'tag36h11-{i}.png').exists()]
    if missing:
        raise FileNotFoundError(f'missing texture files in {TEXTURE_DIR}: {missing}')

    z = args.thickness / 2 + args.z_lift
    collision = COLLISION_TEMPLATE.format(size=args.size, thickness=args.thickness) if not args.no_collision else ''

    blocks = []
    for row in range(args.grid_height):
        for col in range(args.grid_width):
            tag_id = row * args.grid_width + col
            x = args.origin_x + col * args.resolution
            y = args.origin_y - row * args.resolution
            blocks.append(MODEL_TEMPLATE.format(
                tag_id=tag_id,
                x=x, y=y, z=z,
                yaw=args.yaw,
                size=args.size,
                thickness=args.thickness,
                collision=collision,
                model_name=MODEL_NAME,
            ))

    header = f'    <!-- {args.tag_count} floor AprilTags on the {args.grid_width}x{args.grid_height} path-planner grid -->'
    return '\n'.join([header] + blocks)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--grid-width', type=int, default=7)
    parser.add_argument('--grid-height', type=int, default=9)
    parser.add_argument('--resolution', type=float, default=1.0)
    parser.add_argument('--origin-x', type=float, default=-2.0)
    parser.add_argument('--origin-y', type=float, default=4.0)
    parser.add_argument('--tag-count', type=int, default=63, help='number of tag36h11-*.png textures available')
    parser.add_argument('--size', type=float, default=0.55, help='tag edge length in meters (incl. white margin)')
    parser.add_argument('--thickness', type=float, default=0.001, help='decal thickness in meters')
    parser.add_argument('--z-lift', type=float, default=0.0001, help='extra clearance above the ground plane to avoid z-fighting')
    parser.add_argument('--yaw', type=float, default=0.0, help='fixed yaw (radians) applied to every tag')
    parser.add_argument('--no-collision', action='store_true', help='omit collision geometry for the tags')
    parser.add_argument('--output', type=Path, default=Path('/tmp/apriltag_grid_snippet.xml'))
    return parser.parse_args()


def main():
    args = parse_args()
    snippet = build_snippet(args)
    args.output.write_text(snippet + '\n')
    print(f'Wrote {args.grid_width * args.grid_height} tag models to {args.output}')


if __name__ == '__main__':
    main()
