#!/usr/bin/env python3
"""Generate Gazebo Harmonic SDF maze approximating assignment Figure 1.
Grid is 7 columns x 6 rows, cell=0.5m. Grid y increases downwards.
Spawn at world (0.25, 0.0) faces +X in the open bottom corridor.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

CELL = 0.5
HEIGHT = 0.2
THICKNESS = 0.01
# (grid_x_start, grid_y_start, grid_x_end, grid_y_end)
# Outer bounds: x=[-0.5, 3.0], y=[-0.5, 2.5].
WALLS = [
    (0,0,7,0),  # top, including black reference segment
    (7,0,7,6),  # right
    (0,6,7,6),  # bottom
    (2,0,2,1),
    (4,0,4,1),
    (1,1,1,4),
    (3,1,3,2),
    (5,1,5,2),
    (2,2,5,2),
    (2,2,2,3),
    (4,2,4,3),
    (4,3,5,3),
    (5,3,5,4),
    (1,4,4,4),
    (3,3,3,4),
    (0,0,0,6),  # missing left boundary; append to preserve existing wall IDs
]

def xyz(gx, gy):
    return ((gx - 1) * CELL, (5 - gy) * CELL)

def box_model(world, name, x, y, z, sx, sy, sz, rgb):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x:.4f} {y:.4f} {z:.4f} 0 0 0'
    link = ET.SubElement(model, 'link', name='link')
    for kind in ('collision','visual'):
        obj = ET.SubElement(link, kind, name=kind)
        geometry = ET.SubElement(obj, 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = f'{sx:.4f} {sy:.4f} {sz:.4f}'
        if kind == 'visual':
            material = ET.SubElement(obj, 'material')
            ET.SubElement(material, 'ambient').text = rgb
            ET.SubElement(material, 'diffuse').text = rgb

root = ET.Element('sdf', version='1.9')
world = ET.SubElement(root, 'world', name='custom_maze')
physics = ET.SubElement(world, 'physics', name='default_physics', type='ignored')
ET.SubElement(physics, 'max_step_size').text = '0.001'
ET.SubElement(physics, 'real_time_factor').text = '1.0'
for filename, name in [
    ('gz-sim-physics-system','gz::sim::systems::Physics'),
    ('gz-sim-user-commands-system','gz::sim::systems::UserCommands'),
    ('gz-sim-scene-broadcaster-system','gz::sim::systems::SceneBroadcaster'),
    ('gz-sim-imu-system','gz::sim::systems::Imu'),
    ('gz-sim-sensors-system','gz::sim::systems::Sensors'),
]:
    plugin = ET.SubElement(world, 'plugin', filename=filename, name=name)
    if 'Sensors' in name:
        ET.SubElement(plugin, 'render_engine').text = 'ogre2'
ET.SubElement(world, 'gravity').text = '0 0 -9.8'
light = ET.SubElement(world, 'light', type='directional', name='sun')
ET.SubElement(light,'pose').text = '0 0 10 0 0 0'
ET.SubElement(light,'diffuse').text = '0.8 0.8 0.8 1'
ET.SubElement(light,'specular').text = '0.2 0.2 0.2 1'
ET.SubElement(light,'direction').text = '-0.5 0.2 -1'
ET.SubElement(light,'cast_shadows').text = 'true'
box_model(world, 'ground_plane', 1.25, 1.0, -0.05, 4.5, 4.0, 0.1, '0.72 0.72 0.72 1')
for i,(x1,y1,x2,y2) in enumerate(WALLS,1):
    ax,ay = xyz(x1,y1)
    bx,by = xyz(x2,y2)
    sx = abs(bx-ax) if x1!=x2 else THICKNESS
    sy = abs(by-ay) if y1!=y2 else THICKNESS
    box_model(world, f'wall_{i:02d}', (ax+bx)/2, (ay+by)/2, HEIGHT/2,
              sx,sy,HEIGHT,'0.85 0.47 0.04 1')
ET.indent(root, space='  ')
out = Path(__file__).parent / 'worlds' / 'custom_maze.sdf'
ET.ElementTree(root).write(out, encoding='unicode', xml_declaration=True)
print(f'Generated {out} with {len(WALLS)} wall segments, 3.5m x 3.0m grid')
print('Robot spawn: (0.25, 0.0, 0.01), yaw=0; bottom corridor')
