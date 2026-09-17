#!/usr/bin/env python3
"""Generate planner-only geometry; read AWSIM assets without modifying them.

Offline dependencies: numpy, scipy, Pillow, PyYAML, UnityPy (asset extraction).
The installed planner consumes only the generated PGM/YAML files.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial import ConvexHull, cKDTree
import yaml


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform_matrix(t):
    q = t.m_LocalRotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    scale, position = t.m_LocalScale, t.m_LocalPosition
    result = np.eye(4)
    result[:3, :3] = rotation @ np.diag([scale.x, scale.y, scale.z])
    result[:3, 3] = [position.x, position.y, position.z]
    return result


def extract_asset(asset, mgrs_offset):
    import UnityPy
    from UnityPy.helpers.MeshHelper import MeshHandler

    env = UnityPy.load(str(asset))
    transforms = {o.read().m_GameObject.path_id: o.read()
                  for o in env.objects if o.type.name == 'Transform'}

    def relative(t, stop_at_kart=False):
        matrix = np.eye(4)
        while t and not (stop_at_kart and t.m_GameObject.read().m_Name.startswith('GoKart')):
            matrix = transform_matrix(t) @ matrix
            t = t.m_Father.read() if t.m_Father.path_id else None
        return matrix, t.m_GameObject.read().m_Name if t else None

    def mesh_data(collider, matrix):
        handler = MeshHandler(collider.m_Mesh.read())
        handler.process()
        vertices = np.array(handler.m_Vertices)
        vertices = np.column_stack((vertices, np.ones(len(vertices)))) @ matrix.T
        ros_vertices = np.column_stack((vertices[:, 2], -vertices[:, 0], vertices[:, 1]))
        triangles = np.array([tri for sub in handler.get_triangles() for tri in sub], dtype=np.int32)
        return ros_vertices, triangles

    base = next(relative(t, True)[0] for t in transforms.values()
                if t.m_GameObject.read().m_Name == 'base_link'
                and relative(t, True)[1] == 'GoKart1')
    environment = body = None
    for obj in env.objects:
        if obj.type.name != 'MeshCollider':
            continue
        collider = obj.read()
        if collider.m_IsTrigger or not collider.m_Enabled:
            continue
        name = collider.m_GameObject.read().m_Name
        t = transforms[collider.m_GameObject.path_id]
        if name == 'citycircuit_marge_inner_10':
            vertices, triangles = mesh_data(collider, relative(t)[0])
            environment = vertices + np.asarray(mgrs_offset), triangles
        elif name == 'Collider':
            matrix, kart = relative(t, True)
            if kart == 'GoKart1':
                body, _ = mesh_data(collider, np.linalg.inv(base) @ matrix)
    if environment is None or body is None:
        raise ValueError('expected Odaiba environment and GoKart1 physical mesh colliders')
    return *environment, body


def cell_values(array, xy, origin, resolution, outside=0):
    ij = np.floor((xy-origin) / resolution).astype(int)
    inside = ((ij[:, 0] >= 0) & (ij[:, 0] < array.shape[1]) &
              (ij[:, 1] >= 0) & (ij[:, 1] < array.shape[0]))
    out = np.full(len(xy), outside, dtype=float)
    out[inside] = array[ij[inside, 1], ij[inside, 0]]
    return out


def clip_height(poly, lower, upper):
    for height, sign in [(lower, 1), (upper, -1)]:
        out = []
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            inside_a = sign * (a[2]-height) >= 0
            inside_b = sign * (b[2]-height) >= 0
            if inside_a:
                out.append(a)
            if inside_a != inside_b:
                out.append(a + (b-a) * (height-a[2]) / (b[2]-a[2]))
        poly = np.asarray(out)
        if len(poly) < 2:
            break
    return poly


def rasterize_polygon(occupied, polygon, origin, resolution):
    """Mark every closed grid cell intersected by a convex face projection."""
    p = (polygon[:, :2] - origin) / resolution
    low = np.maximum(np.floor(p.min(axis=0)-1e-9).astype(int), 0)
    high = np.minimum(np.floor(p.max(axis=0)+1e-9).astype(int),
                      [occupied.shape[1]-1, occupied.shape[0]-1])
    x, y = np.meshgrid(np.arange(low[0], high[0]+1), np.arange(low[1], high[1]+1))
    cells = np.column_stack((x.ravel()+.5, y.ravel()+.5))
    if not len(cells):
        return
    overlap = ((cells+.5 >= p.min(axis=0)-1e-9) &
               (cells-.5 <= p.max(axis=0)+1e-9)).all(axis=1)
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        edge = b-a
        axis = np.array([-edge[1], edge[0]])
        projection = p @ axis
        cp = cells @ axis
        radius = .5 * abs(axis).sum()
        overlap &= ((cp+radius >= projection.min()-1e-9) &
                    (cp-radius <= projection.max()+1e-9))
    occupied[y.ravel()[overlap], x.ravel()[overlap]] = True


def mesh_surfaces(vertices, triangles, blocked, origin, resolution):
    """Project physical walls and locally supported road surfaces.

    The course has varying elevation. Interior, near-horizontal road samples
    supply local ground height; a single global-z slice would include road.
    The old map selects interior height samples only. Its occupied boundary
    is not copied into the resulting physical wall geometry.
    """
    tri = vertices[triangles]
    centers = tri.mean(axis=1)
    normals = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
    norm = np.linalg.norm(normals, axis=1)
    cosz = abs(normals[:, 2]) / np.maximum(norm, 1e-20)
    interior_distance = distance_transform_edt(~blocked) * resolution
    seed = ((cosz > .98) &
            (cell_values(interior_distance, centers[:, :2], origin, resolution) > 1.0))
    points = centers[seed]
    if len(points) < 3:
        raise ValueError('insufficient interior road samples')
    keys = np.floor((points[:, :2]-origin)/.5).astype(int)
    order = np.argsort(points[:, 2], kind='stable')
    _, ids = np.unique(keys[order], axis=0, return_index=True)
    points = points[order[ids]]
    distance, ids = cKDTree(points[:, :2]).query(centers[:, :2], k=3)
    ground = np.median(points[ids, 2], axis=1)
    relative_z = tri[:, :, 2] - ground[:, None]
    selected = ((cosz < .5) & (relative_z.max(axis=1) > .15) &
                (relative_z.min(axis=1) < .75) & (distance[:, 0] < 4.0))
    added = np.zeros_like(blocked)
    for face, z in zip(tri[selected], ground[selected]):
        polygon = clip_height(face, z+.15, z+.75)
        if len(polygon) >= 2:
            rasterize_polygon(added, polygon, origin, resolution)
    road_faces = ((cosz > .98) & (abs(centers[:, 2]-ground) < .15) &
                  (distance[:, 0] < 4.0))
    road = np.zeros_like(blocked)
    for face in tri[road_faces]:
        rasterize_polygon(road, face, origin, resolution)
    details = dict(road_samples=len(points), selected_faces=int(selected.sum()),
                   selected_road_faces=int(road_faces.sum()), road_height_tolerance_m=.15,
                   added_free_cells=int((added & ~blocked).sum()),
                   road_normal_cos_z_min=.98, road_interior_distance_m=1.0,
                   road_sample_cell_m=.5, road_nearest_samples=3,
                   wall_normal_cos_z_max=.5, wall_height_above_road_m=[.15, .75],
                   maximum_road_sample_distance_m=4.0,
                   rasterization='closed-cell convex projection overlap')
    return added, road, points, details


def wall_cells(vertices, triangles, blocked, origin, resolution):
    walls, _, _, details = mesh_surfaces(vertices, triangles, blocked, origin, resolution)
    return walls, details


def compose_occupancy(walls, road, road_samples, origin, resolution, unknown_pixel=205):
    """Keep road components reached from observed interior samples.

    Unobserved ground and disconnected surfaces stay unknown. Walls always
    override road; no old-map margin is carried into the free-space boundary.
    Four-neighbour connectivity cannot jump diagonally across wall cells.
    """
    regions, count = label(road & ~walls)
    ij = np.floor((road_samples[:, :2]-origin)/resolution).astype(int)
    inside = ((ij[:, 0] >= 0) & (ij[:, 0] < road.shape[1]) &
              (ij[:, 1] >= 0) & (ij[:, 1] < road.shape[0]))
    ij = ij[inside]
    anchors = np.unique(regions[ij[:, 1], ij[:, 0]])
    anchors = anchors[anchors > 0]
    if not len(anchors):
        raise ValueError('no connected road component contains interior road samples')
    free = np.isin(regions, anchors) & (regions > 0)
    pixels = np.full(road.shape, unknown_pixel, dtype=np.uint8)
    pixels[free] = 254
    pixels[walls] = 0
    details = dict(road_components=int(count), connected_road_components=len(anchors),
                   free_cells=int(free.sum()), wall_cells=int(walls.sum()),
                   unknown_cells=int((~free & ~walls).sum()),
                   occupancy_source='physical walls and connected observed road; old occupancy not inherited')
    return pixels, free, details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--awsim-level', type=Path, required=True)
    parser.add_argument('--mgrs-offset', type=float, nargs=3, required=True,
                        help='Environment MGRS east, north, up offset for this AWSIM release')
    parser.add_argument('--base-map-yaml', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.resolve().is_relative_to(args.awsim_level.resolve().parent):
        parser.error('output must be outside AWSIM_Data')
    cfg = yaml.safe_load(args.base_map_yaml.read_text())
    if cfg['origin'][2] != 0 or cfg.get('negate', 0) != 0:
        parser.error('this asset generator expects an unrotated, non-negated base map')
    pgm = args.base_map_yaml.parent / cfg['image']
    pixels = np.array(Image.open(pgm))[::-1].copy()
    blocked = 1.0-pixels.astype(float)/255.0 >= cfg['free_thresh']
    origin = np.asarray(cfg['origin'][:2])
    resolution = float(np.float32(cfg['resolution']))
    vertices, triangles, body = extract_asset(args.awsim_level, args.mgrs_offset)
    walls, road, samples, details = mesh_surfaces(vertices, triangles, blocked, origin, resolution)
    # Choose a pixel strictly between the configured free/occupied thresholds.
    unknown_pixel = int(round(255 * (1-(cfg['free_thresh']+cfg['occupied_thresh'])/2)))
    pixels, free, domain_details = compose_occupancy(
        walls, road, samples, origin, resolution, unknown_pixel)
    details.update(domain_details, freed_old_blocked_cells=int((blocked & free).sum()),
                   removed_old_free_cells=int((~blocked & ~free).sum()), unknown_pixel=unknown_pixel)
    hull = body[ConvexHull(body[:, :2]).vertices, :2]
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels[::-1]).save(out/'occupancy_grid_map.pgm')
    cfg['image'] = 'occupancy_grid_map.pgm'
    (out/'occupancy_grid_map.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    footprint = ('# Generated physical MeshCollider XY hull about base_link. Do not hand edit.\n'
                 '/**:\n  ros__parameters:\n    brain.wall_footprint_xy_m: [\n')
    footprint += ''.join(f'      {x:.12f}, {y:.12f},\n' for x, y in hull)
    footprint += '    ]\n'
    (out/'footprint.param.yaml').write_text(footprint)
    metadata = dict(awsim_level_sha256=sha(args.awsim_level),
                    base_map_yaml_sha256=sha(args.base_map_yaml),
                    base_map_pgm_sha256=sha(pgm), mgrs_offset=args.mgrs_offset,
                    environment_mesh='citycircuit_marge_inner_10',
                    body_mesh='GoKart1/Colliders/Collider: Collision (non-trigger)',
                    body_anchor='base_link', body_xy_vertices=hull.tolist(),
                    body_xyz_min=body.min(axis=0).tolist(), body_xyz_max=body.max(axis=0).tolist(),
                    environment_triangles=len(triangles), **details,
                    output_pgm_sha256=sha(out/'occupancy_grid_map.pgm'),
                    output_footprint_sha256=sha(out/'footprint.param.yaml'))
    (out/'provenance.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps({k: v for k, v in metadata.items() if k != 'body_xy_vertices'}, indent=2))


if __name__ == '__main__':
    main()
