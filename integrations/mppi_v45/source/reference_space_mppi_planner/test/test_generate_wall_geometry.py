import importlib.util
from pathlib import Path
import unittest

import numpy as np
from PIL import Image
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('wall_geometry_generator', PACKAGE/'scripts/generate_awsim_wall_geometry.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class WallGeometryGeneration(unittest.TestCase):
    def test_only_observed_connected_road_becomes_free(self):
        road = np.ones((10, 12), dtype=bool)
        road[2, 2] = False
        walls = np.zeros_like(road)
        walls[:, 6] = True
        pixels, free, _ = generator.compose_occupancy(
            walls, road, np.array([[3.5, 4.5, 0.]]), np.zeros(2), 1.)
        self.assertTrue(free[4, 5])
        self.assertFalse(free[2, 2])
        self.assertTrue((pixels[:, 6] == 0).all())
        self.assertTrue((pixels[:, 7:] == 205).all())
        self.assertEqual(pixels[2, 2], 205)

    def test_road_connectivity_does_not_cross_diagonal_gap(self):
        road = np.eye(3, dtype=bool)
        pixels, free, _ = generator.compose_occupancy(
            np.zeros_like(road), road, np.array([[.5, .5, 0.]]), np.zeros(2), 1.)
        self.assertEqual(int(free.sum()), 1)
        self.assertEqual(pixels[1, 1], 205)

    def test_physical_road_replaces_old_inflated_margin(self):
        vertices, triangles = [], []
        for x in np.arange(.1, 7.9, .4):
            for y in np.arange(.1, 7.9, .4):
                i = len(vertices)
                vertices.extend([[x, y, 0.], [x+.4, y, 0.], [x+.4, y+.4, 0.], [x, y+.4, 0.]])
                triangles.extend([[i, i+1, i+2], [i, i+2, i+3]])
        i = len(vertices)
        vertices.extend([[6., -1., 0.], [6., 10., 0.], [6., 10., 1.], [6., -1., 1.]])
        triangles.extend([[i, i+1, i+2], [i, i+2, i+3]])
        old_blocked = np.ones((90, 90), bool)
        old_blocked[5:75, 5:50] = False
        walls, road, samples, _ = generator.mesh_surfaces(
            np.array(vertices), np.array(triangles), old_blocked, np.zeros(2), .1)
        pixels, free, _ = generator.compose_occupancy(walls, road, samples, np.zeros(2), .1)
        self.assertTrue(old_blocked[40, 55])
        self.assertTrue(free[40, 55])
        self.assertEqual(pixels[40, 59], 0)
        self.assertEqual(pixels[40, 60], 0)
        self.assertFalse(free[40, 61])

    def test_vertical_face_on_cell_edge_covers_both_sides(self):
        cells = np.zeros((8, 8), dtype=bool)
        face = np.array([[3., 2., .2], [3., 4., .2], [3., 4., .8]])
        generator.rasterize_polygon(cells, face, np.zeros(2), 1.)
        self.assertTrue(cells[2:5, 2:4].all())
        self.assertFalse(cells[:, :2].any())
        self.assertFalse(cells[:, 4:].any())

    def test_height_clipping_interpolates_sloping_faces(self):
        face = np.array([[0., 0., 0.], [1., 0., 1.], [0., 1., 1.]])
        clipped = generator.clip_height(face, .2, .7)
        self.assertGreaterEqual(clipped[:, 2].min(), .2-1e-12)
        self.assertLessEqual(clipped[:, 2].max(), .7+1e-12)
        self.assertTrue(np.allclose(clipped[:, :2].sum(axis=1), clipped[:, 2]))

    def test_sloping_road_is_not_projected_as_a_wall(self):
        vertices, triangles = [], []
        for x in np.arange(1.5, 8., .5):
            for y in np.arange(1.5, 8., .5):
                i = len(vertices)
                vertices.extend([[x, y, .05*x], [x+.3, y, .05*(x+.3)], [x, y+.3, .05*x]])
                triangles.append([i, i+1, i+2])
        blocked = np.zeros((100, 100), bool)
        blocked[[0, -1], :] = True
        blocked[:, [0, -1]] = True
        cells, _ = generator.wall_cells(np.array(vertices), np.array(triangles), blocked, np.zeros(2), .1)
        self.assertFalse(cells.any())

    def test_release_assets_are_internally_consistent(self):
        import json
        folder = PACKAGE/'config/awsim_wall_map'
        meta = json.loads((folder/'provenance.json').read_text())
        self.assertEqual(generator.sha(folder/'occupancy_grid_map.pgm'), meta['output_pgm_sha256'])
        self.assertEqual(generator.sha(folder/'footprint.param.yaml'), meta['output_footprint_sha256'])
        self.assertIn('old occupancy not inherited', meta['occupancy_source'])
        self.assertGreater(meta['freed_old_blocked_cells'], 0)
        self.assertGreater(meta['wall_cells'], 0)
        config = yaml.safe_load((folder/'footprint.param.yaml').read_text())
        hull = np.array(config['/**']['ros__parameters']['brain.wall_footprint_xy_m']).reshape(-1, 2)
        self.assertTrue(np.allclose(hull, meta['body_xy_vertices'], atol=1e-11, rtol=0))
        self.assertAlmostEqual(hull[:, 0].max(), 1.6148512754142978)
        self.assertAlmostEqual(hull[:, 0].min(), -.37850655262565724)
        with Image.open(folder/'occupancy_grid_map.pgm') as image:
            self.assertEqual(image.size, (751, 759))
            pixels = np.array(image)
            self.assertEqual(int((pixels == 254).sum()), meta['free_cells'])
            self.assertEqual(int((pixels == 0).sum()), meta['wall_cells'])
            self.assertEqual(int((pixels == meta['unknown_pixel']).sum()), meta['unknown_cells'])
        cfg = yaml.safe_load((folder/'occupancy_grid_map.yaml').read_text())
        unknown_occupancy = 1.-meta['unknown_pixel']/255.
        self.assertGreater(unknown_occupancy, cfg['free_thresh'])
        self.assertLess(unknown_occupancy, cfg['occupied_thresh'])


if __name__ == '__main__':
    unittest.main()
