"""Noise semantics, actual collision heights, and unchanged plank layout."""
from dataclasses import replace
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from g1_29dof.perlin_noise import generate_fractal_noise_2d
from g1_29dof.perlin_surface import PerlinSurface, PerlinSurfaceCfg
from g1_29dof.rough_terrain_generator import RoughTerrainGenerator, SceneCfg, WoodenPlanksCfg
from g1_23dof import config
from g1_23dof.rough_terrain_cfg import DEFAULT_GROUND_Z, PLANK_SETTINGS, PERLIN_SETTINGS


class PerlinNoiseTests(unittest.TestCase):
    def test_samples_match_instinctlab_reference(self):
        # Values generated using upstream perlin.py (seed 23) before adaptation.
        values = generate_fractal_noise_2d(
            xSize=.6, ySize=.9, xSamples=25, ySamples=37, frequency=20,
            fractalOctaves=2, fractalLacunarity=2., fractalGain=.25,
            zScale=.025, centering=True, rng=np.random.RandomState(23),
        )
        actual = [values[i,j] for i,j in [(0,0),(1,1),(3,5),(11,20),(24,36)]]
        np.testing.assert_allclose(actual, [
            1.1963590109177052e-05, -.00214766625422335, -.0021899180141745165,
            -.0035731611980826163, 1.1963590109177052e-05,
        ], rtol=1e-10, atol=1e-12)

    def test_difficulty_scales_noise_and_quantizes_to_millimeters(self):
        cfg = PerlinSurfaceCfg()
        self.assertAlmostEqual(replace(cfg, difficulty=0).amplitude(), .01)
        self.assertAlmostEqual(cfg.amplitude(), .025)
        self.assertAlmostEqual(replace(cfg, difficulty=1).amplitude(), .04)
        low = PerlinSurface(replace(cfg, difficulty=0), 0,.6,0,.9)
        high = PerlinSurface(replace(cfg, difficulty=1), 0,.6,0,.9)
        self.assertGreater(np.std(high.heights), 3*np.std(low.heights))
        np.testing.assert_allclose(high.heights/.001, np.round(high.heights/.001), atol=1e-10)
        self.assertLess(high.heights.min(), 0)
        self.assertGreater(high.heights.max(), 0)

    def test_seed_reproducibility_and_local_rng(self):
        np.random.seed(77)
        before = np.random.get_state()
        a = PerlinSurface(PerlinSurfaceCfg(),0,.6,0,.9)
        b = PerlinSurface(PerlinSurfaceCfg(),0,.6,0,.9)
        c = PerlinSurface(PerlinSurfaceCfg(seed=24),0,.6,0,.9)
        np.testing.assert_array_equal(a.heights,b.heights)
        self.assertFalse(np.array_equal(a.heights,c.heights))
        after = np.random.get_state()
        np.testing.assert_array_equal(before[1],after[1])
        self.assertEqual(before[2:],after[2:])

    def test_absolute_height_limit_holds_across_seeds_and_difficulties(self):
        for seed in (23,24):
            for difficulty in (0.,.5,1.):
                cfg = PerlinSurfaceCfg(seed=seed,difficulty=difficulty,height_limit=.005)
                with self.subTest(seed=seed,difficulty=difficulty):
                    surface = PerlinSurface(cfg,0,.6,0,.9)
                    self.assertLessEqual(np.max(np.abs(surface.heights)),.005)
                    np.testing.assert_allclose(surface.heights/.001,np.round(surface.heights/.001),atol=1e-10)
        zero = PerlinSurface(PerlinSurfaceCfg(height_limit=0),0,.6,0,.9)
        self.assertFalse(zero.heights.any())
        subgrid = PerlinSurface(PerlinSurfaceCfg(height_limit=.0045),0,.6,0,.9)
        self.assertLessEqual(np.max(np.abs(subgrid.heights)),.004)

    def test_invalid_parameters_fail_clearly(self):
        for changes in ({"difficulty":1.1},{"noise_scale":[.04,.01]}, {"horizontal_scale":0}, {"vertical_scale":0}, {"fractal_octaves":0}, {"seed":-1}, {"height_limit":-1}, {"height_limit":float('nan')}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                PerlinSurface(replace(PerlinSurfaceCfg(),**changes),0,.6,0,.9)


class PerlinCollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Exercise rough geometry even when the user disables it for playback.
        cls.planks = WoodenPlanksCfg(**{**PLANK_SETTINGS, 'perlin_cfg':PERLIN_SETTINGS})
        scene = SceneCfg(terrains=(cls.planks,),ground_z=DEFAULT_GROUND_Z)
        cls.generator = RoughTerrainGenerator(scene, template_scene=config.HERE/'scene_flat.xml')
        cls.generator.add_all()
        # Use actual terrain geoms but omit the robot; retain XML-relative mesh
        # independence and make it easy to add a small test contact sphere.
        root = ET.fromstring(ET.tostring(cls.generator.root))
        root.remove(root.find('include'))
        cls.terrain_xml = ET.tostring(root, encoding='unicode')
        cls.model = mujoco.MjModel.from_xml_string(cls.terrain_xml)
        cls.data = mujoco.MjData(cls.model)
        mujoco.mj_forward(cls.model,cls.data)

    def height_data(self, name):
        model=self.model
        geom=model.geom(name)
        hid=model.geom_dataid[geom.id]
        nr,nc=model.hfield_nrow[hid],model.hfield_ncol[hid]
        start=model.hfield_adr[hid]
        return geom,hid,model.hfield_data[start:start+nr*nc].reshape(nr,nc)*model.hfield_size[hid,2]+geom.pos[2]

    def test_compiled_noise_matches_global_field_on_platform_and_floor(self):
        cfg = PerlinSurfaceCfg(**PERLIN_SETTINGS)
        p=self.planks
        end=(p.num_planks*p.plank_width + (p.num_planks-1)*p.min_gap)+p.landing_length
        surface=PerlinSurface(cfg,-p.approach_length-cfg.ground_padding,end+cfg.ground_padding,
                              -p.plank_length/2-cfg.ground_padding,p.plank_length/2+cfg.ground_padding)
        _,_,ground=self.height_data('wooden_planks_ground')
        np.testing.assert_allclose(ground, DEFAULT_GROUND_Z+surface.heights,atol=1e-8)
        for i in range(p.num_planks):
            name=f'wooden_planks_board_{i:02d}'
            _,_,heights=self.height_data(name)
            x=i*(p.plank_width+p.min_gap)
            expected=surface.patch(x,x+p.plank_width,-p.plank_length/2,p.plank_length/2)
            np.testing.assert_allclose(heights,p.start[2]+expected,atol=1e-8)
            self.assertGreater(np.ptp(heights),.001)
            limit=cfg.height_limit
            if limit is not None:
                self.assertGreaterEqual(heights.min(),p.start[2]-limit-1e-8)
                self.assertLessEqual(heights.max(),p.start[2]+limit+1e-8)
        if cfg.height_limit is not None:
            self.assertGreaterEqual(ground.min(),DEFAULT_GROUND_Z-cfg.height_limit-1e-8)
            self.assertLessEqual(ground.max(),DEFAULT_GROUND_Z+cfg.height_limit+1e-8)

    def test_rays_and_physical_contact_follow_troughs(self):
        # Check a negative trough on a board and a negative trough in a gap.
        groups=np.zeros(6,dtype=np.uint8); groups[2]=1
        for name, gap_only in [('wooden_planks_board_00',False),('wooden_planks_ground',True)]:
            geom,hid,heights=self.height_data(name)
            nr,nc=heights.shape
            sx,sy=self.model.hfield_size[hid,:2]
            xs=np.linspace(geom.pos[0]-sx,geom.pos[0]+sx,nc)
            ys=np.linspace(geom.pos[1]-sy,geom.pos[1]+sy,nr)
            valid=np.ones_like(heights,dtype=bool)
            valid[[0,-1],:]=False;valid[:,[0,-1]]=False
            if gap_only:
                # Stay within the middle half of the first configured gap.
                gap_start=self.planks.start[0]+self.planks.plank_width
                valid &= (xs[None,:]>gap_start+.25*self.planks.min_gap)&(xs[None,:]<gap_start+.75*self.planks.min_gap)
            sample=np.where(valid,heights,np.inf)
            row,col=np.unravel_index(np.argmin(sample),sample.shape)
            z=heights[row,col];x=xs[col];y=ys[row]
            self.assertLess(z, DEFAULT_GROUND_Z if gap_only else 0)
            hit=np.array([-1],dtype=np.int32)
            distance=mujoco.mj_ray(self.model,self.data,np.array([x,y,1.]),np.array([0.,0.,-1.]),groups,1,-1,hit)
            self.assertEqual(hit[0],geom.id)
            self.assertAlmostEqual(1-distance,z,places=6)
            root=ET.fromstring(self.terrain_xml)
            sphere=ET.SubElement(root.find('worldbody'),'body',name='probe',pos=f'{x} {y} {z+.005-.0001}')
            ET.SubElement(sphere,'freejoint')
            ET.SubElement(sphere,'geom',name='probe',type='sphere',size='.005',mass='.01')
            model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
            data=mujoco.MjData(model);mujoco.mj_forward(model,data)
            probe=model.geom('probe').id; target=model.geom(name).id
            contacts=[c for c in data.contact if {c.geom1,c.geom2}=={probe,target}]
            self.assertTrue(contacts, name)
            self.assertLess(min(c.dist for c in contacts),0)

    def test_flat_mode_remains_available_and_zero_amplitude_is_valid(self):
        for perlin in (None, {'noise_scale':0.0}):
            p=replace(self.planks,perlin_cfg=perlin)
            gen=RoughTerrainGenerator(SceneCfg(terrains=(p,),ground_z=DEFAULT_GROUND_Z),template_scene=config.HERE/'scene_flat.xml')
            gen.add_all()
            boards=[g for g in gen.root.findall('worldbody/geom') if g.get('name','').startswith('wooden_planks_board_')]
            self.assertEqual(len(boards),16)
            self.assertTrue(all(g.get('type')=='box' for g in boards))


if __name__ == '__main__':
    unittest.main()
