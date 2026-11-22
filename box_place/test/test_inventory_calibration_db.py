from copy import deepcopy
import unittest
from unittest.mock import Mock, patch
from box_comm.common import PartError, default_poses
from box_place.inventory_check_algorithm import InventoryCheckAlgorithm, DEFAULT_CONFIG, validate


class DatabaseCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.algo = InventoryCheckAlgorithm.__new__(InventoryCheckAlgorithm)
        self.algo.c = validate(deepcopy(DEFAULT_CONFIG))
        self.algo.calibration_api = 'http://test'
        self.algo.calibration_source = 'supabase'
        self.algo.calibrate = False
        self.algo.state = {'version': 2, 'references': {}}
        self.poses = default_poses()
        self.sample = {'version': 2, 'references': {}}
        heights = {'A': [30.03948974609375,30.0197811126709],
                   'B': [30.568103790283203,30.59018783569336],
                   'C': [30.79688949584961,30.909153366088866],
                   'D': [30.683145141601564,30.914116668701173]}
        for category, xy in self.poses['place_zones'].items():
            self.sample['references'][category] = {}
            for tier in (1,2):
                pos = self.algo._slot_xy(category,tier,xy)
                self.sample['references'][category][str(tier)] = {
                    'signature': self.algo._signature(category,pos), 'empty_contact_tcp_z': heights[category][tier-1]}

    def test_all_eight_db_references_are_preserved(self):
        with patch('box_place.inventory_check_algorithm.read_calibration',return_value=self.sample):
            self.algo._load_db_calibration(self.poses)
        self.assertEqual(self.algo.state,self.sample)
        self.assertEqual(self.algo._reference('A',1,[480,-200]),30.03948974609375)

    def test_changed_orientation_blocks_before_any_measurement(self):
        self.sample['references']['D']['2']['signature']['orientation'] = [0,0,0]
        self.algo._measure_slot = Mock()
        with patch('box_place.inventory_check_algorithm.read_calibration',return_value=self.sample):
            with self.assertRaisesRegex(PartError,'DB 교정값 검증 실패'):
                self.algo.measure('A',self.poses,Mock())
        self.algo._measure_slot.assert_not_called()
        self.assertEqual(self.algo.state,{'version':2,'references':{}})

    def test_db_failure_never_falls_back_to_old_file(self):
        with patch('box_place.inventory_check_algorithm.read_calibration',side_effect=RuntimeError('offline')):
            with self.assertRaises(PartError): self.algo._load_db_calibration(self.poses)

    def test_current_goal_xy_must_match_calibration(self):
        self.poses['place_zones']['A'] = [500,-200]
        with patch('box_place.inventory_check_algorithm.read_calibration',return_value=self.sample):
            with self.assertRaises(PartError): self.algo._load_db_calibration(self.poses)
