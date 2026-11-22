import unittest
from box_database.database_store import _location_from_row


class LocationCalibrationTests(unittest.TestCase):
    def test_legacy_angles_never_become_xy(self):
        row={'id':'A','coords':'-150,180,120','inventory_calibration':{'1':{'signature':{'xy':[480,-200]}}}}
        self.assertEqual(_location_from_row(row),{'name':'A','x':480.0,'y':-200.0,'z':None})

    def test_missing_calibration_does_not_fall_back_to_angles(self):
        with self.assertRaises(ValueError):_location_from_row({'id':'B','coords':'-150,-180,120'})

    def test_korean_measure_zone_name_is_normalized(self):
        self.assertEqual(_location_from_row({'id':'판별존','coords':'300,-160,30.8'}),
                         {'name':'measure_zone','x':300.0,'y':-160.0,'z':30.8})
