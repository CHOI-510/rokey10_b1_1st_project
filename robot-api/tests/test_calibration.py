import io
import json
import os
import unittest
from unittest.mock import patch
from fastapi.testclient import TestClient
from app.main import app


class CalibrationApiTests(unittest.TestCase):
    def test_server_fetches_all_zone_records_without_modifying_them(self):
        rows=[{'id':c,'inventory_calibration':{'1':{'empty_contact_tcp_z':30.1,'signature':{'probe':'closed_empty_gripper'}},'2':{'empty_contact_tcp_z':30.2,'signature':{}}}} for c in 'ABCD']
        with patch.dict(os.environ,{'SUPABASE_URL':'https://db.example','SUPABASE_SERVICE_KEY':'sb_secret_test'}), patch('app.calibration.urlopen',return_value=io.BytesIO(json.dumps(rows).encode())) as request:
            result=TestClient(app).get('/robot/inventory/calibration')
        self.assertEqual(result.status_code,200)
        self.assertEqual(result.json()['references']['A'],rows[0]['inventory_calibration'])
        self.assertEqual(request.call_args.args[0].get_method(),'GET')
        self.assertEqual(request.call_args.args[0].get_header('Apikey'),'sb_secret_test')
        self.assertIsNone(request.call_args.args[0].get_header('Authorization'))

    def test_missing_zone_is_not_a_valid_calibration(self):
        with patch.dict(os.environ,{'SUPABASE_URL':'https://db.example','SUPABASE_SERVICE_KEY':'test'}), patch('app.calibration.urlopen',return_value=io.BytesIO(b'[]')):
            self.assertEqual(TestClient(app).get('/robot/inventory/calibration').status_code,503)

    def test_browser_publishable_key_cannot_replace_service_key(self):
        with patch.dict(os.environ,{'SUPABASE_URL':'https://db.example','SUPABASE_SERVICE_KEY':'','NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY':'browser'}), patch('app.calibration.urlopen') as request:
            self.assertEqual(TestClient(app).get('/robot/inventory/calibration').status_code,503)
            request.assert_not_called()
