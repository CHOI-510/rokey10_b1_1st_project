from copy import deepcopy
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from box_database.database_store import SupabaseStore, make_stores
from box_database import database_config as cfg
from box_database.server_settings import server_settings


class SchemaStore(SupabaseStore):
    def __init__(self):
        super().__init__('https://test.example','sb_secret_test')
        self.inventory=[dict(loc_id=c,product_name='product '+c,cnt='0',standard='규격 '+c) for c in 'ABCDU']
        self.product=[dict(name='product '+c,len=d,weight=w) for c,d,w in [
            ('A','60,40,20','130'),('B','75,29,25',None),('C','50,40,20','108'),('D','50,40,20','55.3'),('U',None,None)]]
        self.calls=[]
        self.fail_b=False

    def _request(self,table,query,method='GET',patch=None):
        self.calls.append((table,method,deepcopy(query),deepcopy(patch)))
        if table=='product':return deepcopy(self.product)
        if table!='inventory':raise AssertionError('Unexpected table '+table)
        if method=='GET':return deepcopy(self.inventory)
        category=query['loc_id'].removeprefix('eq.')
        if category=='B' and self.fail_b:raise OSError('offline')
        row=next(r for r in self.inventory if r['loc_id']==category)
        row.update(patch)
        return [dict(loc_id=category,cnt=row['cnt'])]


class ExistingSupabaseSchemaTests(unittest.TestCase):
    def test_specs_use_inventory_product_mapping_and_len(self):
        store=SchemaStore();specs=store.products()
        self.assertEqual(len(specs),4)
        self.assertEqual(specs[0]['width_mm'],60)
        self.assertEqual(specs[1]['depth_mm'],29)
        self.assertEqual(specs[1]['weight_g'],0)
        self.assertEqual(specs[3]['weight_g'],55.3)

    def test_state_table_is_not_required_and_stock_is_read_from_db(self):
        store=SchemaStore();store.update_state({'stage':'SCANNING','job_id':'live-job'})
        store.inventory[0]['cnt']='4'
        state=store.get_state()
        self.assertEqual(state['a'],4)
        self.assertEqual(state['stage'],'SCANNING')
        self.assertEqual(state['job_id'],'live-job')
        self.assertFalse(any(call[0]=='state' for call in store.calls))

    def test_updates_only_cnt_and_keeps_other_inventory_columns(self):
        store=SchemaStore();before=deepcopy(store.inventory)
        store.update_state({'a':2,'u':1,'status':'RUNNING'})
        self.assertEqual(store.inventory[0]['cnt'],'2')
        self.assertEqual(store.inventory[-1]['cnt'],'1')
        self.assertEqual(store.inventory[0]['product_name'],before[0]['product_name'])
        self.assertEqual(store.get_state()['status'],'RUNNING')
        self.assertTrue(all(set(call[3])=={'cnt'} for call in store.calls if call[1]=='PATCH'))

    def test_retry_after_partial_failure_sets_absolute_counts(self):
        store=SchemaStore();store.fail_b=True
        with self.assertRaises(OSError):store.update_state({'a':2,'b':3})
        store.fail_b=False
        store.update_state({'a':2,'b':3})
        self.assertEqual(store.get_state()['a'],2)
        self.assertEqual(store.get_state()['b'],3)
        self.assertEqual(sum(call[1]=='PATCH' and call[2]['loc_id']=='eq.A' for call in store.calls),1)

    def test_invalid_or_missing_data_is_not_replaced_with_fake_stock(self):
        for count in [-1,7,True,1.5]:
            store=SchemaStore()
            with self.assertRaises(ValueError):store.update_state({'a':count})
            self.assertEqual(store.calls,[])
        store=SchemaStore();store.inventory.pop()
        with self.assertRaises(ValueError):store.get_state()
        store=SchemaStore();store.product[0]['len']='wrong'
        with self.assertRaises(ValueError):store.products()

    def test_service_and_background_writer_share_runtime_metadata(self):
        with patch.object(cfg,'SUPABASE_URL','https://test'),patch.object(cfg,'SUPABASE_KEY','sb_secret_test'):
            service,writer=make_stores('supabase','unused',None)
        self.assertIs(service,writer)

    def test_http_uses_secret_key_and_existing_table_schema(self):
        store=SupabaseStore('https://test.example','sb_secret_test')
        with patch('box_database.database_store.urlopen',return_value=io.BytesIO(b'[]')) as call:
            self.assertEqual(store._request('inventory',{'select':'loc_id,cnt'}),[])
        request=call.call_args.args[0]
        self.assertEqual(request.get_header('Apikey'),'sb_secret_test')
        self.assertIsNone(request.get_header('Authorization'))
        self.assertIn('/rest/v1/inventory?',request.full_url)

    def test_server_env_file_is_read_without_exporting_secrets(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'.env.local'
            path.write_text('NEXT_PUBLIC_SUPABASE_URL="https://db.example"\nSUPABASE_SERVICE_KEY="sb_secret_test"\nNEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY=browser\n')
            with patch.dict(os.environ,{'BOX_SERVER_ENV_FILE':str(path)},clear=True):
                self.assertEqual(server_settings(),('https://db.example','sb_secret_test'))
                self.assertNotIn('SUPABASE_SERVICE_KEY',os.environ)

    def test_regular_colcon_install_finds_workspace_server_config(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            module = workspace / 'install/box_database/lib/python3.12/site-packages/box_database/server_settings.py'
            config = workspace / 'src/project/my-next-app/.env.local'
            config.parent.mkdir(parents=True)
            config.write_text('NEXT_PUBLIC_SUPABASE_URL=https://db.example\nSUPABASE_SERVICE_KEY=sb_secret_test\n')
            with patch('box_database.server_settings.__file__', str(module)), patch.dict(os.environ, {}, clear=True):
                self.assertEqual(server_settings(), ('https://db.example', 'sb_secret_test'))
