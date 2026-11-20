import threading
import time
import unittest
from unittest.mock import patch

from box_database import database_work
from box_database.database_work import DatabaseWork


class MemoryStore:
    kind = 'file'

    def __init__(self):
        self.state = {'a': 0, 'b': 0, 'c': 0, 'd': 0, 'u': 0}
        self.updates = []

    def get_state(self):
        return dict(self.state)

    def update_state(self, patch):
        self.updates.append(dict(patch))
        self.state.update(patch)


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self):
        self.commands = []

    def request_job_command(self, command, source):
        self.commands.append((command, source))
        return True, 'accepted'


class DatabaseWorkInventoryTests(unittest.TestCase):
    def make_work(self):
        work = DatabaseWork.__new__(DatabaseWork)
        work.store = MemoryStore()
        work._pending = {}
        work._lock = threading.Lock()
        work._log_lock = threading.Lock()
        work._log_job = 'test-job'
        work._written = {}
        work._seen_wall = 'now'
        work._seen_mono = time.monotonic()
        work.stage = 'READY'
        work.log = FakeLogger()
        work.node = FakeNode()
        work.events = []
        work._append_log = lambda job, event: work.events.append((job, event))
        return work

    def test_inventory_update_persists_and_notifies_control(self):
        work = self.make_work()
        body = b'{"a":2,"b":1,"c":0,"d":3,"u":4,"source":"ui"}'

        with patch.object(database_work.cfg, 'START_TOKEN', ''):
            status, response = work.set_inventory(body, {})

        self.assertEqual(status, 200)
        self.assertTrue(response['ok'])
        self.assertEqual(work.store.state, {'a': 2, 'b': 1, 'c': 0, 'd': 3, 'u': 4,
                                            'updated_at': work.store.updates[0]['updated_at']})
        self.assertEqual(work.node.commands, [('inventory_changed', 'db')])
        self.assertEqual(work.events[0][1]['code'], 'INVENTORY_SET')

    def test_invalid_inventory_is_rejected_without_writing(self):
        for body in (
            b'{"a":true,"b":0,"c":0,"d":0,"u":0}',
            b'{"a":1.5,"b":0,"c":0,"d":0,"u":0}',
            b'{"a":-1,"b":0,"c":0,"d":0,"u":0}',
            b'{"a":0,"b":0,"c":0,"d":0}',
            b'{',
        ):
            with self.subTest(body=body):
                work = self.make_work()
                with patch.object(database_work.cfg, 'START_TOKEN', ''):
                    status, response = work.set_inventory(body, {})
                self.assertEqual(status, 400)
                self.assertFalse(response['ok'])
                self.assertEqual(work.store.updates, [])
                self.assertEqual(work.node.commands, [])

    def test_inventory_update_is_rejected_outside_ready(self):
        work = self.make_work()
        work.stage = 'RUNNING'

        with patch.object(database_work.cfg, 'START_TOKEN', ''):
            status, response = work.set_inventory(b'{"a":1,"b":0,"c":0,"d":0,"u":0}', {})

        self.assertEqual(status, 200)
        self.assertFalse(response['ok'])
        self.assertEqual(work.store.updates, [])

    def test_state_response_includes_latest_control_status(self):
        work = self.make_work()
        work._written = {'stage': 'SCANNING', 'job_id': 'job-2', 'status': 'RUNNING'}
        work._pending = {'box_id': 'job-2_001'}

        status, response = work.handle_get('/state')

        self.assertEqual(status, 200)
        self.assertEqual(response['stage'], 'READY')
        self.assertEqual(response['job_id'], 'job-2')
        self.assertEqual(response['box_id'], 'job-2_001')
        self.assertTrue(response['control_connected'])


if __name__ == '__main__':
    unittest.main()
