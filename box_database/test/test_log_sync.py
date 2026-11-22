import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from box_database.log_sync import LogSync, event_row


class LogSyncTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'job.jsonl'
        self.event = dict(job_id='job', seq=1, source='control', message='작업 시작',
                          received_at='2026-11-20T08:34:02+00:00')
        self.sync = LogSync(self.tmp.name, 'https://example.com', 'unused', Mock())
        self.sync.upload = Mock()

    def test_retries_from_same_position_after_failure(self):
        self.path.write_text(json.dumps(self.event) + '\n')
        self.sync.upload.side_effect = OSError('offline')
        with self.assertRaises(OSError):
            self.sync.sync_file(self.path)
        self.assertEqual(self.sync.offsets, {})
        self.sync.upload.side_effect = None
        self.assertEqual(self.sync.sync_file(self.path), 1)
        self.assertEqual(self.sync.sync_file(self.path), 0)

    def test_partial_append_waits_for_complete_line(self):
        line = json.dumps(self.event)
        self.path.write_text(line[:20])
        self.assertEqual(self.sync.sync_file(self.path), 0)
        with self.path.open('a') as stream:
            stream.write(line[20:] + '\n')
        self.assertEqual(self.sync.sync_file(self.path), 1)

    def test_row_has_requested_schema_and_original_timestamp(self):
        row = event_row(self.event)
        self.assertEqual(set(row), {'timestamp', 'job_id', 'message', 'state'})
        self.assertEqual(row['timestamp'], self.event['received_at'])
        self.assertEqual(row, event_row(dict(self.event)))

    def test_keyword_filter_and_controller_state(self):
        self.assertIsNone(event_row(dict(self.event, message='robot_setup joint=1')))
        row = event_row(dict(self.event, message='물품 판별 완료', control_stage='JUDGE_DONE'))
        self.assertEqual(row['state'], '판별완료')
        historical = event_row(dict(self.event, message='탐지 성공', source='scan'))
        self.assertEqual(historical['state'], '스캔중')

    def test_filtered_lines_advance_without_upload(self):
        self.path.write_text(json.dumps(dict(self.event, message='debug xyz')) + '\n')
        self.assertEqual(self.sync.sync_file(self.path), 1)
        self.sync.upload.assert_not_called()
        self.assertEqual(self.sync.sync_file(self.path), 0)

    def test_events_appended_after_initial_upload_are_sent(self):
        self.path.write_text(json.dumps(self.event) + '\n')
        self.sync.sync_file(self.path)
        with self.path.open('a') as stream:
            stream.write(json.dumps(dict(self.event, seq=2)) + '\n')
        self.sync.sync_file(self.path)
        self.assertEqual(self.sync.upload.call_count, 2)
        self.assertEqual(set(self.sync.upload.call_args.args[0][0]), {'timestamp', 'job_id', 'message', 'state'})


if __name__ == '__main__':
    unittest.main()

class LiveFolderTests(unittest.TestCase):
    def test_bad_line_does_not_block_new_events(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '20261122_114422.jsonl'
            event = dict(job_id='20261122_114422', message='A 적재 완료 → A 3개',
                         received_at='2026-11-22T02:46:11.559915+00:00')
            path.write_text('{bad json}\n' + json.dumps(event) + '\n')
            sync = LogSync(directory, 'https://example.com', 'unused', Mock())
            sync.upload = Mock()
            self.assertEqual(sync.sync_file(path), 2)
            self.assertEqual(sync.upload.call_args.args[0][0]['message'], event['message'])

    def test_replaced_file_with_same_name_is_read_from_start(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '20261122_job.jsonl'
            event = dict(job_id='job', message='작업 시작', received_at='2026-11-22T00:00:00Z')
            path.write_text(json.dumps(event) + '\n')
            sync = LogSync(directory, 'https://example.com', 'unused', Mock())
            sync.upload = Mock()
            sync.sync_file(path)
            replacement = Path(directory) / 'replacement'
            replacement.write_text(json.dumps(dict(event, message='작업 완료')) + '\n')
            replacement.replace(path)
            self.assertEqual(sync.sync_file(path), 1)
            self.assertEqual(sync.upload.call_count, 2)
