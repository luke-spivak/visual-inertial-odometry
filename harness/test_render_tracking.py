import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import cv2
from render_tracking import inspect, render


class TrackingTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.prefix = Path(self.tmp.name) / 'run-test'
        self.stamps = [1234567890000000000 + i * 50000000 for i in range(5)]
        raw = np.zeros((5, 160, 320), dtype='<u2')
        raw[:, :, :] = 40 << 8
        raw.tofile(str(self.prefix) + '.y16')
        Path(str(self.prefix) + '.meta.json').write_text(json.dumps([
            {'SensorTimestamp': t} for t in self.stamps]))
        self.rows = [{'type': 'header', 'version': 1, 'coordinates': 'raw_pixels'}] + [
            {'type': 'frame', 'timestamp_ns': self.stamps[i], 'frame_index': i,
             'camera_id': 0, 'width': 320, 'height': 160, 'initialized': i > 0,
             'features': [[1, 70 + i * 10, 100]]} for i in [0, 1, 3, 4]]
        self.write()

    def write(self):
        Path(str(self.prefix) + '.features.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in self.rows))

    def test_exact_match(self):
        data = inspect(self.prefix)
        self.assertEqual(set(data[2]), {0, 1, 3, 4})
        self.rows[1]['timestamp_ns'] += 1
        self.write()
        with self.assertRaisesRegex(ValueError, 'mismatch'): inspect(self.prefix)

    def test_truncated_tail(self):
        with open(str(self.prefix) + '.features.jsonl', 'a') as f: f.write('{"type":')
        with open(str(self.prefix) + '.y16', 'ab') as f: f.write(b'\x00')
        self.assertTrue(any('Incomplete' in w for w in inspect(self.prefix)[5]))

    def test_invalid_coordinates(self):
        self.rows[1]['features'][0][1] = float('nan')
        self.write()
        with self.assertRaisesRegex(ValueError, 'coordinates'): inspect(self.prefix)

    def test_plain(self):
        Path(str(self.prefix) + '.features.jsonl').unlink()
        self.assertEqual(inspect(self.prefix, 320, 160)[2], {})

    def test_capture_gap_preserves_duration(self):
        for i in range(2, 5): self.stamps[i] += 500000000
        Path(str(self.prefix) + '.meta.json').write_text(json.dumps([
            {'SensorTimestamp': t} for t in self.stamps]))
        for row in self.rows[1:]: row['timestamp_ns'] = self.stamps[row['frame_index']]
        self.write()
        result = render(self.prefix, fps=20)
        self.assertAlmostEqual(result['duration_seconds'], .75)
        self.assertEqual(result['output_frames'], 15)
        video = cv2.VideoCapture(str(self.prefix) + '.tracking.mp4')
        video.set(cv2.CAP_PROP_POS_FRAMES, 5)
        ok, frame = video.read()
        video.release()
        self.assertTrue(ok)
        self.assertLess(frame[70:].max(), 5)  # capture gap is blank below label

    def test_video_and_idempotence(self):
        result = render(self.prefix, fps=20)
        self.assertEqual(result['frames_without_observations'], 1)
        path = Path(str(self.prefix) + '.tracking.mp4')
        before = path.stat().st_mtime_ns
        self.assertEqual(render(self.prefix, fps=20), result)
        self.assertEqual(path.stat().st_mtime_ns, before)
        video = cv2.VideoCapture(str(path))
        frames = []
        while True:
            ok, frame = video.read()
            if not ok: break
            frames.append(frame)
        video.release()
        self.assertEqual(len(frames), 5)
        # Missing record must have no dot or carried-over trail in the image area.
        self.assertLess(np.ptp(frames[2][80:120, 60:130].astype(float), axis=2).max(), 10)
        self.assertGreater(np.ptp(frames[1][95:105, 75:85].astype(float), axis=2).max(), 20)
        # Frame 3 starts a new trail after the missing record.
        self.assertLess(np.ptp(frames[3][95:105, 82:90].astype(float), axis=2).max(), 10)


if __name__ == '__main__': unittest.main()
