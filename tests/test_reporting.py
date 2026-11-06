import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from amr_research_lab.reporting import write_report
from amr_research_lab.simulation import Simulation


class ReportingTests(unittest.TestCase):
    def test_reports_are_valid_and_reproducible(self):
        sim = Simulation('open_floor').run()
        with tempfile.TemporaryDirectory() as first, tempfile.TemporaryDirectory() as second:
            write_report(sim, first)
            write_report(Simulation('open_floor').run(), second)
            files = list(Path(first).iterdir())
            self.assertEqual({p.suffix for p in files}, {'.csv', '.json', '.svg'})
            for path in files:
                self.assertEqual(path.read_bytes(), (Path(second) / path.name).read_bytes())
            metrics = json.loads(next(Path(first).glob('*.json')).read_text())
            self.assertEqual(metrics['status'], 'completed')
            with next(Path(first).glob('*.csv')).open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), len(sim.trace))
            self.assertAlmostEqual(float(rows[-1]['x']), sim.pose.x)
            root = ET.parse(next(Path(first).glob('*.svg'))).getroot()
            self.assertEqual(root.tag, '{http://www.w3.org/2000/svg}svg')
            self.assertGreater(len(root.findall('.//{http://www.w3.org/2000/svg}polyline')), 1)

    def test_cli_generates_a_completed_experiment(self):
        with tempfile.TemporaryDirectory() as output:
            result = subprocess.run([sys.executable, '-m', 'amr_research_lab', 'run',
                                     '--scenario', 'open_floor', '--output', output],
                                    capture_output=True, text=True, timeout=20)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)['status'], 'completed')


if __name__ == '__main__':
    unittest.main()
