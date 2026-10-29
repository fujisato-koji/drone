import importlib.util
import tempfile
import unittest
from pathlib import Path

from sim import record


def _yaml_value(key):
    spec = importlib.util.spec_from_file_location("_tdr_t", record.ROOT / "tools" / "tdr.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    data = mod.load_yaml(record.VALUES_PATH.read_text(encoding="utf-8"))
    return next(d["v"] for d in data["items"] if d["key"] == key)


class TestRecord(unittest.TestCase):
    def test_value_matches_yaml(self):
        self.assertEqual(record.value("env.karman"), _yaml_value("env.karman"))
        self.assertEqual(record.value("env.charnock"), _yaml_value("env.charnock"))
        self.assertIsInstance(record.value("ds.min_u10_ms"), float)

    def test_value_missing(self):
        with self.assertRaises(KeyError):
            record.value("no.such.key")
        self.assertEqual(record.value("no.such.key", 2.5), 2.5)

    def test_values_prefix(self):
        d = record.values("env.")
        self.assertIn("env.karman", d)
        self.assertTrue(all(k.startswith("env.") for k in d))
        self.assertEqual(d["env.karman"], record.value("env.karman"))
        self.assertEqual(record.values("no.such."), {})

    def test_stamp_stable_across_line_endings(self):
        with tempfile.TemporaryDirectory() as td:
            a, b = Path(td) / "a.txt", Path(td) / "b.txt"
            a.write_bytes(b"one\ntwo\n")
            b.write_bytes(b"one\r\ntwo\r\n")
            s = record.stamp([a])
            self.assertTrue(s.startswith("sha-"))
            self.assertEqual(len(s), 4 + 12)
            # same path, other line endings: same stamp
            a.write_bytes(b"one\r\ntwo\r\n")
            self.assertEqual(record.stamp([a]), s)
            # different content: different stamp
            a.write_bytes(b"one\ntwo!\n")
            self.assertNotEqual(record.stamp([a]), s)
            # order matters
            a.write_bytes(b"x\n")
            b.write_bytes(b"y\n")
            self.assertNotEqual(record.stamp([a, b]), record.stamp([b, a]))

    def test_commit(self):
        c = record.commit()
        self.assertTrue(c == "unknown" or all(ch in "0123456789abcdef" for ch in c))

    def test_header(self):
        h = record.header("python -m x", [record.ROOT / "sim" / "record.py"])
        self.assertTrue(h.startswith("// tdr-run: python -m x commit "))
        self.assertIn(" stamp sha-", h)


if __name__ == "__main__":
    unittest.main()
