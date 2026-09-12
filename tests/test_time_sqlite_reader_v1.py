from __future__ import annotations

import sqlite3
import numpy as np
from pathlib import Path
from types import SimpleNamespace

from aic_transfuser_lite.data.clock_segments import ClockSample, segment_clock_epochs
from aic_transfuser_lite.data.time_sqlite_reader_v1 import RawMessageRef, read_time_sqlite_run


def test_clock_epochs_cover_reset_without_loading_sensor_payloads() -> None:
    epochs = segment_clock_epochs(
        [ClockSample(100, 1000), ClockSample(200, 1100), ClockSample(300, 0), ClockSample(400, 100)],
        max_forward_jump_ns=1_000_000,
    )
    assert len(epochs) == 2
    assert epochs[1].reset_reason == "clock_reset_zero"


def test_sqlite_index_keeps_camera_as_raw_reference(tmp_path: Path, monkeypatch) -> None:
    bag = tmp_path / "run"
    bag.mkdir()
    (bag / "bag").mkdir()
    db = bag / "bag" / "rosbag2_0.db3"
    with sqlite3.connect(db) as conn:
        conn.executescript("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);")
        conn.executemany("INSERT INTO topics VALUES(?,?,?)", [(1, "/clock", "rosgraph_msgs/msg/Clock"), (2, "/sensing/camera/image_raw", "sensor_msgs/msg/Image")])
        conn.executemany("INSERT INTO messages VALUES(?,?,?,?)", [(1, 1, 100, b"clock"), (2, 1, 200, b"clock2"), (3, 2, 150, b"image")])

    class Store:
        def deserialize_cdr(self, raw, typ):
            if typ.endswith("Clock"):
                return SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=1000 if raw == b"clock" else 1100))
            if typ.endswith("Image"):
                return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=1)), height=2, width=3, encoding="rgb8", step=9, data=np.zeros(18, dtype=np.uint8))
            raise AssertionError("unexpected sensor deserialization")

    monkeypatch.setattr("aic_transfuser_lite.data.time_sqlite_reader_v1._store", lambda _: Store())
    index = read_time_sqlite_run(bag, "run")
    camera = next(e for e in index.events if e.role == "camera")
    assert isinstance(camera.payload, RawMessageRef)
    assert camera.payload.row_id == 3
    assert camera.capture_clock == "sim"
    assert camera.capture_ns == 1
    assert camera.payload.shape == (2, 3, 3)


def test_real_cdr_nested_sqlite_roundtrip_and_fallback_is_dropped(tmp_path: Path) -> None:
    from rosbags.typesys import Stores, get_typestore
    from aic_transfuser_lite.data.time_sqlite_reader_v1 import load_event
    store = get_typestore(Stores.ROS2_HUMBLE)
    typ = store.types
    stamp = typ["builtin_interfaces/msg/Time"]
    header = typ["std_msgs/msg/Header"]
    pixels = np.arange(18, dtype=np.uint8)
    image = typ["sensor_msgs/msg/Image"](header(stamp(1, 100), "camera"), 2, 3, "rgb8", 0, 9, pixels)
    missing_stamp = typ["sensor_msgs/msg/Image"](header(stamp(0, 0), "camera"), 2, 3, "rgb8", 0, 9, pixels)
    clock = typ["rosgraph_msgs/msg/Clock"](stamp(1, 0))
    end_clock = typ["rosgraph_msgs/msg/Clock"](stamp(2, 0))
    (tmp_path / "bag").mkdir()
    with sqlite3.connect(tmp_path / "bag" / "bag_0.db3") as conn:
        conn.executescript("CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);")
        conn.executemany("INSERT INTO topics VALUES(?,?,?)", [(1, "/clock", "rosgraph_msgs/msg/Clock"), (2, "/sensing/camera/image_raw", "sensor_msgs/msg/Image")])
        records = [(1, 1, 100_000_000_000, clock), (2, 2, 100_100_000_000, image),
                   (3, 2, 100_200_000_000, missing_stamp), (4, 1, 101_000_000_000, end_clock)]
        conn.executemany("INSERT INTO messages VALUES(?,?,?,?)", [(i, topic, bag, bytes(store.serialize_cdr(msg, msg.__msgtype__))) for i, topic, bag, msg in records])
    index = read_time_sqlite_run(tmp_path, "real")
    assert len(index.events) == 1 and index.fallback_counts == {"camera": 1}
    event = index.events[0]
    assert event.capture_ns == 1_000_000_100 and event.available_ns == 100_100_000_000
    assert event.capture_clock == "sim" and event.available_clock == "bag_receipt"
    assert isinstance(event.payload, RawMessageRef)
    decoded = load_event(tmp_path, event)
    np.testing.assert_array_equal(decoded.payload.image_rgb.reshape(-1), pixels)
    assert index.integrity == {"quick_check": "ok"}
