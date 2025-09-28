#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import Iterable, Set

# ROS 2 / rosbag2_py
import rosbag2_py

def open_reader(input_uri: str) -> rosbag2_py.SequentialReader:
    """
    Open a rosbag2 reader. storage_id="" lets rosbag2 read from metadata.yaml
    and pick the correct plugin (sqlite3 or mcap) automatically.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_uri, storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader

def topic_type_map(reader: rosbag2_py.SequentialReader):
    """
    Returns a dict: topic_name -> (type_name, serialization_format, qos_profiles)
    """
    mapping = {}
    for t in reader.get_all_topics_and_types():
        # t has: name, type, serialization_format, offered_qos_profiles (string)
        qos = getattr(t, "offered_qos_profiles", "") or ""
        ser = getattr(t, "serialization_format", "cdr") or "cdr"
        mapping[t.name] = (t.type, ser, qos)
    return mapping

def normalize_topics(topics: Iterable[str]) -> Set[str]:
    # Ensure no trailing spaces and standard leading slash
    norm = set()
    for tp in topics:
        tp = tp.strip()
        if not tp:
            continue
        if not tp.startswith("/"):
            tp = "/" + tp
        norm.add(tp)
    return norm

def main():
    parser = argparse.ArgumentParser(
        description="Filter a ROS 2 bag and write a new MCAP excluding specified topics."
    )
    parser.add_argument("input", help="Input bag path (folder containing metadata.yaml)")
    parser.add_argument("output", help="Output MCAP file path (e.g., output.mcap)")
    parser.add_argument(
        "--exclude",
        nargs="+",
        default=["/tf", "/tf_static"],
        help="Topic names to exclude (exact match). Default: /tf /tf_static",
    )
    args = parser.parse_args()

    in_uri = str(Path(args.input).resolve())
    out_path = Path(args.output).resolve()

    excl = normalize_topics(args.exclude)

    # --- Open reader (auto-detects storage via metadata) ---
    reader = open_reader(in_uri)
    tmap = topic_type_map(reader)

    # --- Prepare writer (always MCAP on output) ---
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(out_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # Create topics in writer for everything NOT excluded
    created = set()
    for name, (typ, ser, qos) in tmap.items():
        if name in excl:
            continue
        md = rosbag2_py.TopicMetadata(
            name=name,
            type=typ,
            serialization_format=ser or "cdr",
            offered_qos_profiles=qos or "",
        )
        writer.create_topic(md)
        created.add(name)

    # --- Fast pass-through copy (no deserialize) ---
    total_in = 0
    total_out = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        total_in += 1
        if topic in excl:
            continue
        if topic not in created:
            # In case the topic appeared after metadata (rare), create on the fly
            typ, ser, qos = tmap.get(topic, ("", "cdr", ""))
            if not typ:
                # Fallback: attempt to re-scan topics (some storage plugins update this)
                tmap = topic_type_map(reader)
                typ, ser, qos = tmap.get(topic, ("", "cdr", ""))
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    name=topic, type=typ, serialization_format=ser or "cdr",
                    offered_qos_profiles=qos or ""
                )
            )
            created.add(topic)
        writer.write(topic, data, timestamp)
        total_out += 1

    # Cleanup
    del reader
    del writer

    kept_topics = sorted(created)
    skipped_topics = sorted(excl.intersection(set(tmap.keys())))

    print("=== Filter complete ===")
    print(f"Input:  {in_uri}")
    print(f"Output: {out_path}")
    print(f"Messages read:  {total_in}")
    print(f"Messages written: {total_out}")
    print("Topics kept:")
    for t in kept_topics:
        print(f"  {t}  ({tmap[t][0]})")
    if skipped_topics:
        print("Topics excluded:")
        for t in skipped_topics:
            print(f"  {t}")

if __name__ == "__main__":
    main()
