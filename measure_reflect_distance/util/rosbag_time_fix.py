#!/usr/bin/env python3
import argparse
import sys
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Set, Tuple

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    SequentialWriter,
    StorageOptions,
    TopicMetadata,
)
from rosidl_runtime_py.utilities import get_message

NSEC_PER_SEC = 1_000_000_000


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='rosbag2 の storage timestamp を補正し、必要に応じて header.stamp も再計算します。'
    )
    parser.add_argument(
        '-i',
        '--input',
        required=True,
        help='入力 rosbag2 ディレクトリ（metadata.yaml が入っているフォルダ）',
    )
    parser.add_argument(
        '-o',
        '--output',
        required=True,
        help='書き直し結果を保存するディレクトリ名（存在しないこと）',
    )
    parser.add_argument(
        '--storage',
        default='mcap',
        help='入力/出力 bag の storage_id（既定: mcap）',
    )
    parser.add_argument(
        '--reference',
        help='再生速度を合わせたい参照 rosbag2 ディレクトリ（オリジナルの bag など）',
    )
    parser.add_argument(
        '--reference-storage',
        help='参照 bag の storage_id（未指定なら --storage と同じ）',
    )
    parser.add_argument(
        '--sync-pair',
        action='append',
        default=[],
        metavar='PRIMARY:SECONDARY',
        help='SECONDARY トピックのタイムスタンプとヘッダを PRIMARY に合わせる（複数指定可）',
    )
    parser.add_argument(
        '--rewrite-header',
        action='store_true',
        help='storage timestamp に合わせて header.stamp も書き換える',
    )
    return parser.parse_args()


def open_reader(uri: str, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=uri, storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    return reader


def compute_storage_bounds(uri: str, storage_id: str) -> Tuple[Optional[int], Optional[int]]:
    reader = open_reader(uri, storage_id)
    min_t: Optional[int] = None
    max_t: Optional[int] = None
    while reader.has_next():
        _, _, t = reader.read_next()
        if min_t is None or t < min_t:
            min_t = t
        if max_t is None or t > max_t:
            max_t = t
    return min_t, max_t


@dataclass
class ReferenceEntry:
    storage_stamp: int
    header_stamp: Optional[int]


def load_reference_data(
    uri: str,
    storage_id: str,
) -> Tuple[
    Dict[str, List[ReferenceEntry]],
    Dict[str, Dict[int, Deque[ReferenceEntry]]],
    Optional[int],
    Optional[int],
]:
    reader = open_reader(uri, storage_id)
    topics = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics}
    entries: Dict[str, List[ReferenceEntry]] = defaultdict(list)
    header_lookup: Dict[str, Dict[int, Deque[ReferenceEntry]]] = defaultdict(dict)
    min_t: Optional[int] = None
    max_t: Optional[int] = None

    while reader.has_next():
        topic_name, data, t = reader.read_next()
        msg_cls = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_cls)
        if hasattr(msg, 'header') and msg.header.stamp:
            stamp_ns = msg.header.stamp.sec * NSEC_PER_SEC + msg.header.stamp.nanosec
            ref_entry = ReferenceEntry(storage_stamp=t, header_stamp=stamp_ns)
            entries[topic_name].append(ref_entry)
            topic_map = header_lookup[topic_name]
            if stamp_ns not in topic_map:
                topic_map[stamp_ns] = deque()
            topic_map[stamp_ns].append(ref_entry)
        else:
            entries[topic_name].append(ReferenceEntry(storage_stamp=t, header_stamp=None))
        if min_t is None or t < min_t:
            min_t = t
        if max_t is None or t > max_t:
            max_t = t

    return entries, header_lookup, min_t, max_t


def compute_header_bounds(
    uri: str,
    storage_id: str,
) -> Tuple[Optional[int], Optional[int], Dict[str, str], Iterable[TopicMetadata]]:
    reader = open_reader(uri, storage_id)
    topics = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topics}

    min_stamp: Optional[int] = None
    max_stamp: Optional[int] = None

    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        msg_cls = get_message(type_map[topic_name])
        msg = deserialize_message(data, msg_cls)

        if hasattr(msg, 'header') and msg.header.stamp:
            stamp_ns = msg.header.stamp.sec * NSEC_PER_SEC + msg.header.stamp.nanosec
            if min_stamp is None or stamp_ns < min_stamp:
                min_stamp = stamp_ns
            if max_stamp is None or stamp_ns > max_stamp:
                max_stamp = stamp_ns

    return min_stamp, max_stamp, type_map, topics


def main() -> int:
    args = parse_args()

    in_uri = str(Path(args.input).expanduser())
    out_uri = str(Path(args.output).expanduser())
    ref_uri = str(Path(args.reference).expanduser()) if args.reference else None
    storage_id = args.storage
    ref_storage_id = args.reference_storage or storage_id

    sync_pairs: Dict[str, str] = {}
    if args.sync_pair:
        for item in args.sync_pair:
            if ':' not in item:
                print(f'--sync-pair の形式が不正です: {item}', file=sys.stderr)
                return 1
            primary, secondary = item.split(':', 1)
            primary = primary.strip()
            secondary = secondary.strip()
            if not primary or not secondary:
                print(f'--sync-pair の形式が不正です: {item}', file=sys.stderr)
                return 1
            sync_pairs[secondary] = primary

    if not Path(in_uri).exists():
        print(f'入力ディレクトリが見つかりません: {in_uri}', file=sys.stderr)
        return 1

    if Path(out_uri).exists():
        print(f'出力ディレクトリが既に存在します: {out_uri}', file=sys.stderr)
        return 1

    if ref_uri and not Path(ref_uri).exists():
        print(f'参照ディレクトリが見つかりません: {ref_uri}', file=sys.stderr)
        return 1

    rclpy.init()

    header_min, header_max, type_map, topics = compute_header_bounds(in_uri, storage_id)

    if header_min is None or header_max is None:
        print('入力 bag に header.stamp を持つメッセージが見つかりませんでした。', file=sys.stderr)
        rclpy.shutdown()
        return 1

    reference_entries: Optional[Dict[str, List[ReferenceEntry]]] = None
    reference_header_lookup: Optional[Dict[str, Dict[int, Deque[ReferenceEntry]]]] = None
    ref_min: Optional[int] = None
    ref_max: Optional[int] = None
    if ref_uri:
        reference_entries, reference_header_lookup, ref_min, ref_max = load_reference_data(
            ref_uri, ref_storage_id
        )
        if ref_min is None or ref_max is None:
            print('参照 bag にメッセージが存在しませんでした。', file=sys.stderr)
            rclpy.shutdown()
            return 1
    else:
        ref_min = header_min
        ref_max = header_max

    header_duration = max(header_max - header_min, 1)
    ref_duration = max(ref_max - ref_min, 1)

    # スケール係数: header の差分を参照 bag の記録時間に合わせる（参照 bag が無い場合の既定動作）
    scale_num = ref_duration
    scale_den = header_duration

    reader = open_reader(in_uri, storage_id)

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=out_uri, storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    for topic in topics:
        writer.create_topic(topic)

    last_timestamp_ns = -1
    topic_indices: Dict[str, int] = defaultdict(int)
    exhausted_topics: Set[str] = set()

    try:
        while reader.has_next():
            topic_name, data, t = reader.read_next()
            msg_cls = get_message(type_map[topic_name])
            msg = deserialize_message(data, msg_cls)

            ref_sequence = reference_entries.get(topic_name) if reference_entries else None
            ref_idx = topic_indices[topic_name]

            header_stamp_ns: Optional[int] = None
            if hasattr(msg, 'header') and msg.header.stamp:
                header_stamp_ns = msg.header.stamp.sec * NSEC_PER_SEC + msg.header.stamp.nanosec

            new_t: Optional[int] = None
            header_override: Optional[int] = None

            # ヘッダ値が一致する参照エントリがあれば優先して利用
            if (
                header_stamp_ns is not None
                and reference_header_lookup
                and topic_name in reference_header_lookup
            ):
                topic_map = reference_header_lookup[topic_name]
                queue = topic_map.get(header_stamp_ns)
                if queue:
                    ref_entry = queue.popleft()
                    if not queue:
                        del topic_map[header_stamp_ns]
                    new_t = ref_entry.storage_stamp
                    header_override = ref_entry.header_stamp

            if new_t is None and ref_sequence:
                if ref_idx < len(ref_sequence):
                    ref_entry = ref_sequence[ref_idx]
                    new_t = ref_entry.storage_stamp
                    header_override = ref_entry.header_stamp
                elif topic_name not in exhausted_topics:
                    print(
                        f'警告: 参照 bag のメッセージ数が不足しているため {topic_name} のタイムスタンプをコピーできません。',
                        file=sys.stderr,
                    )
                    exhausted_topics.add(topic_name)
            # 同期対象になっている場合、PRIMARY の参照時刻で上書き
            primary_topic = sync_pairs.get(topic_name)
            if primary_topic and reference_entries:
                if (
                    header_stamp_ns is not None
                    and reference_header_lookup
                    and primary_topic in reference_header_lookup
                ):
                    primary_map = reference_header_lookup[primary_topic]
                    queue = primary_map.get(header_stamp_ns)
                    if queue:
                        ref_entry = queue[0]  # don't pop to allow reuse for primary topic itself
                        new_t = ref_entry.storage_stamp
                        header_override = ref_entry.header_stamp
                if new_t is None:
                    primary_seq = reference_entries.get(primary_topic)
                    if primary_seq and ref_idx < len(primary_seq):
                        ref_entry = primary_seq[ref_idx]
                        new_t = ref_entry.storage_stamp
                        header_override = ref_entry.header_stamp
                    elif topic_name not in exhausted_topics:
                        print(
                            f'警告: {primary_topic} と {topic_name} のメッセージ数が一致しません（インデックス {ref_idx}）。',
                            file=sys.stderr,
                        )
                        exhausted_topics.add(topic_name)
                    ref_entry = primary_seq[ref_idx]
                    new_t = ref_entry.storage_stamp
                    header_override = ref_entry.header_stamp
                elif topic_name not in exhausted_topics:
                    print(
                        f'警告: {primary_topic} と {topic_name} のメッセージ数が一致しません（インデックス {ref_idx}）。',
                        file=sys.stderr,
                    )
                    exhausted_topics.add(topic_name)

            if new_t is None and header_stamp_ns is not None:
                original_stamp = header_stamp_ns
                if header_max == header_min:
                    new_t = ref_min
                elif original_stamp == header_max:
                    new_t = ref_max
                else:
                    delta = original_stamp - header_min
                    new_t = ref_min + (delta * scale_num) // scale_den

            if new_t is None:
                # ヘッダが無い場合や参照が取れない場合は元の記録時刻を使用
                new_t = t

            if args.rewrite_header and hasattr(msg, 'header'):
                if header_override is not None:
                    header_value = header_override
                elif header_stamp_ns is not None:
                    header_value = header_stamp_ns
                else:
                    header_value = new_t
                msg.header.stamp.sec = int(header_value // NSEC_PER_SEC)
                msg.header.stamp.nanosec = int(header_value % NSEC_PER_SEC)

            writer.write(topic_name, serialize_message(msg), new_t)
            last_timestamp_ns = max(last_timestamp_ns, new_t)
            topic_indices[topic_name] = ref_idx + 1
    finally:
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
