#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import Dict, Iterable, List

from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions
from rosidl_runtime_py.utilities import get_message

NSEC_PER_SEC = 1_000_000_000
TF_MESSAGE_TYPE = 'tf2_msgs/msg/TFMessage'


def _open_reader(uri: str, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=uri, storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    return reader


def _open_writer(uri: str, storage_id: str) -> SequentialWriter:
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=uri, storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    return writer


def _to_ns(sec: int, nanosec: int) -> int:
    return sec * NSEC_PER_SEC + nanosec


def _parse_topics(raw_topics: Iterable[str]) -> List[str]:
    topics: List[str] = []
    for item in raw_topics:
        for part in item.split(','):
            topic = part.strip()
            if topic:
                topics.append(topic)
    return topics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'rosbag2 の /tf 系トピックについて、Transform の header.stamp を '
            'bag storage timestamp にそろえて新しい bag を作成します。'
        )
    )
    parser.add_argument(
        '-i',
        '--input',
        required=True,
        help='入力 rosbag2 ディレクトリ（metadata.yaml があるディレクトリ）',
    )
    parser.add_argument(
        '-o',
        '--output',
        required=True,
        help='出力 rosbag2 ディレクトリ（未作成パスを指定）',
    )
    parser.add_argument(
        '--storage',
        default='mcap',
        help='storage id（既定: mcap）',
    )
    parser.add_argument(
        '--topics',
        nargs='*',
        default=['/tf', '/tf_static'],
        help='補正対象トピック（カンマ区切り可）。既定: /tf /tf_static',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    input_uri = str(Path(args.input).expanduser())
    output_uri = str(Path(args.output).expanduser())
    storage_id = args.storage
    target_topics = set(_parse_topics(args.topics))

    if not target_topics:
        raise RuntimeError('補正対象トピックが空です。--topics を指定してください。')

    in_path = Path(input_uri)
    out_path = Path(output_uri)
    if not in_path.exists():
        raise FileNotFoundError(f'入力ディレクトリが存在しません: {input_uri}')
    if out_path.exists():
        raise FileExistsError(f'出力ディレクトリが既に存在します: {output_uri}')

    reader = _open_reader(input_uri, storage_id)
    topics = reader.get_all_topics_and_types()
    type_map: Dict[str, str] = {topic.name: topic.type for topic in topics}

    unknown_topics = [topic for topic in target_topics if topic not in type_map]
    if unknown_topics:
        raise RuntimeError(
            f'入力 bag に存在しないトピックがあります: {unknown_topics} '
            f'(available={sorted(type_map.keys())})'
        )

    bad_types = [topic for topic in target_topics if type_map[topic] != TF_MESSAGE_TYPE]
    if bad_types:
        raise RuntimeError(
            f'補正対象に TFMessage 以外が含まれています: '
            f'{[(topic, type_map[topic]) for topic in bad_types]}'
        )

    msg_cls_map = {name: get_message(msg_type) for name, msg_type in type_map.items()}
    writer = _open_writer(output_uri, storage_id)
    for topic in topics:
        writer.create_topic(topic)

    target_msg_count = 0
    target_transform_count = 0
    first_old_ns = None
    first_new_ns = None
    last_old_ns = None
    last_new_ns = None

    while reader.has_next():
        topic_name, data, storage_ts_ns = reader.read_next()
        msg_cls = msg_cls_map[topic_name]

        if topic_name in target_topics:
            msg = deserialize_message(data, msg_cls)
            target_msg_count += 1
            for transform in msg.transforms:
                old_ns = _to_ns(transform.header.stamp.sec, transform.header.stamp.nanosec)
                new_ns = int(storage_ts_ns)

                if first_old_ns is None:
                    first_old_ns = old_ns
                    first_new_ns = new_ns
                last_old_ns = old_ns
                last_new_ns = new_ns

                transform.header.stamp.sec = int(new_ns // NSEC_PER_SEC)
                transform.header.stamp.nanosec = int(new_ns % NSEC_PER_SEC)
                target_transform_count += 1

            writer.write(topic_name, serialize_message(msg), storage_ts_ns)
        else:
            writer.write(topic_name, data, storage_ts_ns)

    print(f'[fix_tf_timestamps] input={input_uri}')
    print(f'[fix_tf_timestamps] output={output_uri}')
    print(f'[fix_tf_timestamps] patched topics={sorted(target_topics)}')
    print(
        f'[fix_tf_timestamps] patched messages={target_msg_count}, '
        f'patched transforms={target_transform_count}'
    )
    if first_old_ns is not None:
        print(
            '[fix_tf_timestamps] first stamp old->new: '
            f'{first_old_ns / 1e9:.9f} -> {first_new_ns / 1e9:.9f}'
        )
        print(
            '[fix_tf_timestamps] last  stamp old->new: '
            f'{last_old_ns / 1e9:.9f} -> {last_new_ns / 1e9:.9f}'
        )

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
