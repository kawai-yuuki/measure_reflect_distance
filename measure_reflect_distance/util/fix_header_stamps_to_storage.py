#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple

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


def _parse_topics(raw_topics: Iterable[str]) -> List[str]:
    topics: List[str] = []
    for item in raw_topics:
        for part in item.split(','):
            topic = part.strip()
            if topic:
                topics.append(topic)
    return topics


def _set_msg_header_stamp_to_ns(msg, stamp_ns: int) -> bool:
    if not hasattr(msg, 'header'):
        return False
    header = getattr(msg, 'header')
    if not hasattr(header, 'stamp'):
        return False
    header.stamp.sec = int(stamp_ns // NSEC_PER_SEC)
    header.stamp.nanosec = int(stamp_ns % NSEC_PER_SEC)
    return True


def _set_tf_stamps_to_ns(msg, stamp_ns: int) -> int:
    count = 0
    for transform in msg.transforms:
        transform.header.stamp.sec = int(stamp_ns // NSEC_PER_SEC)
        transform.header.stamp.nanosec = int(stamp_ns % NSEC_PER_SEC)
        count += 1
    return count


def _get_msg_header_stamp_ns(msg) -> Optional[int]:
    if not hasattr(msg, 'header'):
        return None
    header = getattr(msg, 'header')
    if not hasattr(header, 'stamp'):
        return None
    return int(header.stamp.sec) * NSEC_PER_SEC + int(header.stamp.nanosec)


def _compute_bounds(
    input_uri: str,
    storage_id: str,
    target_topics: Set[str],
    bounds_topics: Set[str],
    type_map: Dict[str, str],
    msg_cls_map: Dict[str, object],
) -> Tuple[Optional[int], Optional[int], Optional[int], Optional[int]]:
    reader = _open_reader(input_uri, storage_id)

    storage_min_ns: Optional[int] = None
    storage_max_ns: Optional[int] = None
    header_min_ns: Optional[int] = None
    header_max_ns: Optional[int] = None

    while reader.has_next():
        topic_name, data, storage_ts_ns = reader.read_next()
        storage_ts_ns = int(storage_ts_ns)
        if storage_min_ns is None or storage_ts_ns < storage_min_ns:
            storage_min_ns = storage_ts_ns
        if storage_max_ns is None or storage_ts_ns > storage_max_ns:
            storage_max_ns = storage_ts_ns

        if topic_name not in bounds_topics:
            continue

        msg_cls = msg_cls_map[topic_name]
        msg = deserialize_message(data, msg_cls)

        if type_map[topic_name] == TF_MESSAGE_TYPE:
            for transform in msg.transforms:
                h_ns = int(transform.header.stamp.sec) * NSEC_PER_SEC + int(
                    transform.header.stamp.nanosec
                )
                if header_min_ns is None or h_ns < header_min_ns:
                    header_min_ns = h_ns
                if header_max_ns is None or h_ns > header_max_ns:
                    header_max_ns = h_ns
        else:
            h_ns = _get_msg_header_stamp_ns(msg)
            if h_ns is None:
                continue
            if header_min_ns is None or h_ns < header_min_ns:
                header_min_ns = h_ns
            if header_max_ns is None or h_ns > header_max_ns:
                header_max_ns = h_ns

    return storage_min_ns, storage_max_ns, header_min_ns, header_max_ns


def _map_header_stamp_ns(
    old_header_ns: int,
    storage_ts_ns: int,
    mode: str,
    storage_min_ns: int,
    storage_max_ns: int,
    header_min_ns: int,
    header_max_ns: int,
) -> int:
    if mode == 'storage':
        return int(storage_ts_ns)
    if mode == 'offset':
        return int(old_header_ns + (storage_min_ns - header_min_ns))
    if mode == 'affine':
        if header_max_ns == header_min_ns:
            return int(storage_min_ns)
        num = (old_header_ns - header_min_ns) * (storage_max_ns - storage_min_ns)
        den = header_max_ns - header_min_ns
        return int(storage_min_ns + (num // den))
    raise RuntimeError(f'unsupported mode: {mode}')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'rosbag2 のメッセージヘッダ時刻を storage timestamp に同期して新しい bag を作成します。'
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
        default=[],
        help=(
            '補正対象トピック（カンマ区切り可）。未指定時は '
            'headerを持つ全トピック + /tf + /tf_static を補正'
        ),
    )
    parser.add_argument(
        '--bounds-topics',
        nargs='*',
        default=[],
        help=(
            '補正係数（offset/affine）の算出に使うトピック。'
            '未指定時は補正対象から /tf_static を除いた集合を使用'
        ),
    )
    parser.add_argument(
        '--mode',
        choices=['storage', 'offset', 'affine'],
        default='storage',
        help=(
            'ヘッダ補正方式: storage=各メッセージのstorage時刻へ一致, '
            'offset=一定オフセット加算, affine=全体区間を線形写像'
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_uri = str(Path(args.input).expanduser())
    output_uri = str(Path(args.output).expanduser())
    storage_id = args.storage
    mode = args.mode

    in_path = Path(input_uri)
    out_path = Path(output_uri)
    if not in_path.exists():
        raise FileNotFoundError(f'入力ディレクトリが存在しません: {input_uri}')
    if out_path.exists():
        raise FileExistsError(f'出力ディレクトリが既に存在します: {output_uri}')

    reader = _open_reader(input_uri, storage_id)
    topics = reader.get_all_topics_and_types()
    type_map: Dict[str, str] = {topic.name: topic.type for topic in topics}
    msg_cls_map = {name: get_message(msg_type) for name, msg_type in type_map.items()}

    specified_topics = set(_parse_topics(args.topics))
    unknown = [topic for topic in specified_topics if topic not in type_map]
    if unknown:
        raise RuntimeError(f'入力 bag に存在しないトピックがあります: {unknown}')

    auto_topics: Set[str] = set()
    if not specified_topics:
        for topic_name, msg_cls in msg_cls_map.items():
            if type_map[topic_name] == TF_MESSAGE_TYPE:
                auto_topics.add(topic_name)
                continue
            msg = msg_cls()
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                auto_topics.add(topic_name)
    target_topics = specified_topics if specified_topics else auto_topics

    if not target_topics:
        raise RuntimeError('補正対象トピックが見つかりませんでした。')

    specified_bounds_topics = set(_parse_topics(args.bounds_topics))
    if specified_bounds_topics:
        missing_bounds = [t for t in specified_bounds_topics if t not in target_topics]
        if missing_bounds:
            raise RuntimeError(
                'bounds topics は補正対象 topics の部分集合にしてください: '
                f'{missing_bounds}'
            )
        bounds_topics = specified_bounds_topics
    else:
        bounds_topics = {t for t in target_topics if t != '/tf_static'}
        if not bounds_topics:
            bounds_topics = set(target_topics)

    (
        storage_min_ns,
        storage_max_ns,
        header_min_ns,
        header_max_ns,
    ) = _compute_bounds(
        input_uri=input_uri,
        storage_id=storage_id,
        target_topics=target_topics,
        bounds_topics=bounds_topics,
        type_map=type_map,
        msg_cls_map=msg_cls_map,
    )

    if storage_min_ns is None or storage_max_ns is None:
        raise RuntimeError('入力 bag の storage 時刻範囲を取得できませんでした。')
    if header_min_ns is None or header_max_ns is None:
        raise RuntimeError('補正対象トピックに header 時刻を持つメッセージがありません。')

    print(f'[fix_header_stamps_to_storage] bounds_topics={sorted(bounds_topics)}')
    print(
        '[fix_header_stamps_to_storage] mode='
        f'{mode}, storage=[{storage_min_ns/1e9:.9f}, {storage_max_ns/1e9:.9f}], '
        f'header=[{header_min_ns/1e9:.9f}, {header_max_ns/1e9:.9f}]'
    )

    writer = _open_writer(output_uri, storage_id)
    for topic in topics:
        writer.create_topic(topic)

    patched_messages = 0
    patched_headers = 0
    patched_tf_transforms = 0

    for topic_name in sorted(target_topics):
        print(f'[fix_header_stamps_to_storage] target topic: {topic_name}')

    while reader.has_next():
        topic_name, data, storage_ts_ns = reader.read_next()
        if topic_name not in target_topics:
            writer.write(topic_name, data, storage_ts_ns)
            continue

        msg_cls = msg_cls_map[topic_name]
        msg = deserialize_message(data, msg_cls)
        patched = False

        if type_map[topic_name] == TF_MESSAGE_TYPE:
            for transform in msg.transforms:
                old_h = int(transform.header.stamp.sec) * NSEC_PER_SEC + int(
                    transform.header.stamp.nanosec
                )
                new_h = _map_header_stamp_ns(
                    old_header_ns=old_h,
                    storage_ts_ns=int(storage_ts_ns),
                    mode=mode,
                    storage_min_ns=storage_min_ns,
                    storage_max_ns=storage_max_ns,
                    header_min_ns=header_min_ns,
                    header_max_ns=header_max_ns,
                )
                transform.header.stamp.sec = int(new_h // NSEC_PER_SEC)
                transform.header.stamp.nanosec = int(new_h % NSEC_PER_SEC)
                patched_tf_transforms += 1
            patched = True
        else:
            old_h = _get_msg_header_stamp_ns(msg)
            if old_h is not None:
                new_h = _map_header_stamp_ns(
                    old_header_ns=old_h,
                    storage_ts_ns=int(storage_ts_ns),
                    mode=mode,
                    storage_min_ns=storage_min_ns,
                    storage_max_ns=storage_max_ns,
                    header_min_ns=header_min_ns,
                    header_max_ns=header_max_ns,
                )
                _set_msg_header_stamp_to_ns(msg, new_h)
                patched_headers += 1
                patched = True

        if patched:
            patched_messages += 1
            writer.write(topic_name, serialize_message(msg), storage_ts_ns)
        else:
            writer.write(topic_name, data, storage_ts_ns)

    print(f'[fix_header_stamps_to_storage] input={input_uri}')
    print(f'[fix_header_stamps_to_storage] output={output_uri}')
    print(f'[fix_header_stamps_to_storage] patched_messages={patched_messages}')
    print(f'[fix_header_stamps_to_storage] patched_headers={patched_headers}')
    print(f'[fix_header_stamps_to_storage] patched_tf_transforms={patched_tf_transforms}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
