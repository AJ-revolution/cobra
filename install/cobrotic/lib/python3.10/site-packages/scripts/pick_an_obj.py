import os
import sqlite3
import yaml

def generate_bag_info(db_path, topic='/joint_states', output_yaml='rosbag2_bagfile_information.yaml'):
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Get topic metadata
    cursor.execute("SELECT id, type, serialization_format FROM topics WHERE name=?", (topic,))
    topic_row = cursor.fetchone()
    if topic_row is None:
        raise ValueError(f"Topic {topic} not found in {db_path}")
    topic_id, topic_type, serialization_format = topic_row

    # Get message count and timestamps
    cursor.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages WHERE topic_id=?", (topic_id,))
    count, start_ts, end_ts = cursor.fetchone()
    duration_ns = end_ts - start_ts

    # Build YAML structure
    bag_info = {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': duration_ns},
            'starting_time': {'nanoseconds_since_epoch': start_ts},
            'message_count': count,
            'topics_with_message_count': [{
                'topic_metadata': {
                    'name': topic,
                    'type': topic_type,
                    'serialization_format': serialization_format,
                    'offered_qos_profiles': "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 1\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
                },
                'message_count': count
            }],
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [os.path.basename(db_path)],
            'files': [{
                'path': os.path.basename(db_path),
                'starting_time': {'nanoseconds_since_epoch': start_ts},
                'duration': {'nanoseconds': duration_ns},
                'message_count': count
            }]
        }
    }

    # Write to YAML
    with open(output_yaml, 'w') as f:
        yaml.dump(bag_info, f, sort_keys=False)

    print(f"✅ Metadata written to {output_yaml}")