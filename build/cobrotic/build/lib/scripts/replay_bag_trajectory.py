import os
import sqlite3
import yaml
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import JointState

# ------------------------
# CONFIGURATION
# ------------------------
base_dir = 'myRecord'             # Root folder containing bag subfolders
topic = '/joint_states'
output_folder = 'merged_bag'
output_db = 'merged.db3'
output_yaml = 'rosbag2_bagfile_information.yaml'

# ------------------------
# HELPER FUNCTIONS
# ------------------------
def find_db3_file(folder):
    for root, _, files in os.walk(folder):
        for f in files:
            if f.endswith('.db3'):
                return os.path.join(root, f)
    raise FileNotFoundError(f"No .db3 files found in {folder}")

def read_bag_messages(db3_file, topic):
    conn = sqlite3.connect(db3_file)
    cursor = conn.cursor()
    cursor.execute("SELECT id FROM topics WHERE name=?", (topic,))
    topic_id_row = cursor.fetchone()
    if topic_id_row is None:
        conn.close()
        return []
    topic_id = topic_id_row[0]
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (topic_id,))
    rows = cursor.fetchall()
    conn.close()
    return [(ts, deserialize_message(data, JointState)) for ts, data in rows]

def write_bag_messages(output_folder, topic, messages):
    os.makedirs(output_folder, exist_ok=True)
    db_path = os.path.join(output_folder, output_db)
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    cursor.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT)")
    cursor.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    cursor.execute("INSERT INTO topics (id,name,type,serialization_format) VALUES (1, ?, ?, 'cdr')",
                   (topic, 'sensor_msgs/msg/JointState'))
    for ts, msg in messages:
        data = serialize_message(msg)
        cursor.execute("INSERT INTO messages (topic_id, timestamp, data) VALUES (1, ?, ?)", (ts, data))
    conn.commit()
    conn.close()
    print(f"✅ Merged {len(messages)} messages into {db_path}")
    return db_path, messages[0][0], messages[-1][0], len(messages)

def generate_bag_info(db_path, topic, start_ts, end_ts, count, output_yaml):
    duration_ns = end_ts - start_ts
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
                    'type': 'sensor_msgs/msg/JointState',
                    'serialization_format': 'cdr',
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
    with open(os.path.join(output_folder, output_yaml), 'w') as f:
        yaml.dump(bag_info, f, sort_keys=False)
    print(f"📄 Metadata written to {output_yaml}")

# ------------------------
# MAIN MERGE FUNCTION
# ------------------------
def merge_all_bags(base_dir, topic):
    all_messages = []
    folders = sorted([os.path.join(base_dir, f) for f in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, f))])
    for folder in folders:
        try:
            db3_file = find_db3_file(folder)
            msgs = read_bag_messages(db3_file, topic)
            if not msgs:
                continue
            if all_messages:
                last_ts = all_messages[-1][0]
                offset = last_ts - msgs[0][0] + 1
                msgs = [(ts + offset, msg) for ts, msg in msgs]
            all_messages.extend(msgs)
            print(f"✔ Read {len(msgs)} messages from {folder}")
        except Exception as e:
            print(f"⚠️ Skipped {folder}: {e}")
    db_path, start_ts, end_ts, count = write_bag_messages(output_folder, topic, all_messages)
    generate_bag_info(db_path, topic, start_ts, end_ts, count, output_yaml)

# ------------------------
# RUN
# ------------------------
if __name__ == '__main__':
    merge_all_bags(base_dir, topic)