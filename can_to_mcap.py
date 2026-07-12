import cantools
import argparse
import re
import os
import json

from mcap.writer import Writer
from typing import Optional, Tuple

parser = argparse.ArgumentParser(description="Convert raw CAN logs to timestamped MCAP files")

parser.add_argument("log", help="The log file to parse.")
parser.add_argument("dbc", help="The DBC file path to use for parsing")

args = parser.parse_args()

INFILE = args.log
base, _ = os.path.splitext(INFILE)
OUTFILE = base + ".mcap"

def parse_line(line: str) -> Optional[Tuple[float, int, bytes]]:
    match = re.match(
        r"\(([\d\.]+)\)\s+\w+\s+([0-9A-Fa-f]+)\s+\[\d+\]\s+((?:[0-9A-Fa-f]{2}\s*)+)", 
        line.strip()
    )
    if not match:
        return None
    t = float(match.group(1))
    can_id = int(match.group(2), 16)
    data = bytes.fromhex(match.group(3))
    return t, can_id, data

def main():
    dbc = cantools.database.load_file(args.dbc)
    writer = Writer(open(OUTFILE, "wb"))
    writer.start()

    test_schema = writer.register_schema(
        name="CANData",
        encoding="jsonschema",
        data=b"""
        {
            "type": "object",
            "properties": {
                "value": {"type": "number"}
            }
        }
        """,
    )
    channels = {}
    with open(INFILE, "r") as f:
        for line in f:
            parsed = parse_line(line)
            if not parsed:
                continue
            t, can_id, data = parsed
            log_time_ns = int(t * 1e9)

            try:
                msg = dbc.get_message_by_frame_id(can_id)
            except KeyError:
                continue

            try:
                decoded = msg.decode(data)
            except Exception:
                continue

            for name, val in decoded.items():
                if hasattr(val, "value"):
                    val = val.value
                topic = f"/can/{msg.name}/{name}"

                if topic not in channels:
                    channels[topic] = writer.register_channel(
                        topic=topic,
                        message_encoding="json",
                        schema_id=test_schema,
                    )

                data_bytes = json.dumps({"value": val}).encode()
                writer.add_message(
                    channel_id=channels[topic],
                    log_time=log_time_ns,
                    publish_time=log_time_ns,
                    data=data_bytes,
                )

    writer.finish()

if __name__ == "__main__":
    main()
