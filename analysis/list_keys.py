"""List all logged keys in a wpilog file."""
import sys
from wpiutil.log import DataLogReader

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "../logs/akit_26-03-08_14-58-00_njwas_q49.wpilog"
    reader = DataLogReader(path)
    entries = {}
    for record in reader:
        if record.isStart():
            data = record.getStartData()
            entries[data.entry] = (data.name, data.type)
    for eid, (name, typ) in sorted(entries.items(), key=lambda x: x[1][0]):
        print(f"{name:60s}  {typ}")

if __name__ == "__main__":
    main()
