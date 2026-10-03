import json, glob, os

results_dir = os.path.dirname(os.path.abspath(__file__))

def load_files(pattern):
    files = sorted(glob.glob(os.path.join(results_dir, pattern)))
    rows = []
    for f in files:
        name = os.path.basename(f).replace("batch_results_", "").replace(".json", "")
        try:
            with open(f) as fh:
                d = json.load(fh)
            n = len(d.get("results", []))
            s = d.get("summary", {})
            rows.append((name, n, s))
        except Exception as e:
            rows.append((name, 0, {}))
    return rows

v1 = load_files("batch_results_[!_]*.json")   # excludes _v2
v2 = load_files("batch_results_*_v2.json")

def print_table(rows, title):
    if not rows:
        return
    print(f"\n{title}")
    print(f"  {'Strategy':<26} {'Runs':>5}  {'Success':>8}  {'Partial':>8}  {'Failed':>7}  {'MPI':>6}  {'Escapes':>8}")
    print("  " + "-" * 72)
    for name, n, s in rows:
        mpi = s.get("avg_mpi", "-")
        print(f"  {name:<26} {n:>5}  {str(s.get('success_rate','-'))+'%':>8}  "
              f"{str(s.get('partial_rate','-'))+'%':>8}  "
              f"{str(s.get('failed_rate','-'))+'%':>7}  "
              f"{str(mpi):>6}  "
              f"{str(s.get('avg_escapes','-')):>8}")

print_table(v1, "=== V1 (Standard profile) ===")
print_table(v2, "=== V2 (Stress profile)   ===")

if not v1 and not v2:
    print("No result files found.")
