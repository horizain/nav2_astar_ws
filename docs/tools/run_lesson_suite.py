#!/usr/bin/env python3
"""Run a bounded matrix of fixed lessons. Every attempt, including failures, is kept."""
import argparse
import ast
import json
from pathlib import Path
import subprocess
import sys
import uuid

from teaching_scenarios import SCENARIOS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controllers", nargs="+",
                        choices=["FollowPath", "PID", "Stanley", "DWB", "MPPI", "MPC"],
                        default=["FollowPath", "MPC"])
    parser.add_argument("--scenarios", nargs="+", choices=SCENARIOS, default=["straight", "arc"])
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--domain", type=int, default=203)
    parser.add_argument("--output", type=Path, default=Path("/tmp/navigation_suites"))
    parser.add_argument("--timeout", type=float, default=90)
    args = parser.parse_args()
    if args.repeat < 1 or not 0 <= args.domain <= 232:
        parser.error("Invalid repetitions or domain")
    tool_dir = Path(__file__).resolve().parent
    for script in ("run_navigation_lesson.py", "navigation_metrics.py", "teaching_scenarios.py"):
        ast.parse((tool_dir / script).read_text(), filename=script)
    output = args.output / ("suite_" + uuid.uuid4().hex[:10])
    output.mkdir(parents=True, exist_ok=False)
    attempts = []
    print(f"Suite: {output}", flush=True)
    for scenario in args.scenarios:
        for controller in args.controllers:
            for repetition in range(1, args.repeat + 1):
                command = [sys.executable, str(Path(__file__).with_name("run_navigation_lesson.py")),
                           "--scenario", scenario, "--controller", controller,
                           "--domain", str(args.domain), "--output", str(output),
                           "--timeout", str(args.timeout)]
                before = set(output.iterdir())
                log_path = output / f"{scenario}_{controller}_repeat{repetition}.log"
                print(f"RUN {scenario} / {controller} / {repetition}", flush=True)
                with log_path.open("w") as log:
                    child = subprocess.Popen(command, stdout=subprocess.PIPE,
                                             stderr=subprocess.STDOUT, text=True)
                    try:
                        for line in child.stdout:
                            log.write(line)
                            log.flush()
                            print(line.rstrip(), flush=True)
                        code = child.wait()
                    except KeyboardInterrupt:
                        child.send_signal(__import__("signal").SIGINT)
                        child.wait(timeout=45)
                        raise
                artifacts = [p for p in set(output.iterdir()) - before if p.is_dir()]
                attempts.append(dict(
                    scenario=scenario, controller=controller, repetition=repetition,
                    exit_code=code, functional_expectation_met=code == 0,
                    artifacts=[str(p.relative_to(output)) for p in sorted(artifacts)],
                    log=log_path.name))
                (output / "suite.json").write_text(json.dumps(
                    dict(schema_version=1, note="All attempts retained; no automatic target substitution",
                         attempts=attempts), ensure_ascii=False, indent=2))
    completed = sum(a["functional_expectation_met"] for a in attempts)
    print(f"Functional expectations met: {completed}/{len(attempts)}; {output}", flush=True)
    return 0 if completed == len(attempts) else 1


if __name__ == "__main__":
    sys.exit(main())
