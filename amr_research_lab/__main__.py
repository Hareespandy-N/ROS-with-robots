import argparse
import json
from pathlib import Path

from .docking import run_docking
from .reporting import write_comparison, write_report
from .simulation import CONTROLLERS, SCENARIOS, Simulation


def main(argv=None):
    parser = argparse.ArgumentParser(description='Original offline AMR simulation and research experiments')
    commands = parser.add_subparsers(dest='command', required=True)
    single = commands.add_parser('run', help='run one ideal-kinematics experiment')
    single.add_argument('--scenario', choices=SCENARIOS, default='warehouse')
    single.add_argument('--controller', choices=CONTROLLERS, default='regulated')
    single.add_argument('--output', type=Path, default=Path('results'))
    batch = commands.add_parser('benchmark', help='run the six fixed comparison cases')
    batch.add_argument('--output', type=Path, default=Path('results'))
    commands.add_parser('dock', help='simulate pose alignment without charger I/O')
    args = parser.parse_args(argv)
    if args.command == 'dock':
        result = run_docking()
        print(json.dumps(result, indent=2))
        return 0 if result['status'] == 'DOCKED' else 1
    if args.command == 'run':
        sim = Simulation(args.scenario, args.controller).run()
        write_report(sim, args.output)
        print(json.dumps(sim.metrics(), indent=2))
        return 0 if sim.status == 'completed' else 1
    results = []
    for scenario in SCENARIOS:
        for controller in CONTROLLERS:
            sim = Simulation(scenario, controller).run()
            write_report(sim, args.output)
            results.append(sim.metrics())
    write_comparison(results, args.output)
    (args.output / 'docking.json').write_text(json.dumps(run_docking(), indent=2) + '\n', encoding='utf-8')
    print(json.dumps(results, indent=2))
    return 0 if all(result['status'] == 'completed' for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
