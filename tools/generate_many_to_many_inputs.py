#!/usr/bin/env python3
import argparse
import copy
import json
import math
from pathlib import Path

R = 6371000.0


def mercator_to_lla(x, y, alt):
    lon = math.degrees(x / R)
    lat = math.degrees(2.0 * math.atan(math.exp(y / R)) - math.pi / 2.0)
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def read_json(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def write_json(path, obj):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)
        f.write('\n')


def patch_tanker(base, tanker_meta, fallback_index):
    data = copy.deepcopy(base)
    data['tanker_id'] = tanker_meta.get('id', f'TK{fallback_index:03d}')
    data.setdefault('performance_limits', {})
    data['performance_limits']['fuel_transfer_speed'] = tanker_meta.get(
        'fuel_transfer_speed', data['performance_limits'].get('fuel_transfer_speed', 25)
    )
    data.setdefault('current_status', {})
    if 'current_fuel' in tanker_meta:
        data['current_status']['fuel_kg'] = tanker_meta['current_fuel']
    if 'initial_position' in tanker_meta and len(tanker_meta['initial_position']) >= 2:
        alt = data.get('initial_position', {}).get('altitude', 5000)
        data['initial_position'] = mercator_to_lla(tanker_meta['initial_position'][0], tanker_meta['initial_position'][1], alt)
    return data


def patch_receiver(base, receiver_meta, fallback_index):
    data = copy.deepcopy(base)
    data['receiver_id'] = receiver_meta.get('id', f'RE{fallback_index:03d}')
    data.setdefault('current_status', {})
    if 'priority' in receiver_meta:
        data['current_status']['priority'] = receiver_meta['priority']
        data['priority'] = receiver_meta['priority']
    max_fuel = receiver_meta.get('max_fuel_kg', data.get('current_status', {}).get('max_fuel_kg', 7000))
    data['current_status']['max_fuel_kg'] = max_fuel
    data['max_fuel_kg'] = max_fuel
    if 'required_fuel' in receiver_meta:
        data['required_fuel_kg'] = receiver_meta['required_fuel']
    if 'initial_position' in receiver_meta and len(receiver_meta['initial_position']) >= 2:
        alt = data.get('initial_position', {}).get('altitude', 5000)
        data['initial_position'] = mercator_to_lla(receiver_meta['initial_position'][0], receiver_meta['initial_position'][1], alt)
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mission', required=True)
    ap.add_argument('--assignment', required=True)
    ap.add_argument('--tanker-template', required=True)
    ap.add_argument('--receiver-template', required=True)
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args()

    mission = read_json(args.mission)
    assignment = read_json(args.assignment)
    tanker_base = read_json(args.tanker_template)
    receiver_base = read_json(args.receiver_template)

    mission_list = mission if isinstance(mission, list) else [mission]
    mission0 = copy.deepcopy(mission_list[0])

    tanker_items = [x for x in assignment.get('Tankers', []) if isinstance(x, dict) and x.get('id')]
    receiver_items = [x for x in assignment.get('Receivers', []) if isinstance(x, dict) and x.get('id')]
    mission0.setdefault('units', {})
    mission0['units']['tanker_id'] = ' '.join(item['id'] for item in tanker_items)
    mission0['units']['receiver_id'] = ' '.join(item['id'] for item in receiver_items)
    mission0.setdefault('refuel_mode', {}).setdefault('timing_params', {})['sub_mode'] = 'dingdian'

    out_dir = Path(args.out_dir)
    write_json(out_dir / 'mission1.json', [mission0])
    write_json(out_dir / 'assignment.json', assignment)

    for idx, tanker_meta in enumerate(tanker_items, start=1):
        write_json(out_dir / f'tankers{idx:02d}_config.json', patch_tanker(tanker_base, tanker_meta, idx))
    for idx, receiver_meta in enumerate(receiver_items, start=1):
        write_json(out_dir / f'receivers{idx:02d}_config.json', patch_receiver(receiver_base, receiver_meta, idx))

    write_json(out_dir / 'generation_summary.json', {
        'generated_tankers': [x['id'] for x in tanker_items],
        'generated_receivers': [x['id'] for x in receiver_items],
        'assignment_count': len([x for x in assignment.get('assignments', []) if isinstance(x, dict) and 'tanker' in x and 'assigned_events' in x])
    })


if __name__ == '__main__':
    main()
