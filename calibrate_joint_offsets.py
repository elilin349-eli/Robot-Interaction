#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive calibration helper to collect matching leader/follower poses
and compute robust per-joint median offsets, then save to joint_offsets.json.

Usage:
  python calibrate_joint_offsets.py --count 4 --leader-port /dev/leader_arm --follower-port /dev/follower_arm

This script ONLY reads encoder values; it does not move servos. Please manually
place both arms in the same physical pose before pressing Enter to record each
pair.
"""
import argparse
import json
import os
import sys
from statistics import median

try:
    from lerobot_hardware import SO101LeaderArm
except Exception as e:
    print(f"⚠️ 无法导入 lerobot_hardware: {e}")
    sys.exit(1)


def read_angles_safe(arm):
    if not arm:
        return None
    if not arm.connected:
        if not arm.connect():
            return None
    return arm.read_angles()


def compute_median_offsets(pairs):
    per_joint = {i: [] for i in range(6)}
    for leader, follower in pairs:
        for i in range(6):
            per_joint[i].append(int(follower[i]) - int(leader[i]))
    offsets = {}
    for i in range(6):
        vals = per_joint[i]
        if not vals:
            offsets[i + 1] = 0
        else:
            offsets[i + 1] = int(median(vals))
    return offsets


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--count', type=int, default=3, help='Number of pose pairs to record (default: 3)')
    p.add_argument('--leader-port', default=None, help='Leader arm port (e.g. /dev/leader_arm)')
    p.add_argument('--follower-port', default=None, help='Follower arm port (e.g. /dev/follower_arm)')
    p.add_argument('--out', default=os.path.join(os.path.dirname(__file__), 'joint_offsets.json'), help='Output file')
    args = p.parse_args()

    leader_port = args.leader_port
    follower_port = args.follower_port

    print('\n=== Joint offsets calibration helper ===')
    print('This tool will record matching encoder readings from leader and follower.')
    print('Manually move both arms into the SAME physical pose, then press Enter to record.')
    print('Do NOT power the arms off during calibration. The script will only read values.\n')

    try:
        leader = SO101LeaderArm(port=leader_port) if leader_port else SO101LeaderArm()
        follower = SO101LeaderArm(port=follower_port, role='follower') if follower_port else SO101LeaderArm(role='follower')
    except Exception as e:
        print(f"⚠️ 无法创建 SO101LeaderArm 实例: {e}")
        sys.exit(1)

    print('Connecting to leader arm...')
    if not leader.connect():
        print('⚠️ 无法连接主臂，继续仍可尝试读取但可能失败')
    else:
        print('✅ 主臂已连接')

    print('Connecting to follower arm...')
    if not follower.connect():
        print('⚠️ 无法连接从臂，继续仍可尝试读取 but results may be invalid')
    else:
        print('✅ 从臂已连接')

    pairs = []
    for i in range(args.count):
        input(f"\n准备记录第 {i+1} 个姿态。请将主臂与从臂摆成相同姿态，然后按 Enter 继续...")
        leader_angles = read_angles_safe(leader)
        follower_angles = read_angles_safe(follower)
        if not leader_angles or not follower_angles or len(leader_angles) < 6 or len(follower_angles) < 6:
            print('⚠️ 读取失败，请检查连接与电源，然后重试该姿态')
            continue
        print(f"  主臂: {leader_angles}")
        print(f"  从臂: {follower_angles}")
        pairs.append((leader_angles, follower_angles))

    if not pairs:
        print('❌ 未记录任何有效姿态对，退出')
        leader.disconnect()
        follower.disconnect()
        sys.exit(1)

    offsets = compute_median_offsets(pairs)
    print('\n=== 计算得到的 JOINT_OFFSETS (follower - leader) ===')
    for j in range(1, 7):
        print(f'  J{j}: {offsets[j]}')

    confirm = input(f"\n保存到 {args.out}? (y/n) > ").strip().lower()
    if confirm == 'y':
        try:
            # backup existing
            if os.path.exists(args.out):
                bak = args.out + '.bak'
                try:
                    os.replace(args.out, bak)
                    print(f'已备份原文件到: {bak}')
                except Exception:
                    pass
            # write with string keys for compatibility
            out_data = {str(k): int(v) for k, v in offsets.items()}
            with open(args.out, 'w', encoding='utf-8') as fh:
                json.dump(out_data, fh, ensure_ascii=False, indent=2)
            print(f'✅ 已保存: {args.out}')
        except Exception as e:
            print(f'❌ 保存失败: {e}')
    else:
        print('已取消保存')

    try:
        leader.disconnect()
    except Exception:
        pass
    try:
        follower.disconnect()
    except Exception:
        pass


if __name__ == '__main__':
    main()

