"""Plot recorded K1 recovery joint targets, feedback and estimated torque.

The native firmware target is not published by the recorded topics. The plot
shows only observed values and explicitly marks that missing sixth series.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


DEFAULT_JOINTS = (2, 5, 6, 9, 10, 16)
JOINT_NAMES = {
    2: "左肩 pitch", 5: "左肘 yaw", 6: "右肩 pitch",
    9: "右肘 yaw", 10: "左髋 pitch", 16: "右髋 pitch",
}
COLORS = {
    "firmware_q": "#087f63",
    "deploy_q": "#5ac3a5",
    "deploy_target": "#2467b1",
    "firmware_tau": "#b85126",
    "deploy_tau": "#ef9b5f",
}


def read_trial(directory):
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    feedback, commands = [], []
    with (directory / "joint_tracking.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("type") == "feedback" and row.get("valid_serial_22"):
                feedback.append(row)
            elif row.get("type") == "command" and row.get("valid_serial_22"):
                commands.append(row)
    if not feedback:
        raise ValueError(f"No valid feedback in {directory}")
    feedback.sort(key=lambda row: row["wall_time_ns"])
    commands.sort(key=lambda row: row["wall_time_ns"])
    return manifest, feedback, commands


def event_ns(directory, manifest, marker):
    year = datetime.fromisoformat(manifest["start"]).year
    tz = datetime.fromisoformat(manifest["start"]).tzinfo
    pattern = re.compile(r"\[(\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]")
    with (directory / "firmware.log").open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if marker not in line:
                continue
            match = pattern.search(line)
            if match:
                stamp = datetime.strptime(f"{year}-{match.group(1)}", "%Y-%m-%d %H:%M:%S.%f")
                return int(stamp.replace(tzinfo=tz).timestamp() * 1e9)
    raise ValueError(f"Event {marker!r} missing from {directory / 'firmware.log'}")


def crossing_ns(feedback, joint, angle, begin_ns):
    for row in feedback:
        if row["wall_time_ns"] >= begin_ns and row["q"][joint] <= angle:
            return row["wall_time_ns"]
    raise ValueError(f"Joint {joint} did not reach {angle} rad")


def extract(feedback, commands, align_ns, earliest_ns, latest_ns, joint):
    f = [row for row in feedback if earliest_ns <= row["wall_time_ns"] < latest_ns]
    c = [row for row in commands if earliest_ns <= row["wall_time_ns"] < latest_ns]
    return {
        "feedback_t": [(row["wall_time_ns"] - align_ns) / 1e9 for row in f],
        "q": [row["q"][joint] for row in f],
        "tau": [row["tau_est"][joint] for row in f],
        "command_t": [(row["wall_time_ns"] - align_ns) / 1e9 for row in c],
        "target": [row["q"][joint] for row in c],
    }


def plot(firmware_dir, deploy_dir, output_prefix, joints, align_joint, align_angle, xlim):
    fm, ff, fc = read_trial(firmware_dir)
    dm, df, dc = read_trial(deploy_dir)
    if fc:
        raise ValueError("Firmware trial has external /joint_ctrl commands; target semantics need review")
    if not dc:
        raise ValueError("Deploy trial has no /joint_ctrl commands")

    fdr_start = event_ns(firmware_dir, fm, "OnEnter FallDownRecovery")
    fdr_success = event_ns(firmware_dir, fm, "FDR exec succeeded")
    try:
        fault = event_ns(deploy_dir, dm, "locked-rotor")
    except ValueError:
        fault = None
    f_align = crossing_ns(ff, align_joint, align_angle, fdr_start)
    d_align = crossing_ns(df, align_joint, align_angle, dc[0]["wall_time_ns"])
    deploy_end = min(fault, dc[-1]["wall_time_ns"]) if fault else dc[-1]["wall_time_ns"]
    if not fdr_start < f_align < fdr_success or not d_align < deploy_end:
        raise ValueError("Alignment event falls outside the recovery comparison window")

    plt.rcParams.update({
        "font.family": "Noto Sans CJK JP", "font.size": 10,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.titleweight": "bold", "savefig.facecolor": "white",
    })
    fig, axes = plt.subplots(3, 2, figsize=(17, 12), sharex=True, constrained_layout=False)
    fig.suptitle("K1 recovery：固件与 deploy 的关节角及估计力矩", fontsize=18, y=0.985)
    end_note = "首次堵转前" if fault else "最后一条 /joint_ctrl 命令"
    fig.text(0.5, 0.952,
             f"以左髋实测角首次到达 {align_angle:.2f} rad 对齐；固件截至 FDR 成功，deploy 截至{end_note}",
             ha="center", fontsize=11)

    for ax, joint in zip(axes.flat, joints):
        native = extract(ff, fc, f_align, fdr_start, fdr_success, joint)
        custom = extract(df, dc, d_align, dc[0]["wall_time_ns"], deploy_end + 1, joint)
        ax_torque = ax.twinx()
        ax.plot(native["feedback_t"], native["q"], color=COLORS["firmware_q"], lw=1.75)
        ax.plot(custom["feedback_t"], custom["q"], color=COLORS["deploy_q"], lw=1.75)
        ax.plot(custom["command_t"], custom["target"], color=COLORS["deploy_target"],
                lw=1.45, ls="--")
        ax_torque.plot(native["feedback_t"], native["tau"],
                       color=COLORS["firmware_tau"], lw=1.05, alpha=0.82)
        ax_torque.plot(custom["feedback_t"], custom["tau"],
                       color=COLORS["deploy_tau"], lw=1.05, ls="--", alpha=0.88)
        ax.axvline(0, color="#69717c", lw=0.9, alpha=0.75)
        ax.axvline((fdr_success - f_align) / 1e9, color=COLORS["firmware_q"],
                   lw=0.85, ls=":", alpha=0.8)
        if fault is not None:
            ax.axvline((fault - d_align) / 1e9, color="#a82832", lw=0.85,
                       ls=":", alpha=0.8)
        ax.set_xlim(*xlim)
        ax.set_title(f"J{joint}  {JOINT_NAMES.get(joint, '关节')}", loc="left", fontsize=11)
        ax.set_ylabel("关节角 (rad)")
        ax_torque.set_ylabel("估计力矩 (N·m)", color="#a3512f")
        ax_torque.tick_params(axis="y", colors="#a3512f")
        if 2 <= joint <= 9:
            ax_torque.set_ylim(-16, 16)
            ax_torque.set_yticks((-14, -7, 0, 7, 14))
        elif joint in (10, 16):
            ax_torque.set_ylim(-75, 75)
        ax.grid(color="#e6e9ed", lw=0.7)
        ax.set_axisbelow(True)

    for ax in axes[-1]:
        ax.set_xlabel(f"相对左髋达到 {align_angle:.2f} rad 的时间 (s)")

    legend = [
        Line2D([], [], color=COLORS["firmware_q"], lw=2, label="固件实测角"),
        Line2D([], [], color=COLORS["deploy_target"], lw=2, ls="--", label="deploy 发送目标角"),
        Line2D([], [], color=COLORS["deploy_q"], lw=2, label="deploy 实测角"),
        Line2D([], [], color=COLORS["firmware_tau"], lw=2, label="固件 tau_est"),
        Line2D([], [], color=COLORS["deploy_tau"], lw=2, ls="--", label="deploy tau_est"),
        Line2D([], [], color=COLORS["firmware_q"], lw=1, ls=":", label="固件 FDR 成功"),
    ]
    if fault is not None:
        legend.append(Line2D([], [], color="#a82832", lw=1, ls=":",
                             label="deploy 首次堵转"))
    fig.legend(handles=legend, loc="upper center", ncol=6, bbox_to_anchor=(0.5, 0.927),
               frameon=False, fontsize=10)
    fig.text(0.5, 0.012,
             "固件内部发送目标角没有公开，因此第 6 条数据曲线无法绘制。绿色=实测角，蓝色=目标角，橙色=反馈估计力矩。",
             ha="center", color="#555c65", fontsize=10)
    fig.subplots_adjust(left=0.065, right=0.91, top=0.88, bottom=0.065,
                        hspace=0.29, wspace=0.29)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    png = output_prefix.with_suffix(".png")
    pdf = output_prefix.with_suffix(".pdf")
    fig.savefig(png, dpi=220)
    fig.savefig(pdf)
    plt.close(fig)
    return png, pdf, ((fault - d_align) / 1e9 if fault is not None else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("firmware", type=Path)
    parser.add_argument("deploy", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--joints", type=int, nargs="+", default=DEFAULT_JOINTS)
    parser.add_argument("--align-joint", type=int, default=10)
    parser.add_argument("--align-angle", type=float, default=-2.30)
    parser.add_argument("--xlim", type=float, nargs=2, default=(-3.0, 3.8),
                        metavar=("START", "END"))
    args = parser.parse_args()
    if len(args.joints) != 6 or any(j < 0 or j >= 22 for j in args.joints):
        parser.error("--joints must contain exactly six joint indices in 0..21")
    if args.xlim[0] >= args.xlim[1]:
        parser.error("--xlim START must be less than END")
    png, pdf, fault_s = plot(args.firmware, args.deploy, args.output_prefix,
                             args.joints, args.align_joint, args.align_angle, args.xlim)
    suffix = f"; deploy fault at alignment +{fault_s:.3f} s" if fault_s is not None else ""
    print(f"Saved {png} and {pdf}{suffix}")


if __name__ == "__main__":
    main()
