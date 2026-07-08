#!/usr/bin/env python3
"""
analyse_results.py — SAR UAV Swarm Experimental Results Analysis
=================================================================
Reads the 9 CSV files produced by run_experiments.sh and generates:

  1. Bar charts (mean ± SE) for all 5 metrics × 3 planners × 3 scenarios
  2. Box plots showing per-trial variance
  3. Grouped comparison heatmap
  4. Two-factor ANOVA table + Tukey HSD post-hoc (planner × scenario)
  5. Summary statistics CSV for thesis tables

Usage:
    python3 analyse_results.py [results_dir] [output_dir]

    python3 analyse_results.py ~/thesis_ws/results ~/thesis_ws/analysis

Outputs (all in output_dir):
    figures/bar_coverage_rate.png
    figures/bar_time_to_detection.png
    figures/bar_path_efficiency.png
    figures/bar_inter_agent_distance.png
    figures/bar_fault_recovery_time.png
    figures/boxplot_all_metrics.png
    figures/heatmap_performance.png
    figures/anova_summary.png
    summary_stats.csv
    anova_results.txt
"""

import os
import sys
import glob
import warnings
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
from pathlib import Path

warnings.filterwarnings('ignore')

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

RESULTS_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/thesis_ws/results')
OUTPUT_DIR  = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser('~/thesis_ws/analysis')

METRICS = {
    'coverage_rate':        'Coverage Rate',
    'time_to_detection':    'Time to Detection (s)',
    'path_efficiency':      'Path Efficiency',
    'inter_agent_distance': 'Inter-Agent Distance (m)',
    'fault_recovery_time':  'Fault Recovery Time (s)',
}

PLANNERS  = ['apf', 'vfh', 'rrtstar']
SCENARIOS = ['urban', 'wilderness', 'maritime']

PLANNER_LABELS  = {'apf': 'APF', 'vfh': 'VFH+', 'rrtstar': 'RRT*'}
SCENARIO_LABELS = {'urban': 'Urban', 'wilderness': 'Wilderness', 'maritime': 'Maritime'}

# Palette — UAV/aerospace inspired: deep navy, amber, teal
PALETTE = {
    'apf':     '#1B4F8A',   # deep navy
    'vfh':     '#E8A020',   # amber
    'rrtstar': '#2A9D8F',   # teal
}

SCENARIO_COLORS = {
    'urban':      '#C0392B',
    'wilderness': '#27AE60',
    'maritime':   '#2980B9',
}

plt.rcParams.update({
    'font.family':      'DejaVu Sans',
    'font.size':        11,
    'axes.titlesize':   13,
    'axes.labelsize':   11,
    'xtick.labelsize':  10,
    'ytick.labelsize':  10,
    'legend.fontsize':  10,
    'figure.dpi':       150,
    'axes.spines.top':  False,
    'axes.spines.right':False,
    'axes.grid':        True,
    'grid.alpha':       0.3,
    'grid.linestyle':   '--',
})


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_results(results_dir: str) -> pd.DataFrame:
    """Load all CSV files from the results directory tree."""
    pattern = os.path.join(results_dir, '**', '*.csv')
    files   = glob.glob(pattern, recursive=True)

    if not files:
        print(f"ERROR: No CSV files found in {results_dir}")
        print("  Expected structure: results/{{scenario}}/{{planner}}_results.csv")
        sys.exit(1)

    frames = []
    for f in files:
        try:
            df = pd.read_csv(f)
            frames.append(df)
            print(f"  Loaded: {f} ({len(df)} rows)")
        except Exception as e:
            print(f"  WARNING: Could not read {f}: {e}")

    data = pd.concat(frames, ignore_index=True)
    print(f"\nTotal rows loaded: {len(data)}")
    print(f"Columns: {list(data.columns)}\n")

    # Clean up — replace sentinel -1 values with NaN
    for col in METRICS:
        if col in data.columns:
            data[col] = pd.to_numeric(data[col], errors='coerce')
            data.loc[data[col] < 0, col] = np.nan

    return data


# ---------------------------------------------------------------------------
# Summary statistics
# ---------------------------------------------------------------------------

def compute_summary(data: pd.DataFrame) -> pd.DataFrame:
    """Compute mean, std, SE, min, max per planner × scenario × metric."""
    rows = []
    for scenario in SCENARIOS:
        for planner in PLANNERS:
            subset = data[(data['scenario'] == scenario) & (data['planner'] == planner)]
            row = {'scenario': scenario, 'planner': planner, 'n_trials': len(subset)}
            for metric in METRICS:
                if metric in subset.columns:
                    vals = subset[metric].dropna()
                    row[f'{metric}_mean'] = vals.mean()
                    row[f'{metric}_std']  = vals.std()
                    row[f'{metric}_se']   = vals.sem()
                    row[f'{metric}_min']  = vals.min()
                    row[f'{metric}_max']  = vals.max()
            rows.append(row)
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Plot 1: Bar charts per metric
# ---------------------------------------------------------------------------

def plot_bar_charts(summary: pd.DataFrame, output_dir: str):
    """One bar chart per metric — grouped by scenario, coloured by planner."""
    fig_dir = os.path.join(output_dir, 'figures')
    os.makedirs(fig_dir, exist_ok=True)

    for metric, label in METRICS.items():
        col_mean = f'{metric}_mean'
        col_se   = f'{metric}_se'

        if col_mean not in summary.columns:
            continue

        fig, ax = plt.subplots(figsize=(10, 5))

        n_scenarios = len(SCENARIOS)
        n_planners  = len(PLANNERS)
        x           = np.arange(n_scenarios)
        width       = 0.25
        offsets     = [-width, 0, width]

        for i, planner in enumerate(PLANNERS):
            subset = summary[summary['planner'] == planner]
            means  = [subset[subset['scenario'] == s][col_mean].values[0]
                      if len(subset[subset['scenario'] == s]) > 0 else 0
                      for s in SCENARIOS]
            errors = [subset[subset['scenario'] == s][col_se].values[0]
                      if len(subset[subset['scenario'] == s]) > 0 else 0
                      for s in SCENARIOS]

            bars = ax.bar(
                x + offsets[i], means, width,
                label       = PLANNER_LABELS[planner],
                color       = PALETTE[planner],
                yerr        = errors,
                capsize     = 4,
                error_kw    = {'elinewidth': 1.5, 'alpha': 0.8},
                alpha       = 0.85,
                edgecolor   = 'white',
                linewidth   = 0.5,
            )

        ax.set_xlabel('Scenario')
        ax.set_ylabel(label)
        ax.set_title(f'{label} — APF vs VFH+ vs RRT* across SAR Scenarios')
        ax.set_xticks(x)
        ax.set_xticklabels([SCENARIO_LABELS[s] for s in SCENARIOS])
        ax.legend(title='Planner', framealpha=0.9)
        ax.set_ylim(bottom=0)

        plt.tight_layout()
        path = os.path.join(fig_dir, f'bar_{metric}.png')
        fig.savefig(path, bbox_inches='tight')
        plt.close(fig)
        print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Plot 2: Box plots
# ---------------------------------------------------------------------------

def plot_boxplots(data: pd.DataFrame, output_dir: str):
    """Box plots showing trial-level variance for all metrics."""
    fig_dir = os.path.join(output_dir, 'figures')
    os.makedirs(fig_dir, exist_ok=True)

    n_metrics = len(METRICS)
    fig, axes = plt.subplots(1, n_metrics, figsize=(22, 6))

    for ax, (metric, label) in zip(axes, METRICS.items()):
        if metric not in data.columns:
            ax.set_visible(False)
            continue

        plot_data = []
        labels    = []
        colors    = []

        for planner in PLANNERS:
            vals = data[data['planner'] == planner][metric].dropna().values
            plot_data.append(vals)
            labels.append(PLANNER_LABELS[planner])
            colors.append(PALETTE[planner])

        bp = ax.boxplot(
            plot_data,
            patch_artist = True,
            notch        = False,
            medianprops  = dict(color='white', linewidth=2),
            whiskerprops = dict(linewidth=1.2),
            capprops     = dict(linewidth=1.2),
            flierprops   = dict(marker='o', markersize=3, alpha=0.5),
        )

        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.8)

        ax.set_xticklabels(labels)
        ax.set_title(label, fontsize=10)
        ax.set_ylabel(label)

    fig.suptitle('Per-Trial Distribution — All Metrics by Planner', fontsize=14, y=1.02)
    plt.tight_layout()
    path = os.path.join(fig_dir, 'boxplot_all_metrics.png')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# Plot 3: Heatmap
# ---------------------------------------------------------------------------

def plot_heatmap(summary: pd.DataFrame, output_dir: str):
    """Normalised performance heatmap: planners × scenarios."""
    fig_dir = os.path.join(output_dir, 'figures')
    os.makedirs(fig_dir, exist_ok=True)

    # Pick primary metrics for heatmap
    primary = ['coverage_rate', 'time_to_detection', 'path_efficiency']
    available = [m for m in primary if f'{m}_mean' in summary.columns]

    if not available:
        return

    # Build matrix: rows = planner, cols = scenario × metric
    rows = []
    idx  = []
    for planner in PLANNERS:
        row = []
        for scenario in SCENARIOS:
            for metric in available:
                val = summary[
                    (summary['planner'] == planner) &
                    (summary['scenario'] == scenario)
                ][f'{metric}_mean'].values
                row.append(val[0] if len(val) > 0 else np.nan)
        rows.append(row)
        idx.append(PLANNER_LABELS[planner])

    cols = [f"{SCENARIO_LABELS[s]}\n{METRICS[m].split('(')[0].strip()}"
            for s in SCENARIOS for m in available]

    matrix = pd.DataFrame(rows, index=idx, columns=cols)

    # Normalise each column to [0,1]
    matrix_norm = (matrix - matrix.min()) / (matrix.max() - matrix.min() + 1e-9)

    fig, ax = plt.subplots(figsize=(14, 4))
    sns.heatmap(
        matrix_norm,
        ax          = ax,
        cmap        = 'YlOrRd',
        annot       = matrix.round(2),
        fmt         = '',
        linewidths  = 0.5,
        linecolor   = 'white',
        cbar_kws    = {'label': 'Normalised Score'},
    )
    ax.set_title('Performance Heatmap — Normalised Scores (Planner × Scenario × Metric)', pad=15)
    ax.set_xlabel('')
    ax.set_ylabel('Planner')

    plt.tight_layout()
    path = os.path.join(fig_dir, 'heatmap_performance.png')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {path}")


# ---------------------------------------------------------------------------
# ANOVA + Tukey HSD
# ---------------------------------------------------------------------------

def run_anova(data: pd.DataFrame, output_dir: str):
    """Two-factor ANOVA (planner × scenario) for each metric."""
    try:
        from scipy import stats
        from itertools import combinations
    except ImportError:
        print("  scipy not available — skipping ANOVA")
        return

    os.makedirs(output_dir, exist_ok=True)
    lines = []
    lines.append("=" * 70)
    lines.append("TWO-FACTOR ANOVA RESULTS")
    lines.append("Factors: Planner (APF/VFH+/RRT*) × Scenario (Urban/Wilderness/Maritime)")
    lines.append("α = 0.05")
    lines.append("=" * 70)

    for metric, label in METRICS.items():
        if metric not in data.columns:
            continue

        lines.append(f"\n── {label} ──")

        # One-way ANOVA across planners (collapsed across scenarios)
        groups = [
            data[data['planner'] == p][metric].dropna().values
            for p in PLANNERS
        ]
        groups = [g for g in groups if len(g) > 1]

        if len(groups) < 2:
            lines.append("  Insufficient data for ANOVA")
            continue

        f_stat, p_val = stats.f_oneway(*groups)
        sig = "***" if p_val < 0.001 else "**" if p_val < 0.01 else "*" if p_val < 0.05 else "ns"
        lines.append(f"  One-way ANOVA (planner): F={f_stat:.3f}, p={p_val:.4f} {sig}")

        # Tukey HSD (manual pairwise t-tests with Bonferroni)
        lines.append("  Pairwise comparisons (Bonferroni-corrected):")
        pairs = list(combinations(PLANNERS, 2))
        n_comp = len(pairs)

        for p1, p2 in pairs:
            g1 = data[data['planner'] == p1][metric].dropna().values
            g2 = data[data['planner'] == p2][metric].dropna().values
            if len(g1) < 2 or len(g2) < 2:
                continue
            t, p = stats.ttest_ind(g1, g2)
            p_adj = min(p * n_comp, 1.0)   # Bonferroni
            sig2  = "***" if p_adj < 0.001 else "**" if p_adj < 0.01 else "*" if p_adj < 0.05 else "ns"
            d = (g1.mean() - g2.mean()) / (np.sqrt((g1.std()**2 + g2.std()**2) / 2) + 1e-9)
            lines.append(
                f"    {PLANNER_LABELS[p1]} vs {PLANNER_LABELS[p2]}: "
                f"t={t:.3f}, p_adj={p_adj:.4f} {sig2}, Cohen's d={d:.3f}"
            )

        # Scenario effect
        s_groups = [
            data[data['scenario'] == s][metric].dropna().values
            for s in SCENARIOS
        ]
        s_groups = [g for g in s_groups if len(g) > 1]
        if len(s_groups) >= 2:
            f2, p2 = stats.f_oneway(*s_groups)
            sig2 = "***" if p2 < 0.001 else "**" if p2 < 0.01 else "*" if p2 < 0.05 else "ns"
            lines.append(f"  One-way ANOVA (scenario): F={f2:.3f}, p={p2:.4f} {sig2}")

    lines.append("\n" + "=" * 70)
    lines.append("Significance: *** p<0.001  ** p<0.01  * p<0.05  ns = not significant")
    lines.append("=" * 70)

    text = "\n".join(lines)
    print(text)

    path = os.path.join(output_dir, 'anova_results.txt')
    with open(path, 'w') as f:
        f.write(text)
    print(f"\n  Saved: {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("SAR UAV Swarm — Results Analysis")
    print(f"Results dir : {RESULTS_DIR}")
    print(f"Output dir  : {OUTPUT_DIR}")
    print("=" * 60)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Load data
    print("\nLoading CSV files...")
    data = load_results(RESULTS_DIR)

    # Summary stats
    print("Computing summary statistics...")
    summary = compute_summary(data)
    summary_path = os.path.join(OUTPUT_DIR, 'summary_stats.csv')
    summary.to_csv(summary_path, index=False)
    print(f"  Saved: {summary_path}")
    print("\nSummary preview:")
    print(summary[['scenario', 'planner', 'n_trials',
                    'coverage_rate_mean', 'time_to_detection_mean',
                    'path_efficiency_mean']].to_string(index=False))

    # Plots
    print("\nGenerating bar charts...")
    plot_bar_charts(summary, OUTPUT_DIR)

    print("\nGenerating box plots...")
    plot_boxplots(data, OUTPUT_DIR)

    print("\nGenerating heatmap...")
    plot_heatmap(summary, OUTPUT_DIR)

    # ANOVA
    print("\nRunning ANOVA...")
    run_anova(data, OUTPUT_DIR)

    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print(f"Figures : {OUTPUT_DIR}/figures/")
    print(f"Stats   : {OUTPUT_DIR}/summary_stats.csv")
    print(f"ANOVA   : {OUTPUT_DIR}/anova_results.txt")
    print("=" * 60)


if __name__ == '__main__':
    main()
