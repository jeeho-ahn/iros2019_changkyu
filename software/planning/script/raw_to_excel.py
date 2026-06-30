#!/usr/bin/env python3
import argparse
import pandas as pd
import os
import re

TIMEOUT_SEC = 1200.0  # threshold for timeout

def clean_key(key: str) -> str:
    return key.replace('(s)', '_s').replace('(m)', '_m')

def parse_file(input_path: str):
    records, current = [], {}
    pattern = re.compile(r'([\w\(\)_]+):\s*([0-9]*\.?[0-9]+)')
    with open(input_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith('==='):
                if current:
                    records.append(current)
                    current = {}
            else:
                for key, val in pattern.findall(line):
                    current[clean_key(key)] = float(val)
    if current:
        records.append(current)
    return records

def main():
    parser = argparse.ArgumentParser(description="Process path analysis data and output Excel summary.")
    parser.add_argument('input_file', help="Path to the .txt file containing raw run data")
    args = parser.parse_args()

    # Parse & DataFrame
    records = parse_file(args.input_file)
    df = pd.DataFrame(records)

    # Add helper flags
    df['is_timeout'] = df['planning_time_s'] > TIMEOUT_SEC
    df['is_zero_len'] = df['total_length_m'] == 0

    # Filters
    df_valid = df[~df['is_zero_len'] & ~df['is_timeout']]

    # Counts
    total_runs   = len(df)
    zero_excl    = int(df['is_zero_len'].sum())
    timeout_excl = int(df['is_timeout'].sum())
    included     = len(df_valid)

    # Decide which length column to use (pushing or transfer)
    push_col = 'pushing_length_m' if 'pushing_length_m' in df_valid.columns else 'transfer_length_m'

    # Statistics
    stats = {
        'planning_time_s': {
            'mean': df_valid['planning_time_s'].mean(),
            'std':  df_valid['planning_time_s'].std(ddof=1)
        },
        'total_length_m': {
            'mean': df_valid['total_length_m'].mean(),
            'std':  df_valid['total_length_m'].std(ddof=1)
        },
        push_col: {
            'mean': df_valid[push_col].mean(),
            'std':  df_valid[push_col].std(ddof=1)
        }
    }

    # DataFrames for Excel
    summary_df = pd.DataFrame([{
        'total_runs': total_runs,
        'included_runs': included,
        'excluded_zero_length': zero_excl,
        'excluded_timeout': timeout_excl,
        'timeout_threshold_s': TIMEOUT_SEC
    }])

    analysis_df = pd.DataFrame([
        {'metric': m, 'mean': v['mean'], 'std': v['std']}
        for m, v in stats.items()
    ])

    # Output path
    base, _ = os.path.splitext(args.input_file)
    output_file = base + '.xlsx'

    with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
        df.to_excel(writer, sheet_name='raw_data', index=False)
        # summary at top, a blank row, then analysis
        summary_df.to_excel(writer, sheet_name='analysis', index=False, startrow=0)
        analysis_df.to_excel(writer, sheet_name='analysis', index=False, startrow=summary_df.shape[0] + 2)

    # Console print
    print(summary_df.to_string(index=False))
    print("\nAnalysis metrics:")
    print(analysis_df.to_string(index=False))

if __name__ == "__main__":
    main()
