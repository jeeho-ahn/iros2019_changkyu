#!/usr/bin/env python3
import argparse
import pandas as pd
import os
import re

def clean_key(key: str) -> str:
    """
    Convert keys like 'planning_time(s)' to 'planning_time_s'
    and 'total_length(m)' to 'total_length_m', etc.
    """
    key = key.replace('(s)', '_s').replace('(m)', '_m')
    return key

def parse_file(input_path: str):
    """
    Parse the input .txt file into a list of records.
    Each block delimited by lines starting with '===' is one record.
    """
    records = []
    current = {}
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
                    ck = clean_key(key)
                    current[ck] = float(val)
        if current:
            records.append(current)
    return records

def main():
    parser = argparse.ArgumentParser(description="Process path analysis data and output Excel summary.")
    parser.add_argument('input_file', help="Path to the .txt file containing raw run data")
    args = parser.parse_args()

    # Parse data
    records = parse_file(args.input_file)
    df = pd.DataFrame(records)

    # Filter out zero total_length runs
    df_valid = df[df['total_length_m'] > 0]
    excluded = len(df) - len(df_valid)
    included = len(df_valid)

    # Compute statistics
    stats = {
        'planning_time_s': {
            'mean': df_valid['planning_time_s'].mean(),
            'std': df_valid['planning_time_s'].std(ddof=1)
        },
        'total_length_m': {
            'mean': df_valid['total_length_m'].mean(),
            'std': df_valid['total_length_m'].std(ddof=1)
        },
        'pushing_length_m': {
            'mean': df_valid.get('pushing_length_m', df_valid['transfer_length_m']).mean(),
            'std': df_valid.get('pushing_length_m', df_valid['transfer_length_m']).std(ddof=1)
        }
    }

    # Prepare DataFrames
    summary_df = pd.DataFrame([{
        'included_runs': included,
        'excluded_runs': excluded
    }])
    analysis_df = pd.DataFrame([
        {'metric': m, 'mean': v['mean'], 'std': v['std']}
        for m, v in stats.items()
    ])

    # Prepare output path
    base, _ = os.path.splitext(args.input_file)
    output_file = base + '.xlsx'

    # Write to Excel: raw_data sheet + combined analysis sheet
    with pd.ExcelWriter(output_file, engine='xlsxwriter') as writer:
        df.to_excel(writer, sheet_name='raw_data', index=False)
        # Write summary and analysis into a single sheet called "analysis"
        summary_df.to_excel(writer, sheet_name='analysis', index=False, startrow=0)
        analysis_df.to_excel(writer, sheet_name='analysis', index=False, startrow=summary_df.shape[0] + 2)

    # Print summary & analysis
    print(summary_df.to_string(index=False))
    print("\nAnalysis metrics:")
    print(analysis_df.to_string(index=False))

if __name__ == "__main__":
    main()
