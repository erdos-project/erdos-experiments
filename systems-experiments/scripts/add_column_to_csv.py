#!/usr/bin/env python3
import argparse
import pandas as pd

def add_column(input_filename, output_filename, column, value):
    df = pd.read_csv(input_filename)
    df[column] = value
    df.to_csv(output_filename)

def main():
    parser = argparse.ArgumentParser(description="Add a column to a CSV file.")
    parser.add_argument("-i", "--input", help="path to input file", required=True)
    parser.add_argument("-o", "--output", help="path to output file", required=True)
    parser.add_argument("-c", "--column", help="new column name", required=True)
    parser.add_argument("-v", "--value", help="value to populate new column", required=True)

    args = parser.parse_args()

    add_column(args.input, args.output, args.column, args.value)


if __name__ == "__main__":
    main()
