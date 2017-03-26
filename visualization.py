import numpy as np
import matplotlib.pyplot as plt
import argparse
import csv

###
# Simple visulization of kalman filter operations 
###
def main():
    # Set parser for inputs
    parser = argparse.ArgumentParser(description="Processing input arguments")
    parser.add_argument("-d", "--data_file", help="Data file", required=True)
    args = parser.parse_args()

    with open(args.data_file) as input_file:
        data = [line.strip().split('\t') for line in input_file]

    for data_el in data:
        estimation = data_el[0:2]
        measurements = data_el[4:6]
        ground_truth = data_el[6:8]

        x = ground_truth[0]
        y = ground_truth[1]
        plt.scatter(x, y, color='g')

        x = measurements[0]
        y = measurements[1]
        plt.scatter(x, y, color='r')

        x = estimation[0]
        y = estimation[1]
        plt.scatter(x, y, color='b')

    plt.show()


if __name__ == "__main__":
    main()
