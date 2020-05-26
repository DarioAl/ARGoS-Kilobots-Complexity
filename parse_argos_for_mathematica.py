# use this file to generate averages to plug in the mathematica file

from os import walk
import csv
import sys
import numpy as np
# save numpy array as csv file
from numpy import asarray
from numpy import savetxt

max_population = 1

def main(argv):
    # load the datasets
    data = []

    # find all folders
    foldernames = []
    (_, foldernames, _) = next(walk(sys.argv[1]))
    for foldername in sorted(foldernames):
        # find all files
        folder_data = []
        (_, _, filenames) = next(walk(sys.argv[1]+foldername))
        for filename in filenames:
            print(filename)
            file_data = []
            with open(sys.argv[1]+foldername+"/"+filename) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=' ')
                line_count = 0
                for row in csv_reader:
                    # skip first line
                    if line_count == 0:
                        file_data.append(1)
                    # read until x-th step
                    # elif line_count == 150:
                        # break
                    else:
                        # 0 for real values 1 for average estimate
                        result = float(row[0])
                        file_data.append(result)
                    line_count = line_count+1
            folder_data.append(file_data)
        ###### average ######
        averaged = np.mean(folder_data, axis=0)
        f = open(foldername+'.csv','w')
        for i in range(len(averaged)):
            f.write(str(i)+' '+str(averaged[i])+'\n')
        f.close()
        ###### original ######
        print(len(folder_data[0]))
        f = open(foldername+'_all.csv','w')
        ffd = np.array(folder_data).flatten()
        for i in range(len(ffd)):
            f.write(str(i)+' '+str(ffd[i])+'\n')
        f.close()

if __name__ == "__main__":
    main(sys.argv)
