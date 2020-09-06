from os import walk
import csv
import sys
import numpy as np

# save numpy array as csv file
from numpy import asarray
from numpy import savetxt


# USAGE: provide folder to scan as input. The folder should contain subfolders for different n
# FILENAME: NUMBEROFAGENT_*_SEED.txt

# data about resources here
nagents = [60, 90]

# cycle over folders
for n in nagents:
    # experiments data
    allpops1,allpops2,allpops3 = [],[],[] # all three pops
    allcommitted1,allcommitted2,allcommitted3 = [],[],[] # all three committed
    allquorum1,allquorum2,allquorum3 = [],[],[] # all three quorum
    alluncommitted = []
    alltemp1, alltemp2, alltemp3 = [],[],[] # debug purpose

    # find all files in the folder
    foldername = sys.argv[1]+str(n)
    print(foldername)
    (_, _, filenames) = next(walk(foldername))
    for filename in filenames:
        # run data
        pops1,pops2,pops3 = [],[],[] # all three pops
        committed1,committed2,committed3 = [],[],[] # all three committed
        quorum1,quorum2,quorum3 = [],[],[] # all three quorum
        uncommitted = []
        temp1, temp2, temp3 = [],[],[] # debug purpose

        # open file and get data
        with open(foldername+"/"+filename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=' ')
            for row in csv_reader:
                pops1.append(float(row[0]))
                pops2.append(float(row[3]))
                pops3.append(float(row[6]))
                committed1.append(float(row[1])/float(n))
                committed2.append(float(row[4])/float(n))
                committed3.append(float(row[7])/float(n))
                quorum1.append(float(row[2])/float(n))
                quorum2.append(float(row[5])/float(n))
                quorum3.append(float(row[8])/float(n))
                uncommitted.append(float(row[9])/float(n))
                temp1.append(float(row[10])/float(n))
                temp2.append(float(row[11])/float(n))
                temp3.append(float(row[12])/float(n))

        # append to allarrays
        allpops1.append(pops1)
        allpops2.append(pops2)
        allpops3.append(pops3)
        allcommitted1.append(committed1)
        allcommitted2.append(committed2)
        allcommitted3.append(committed3)
        allquorum1.append(quorum1)
        allquorum2.append(quorum2)
        allquorum3.append(quorum3)
        alluncommitted.append(uncommitted)
        alltemp1.append(temp1)
        alltemp2.append(temp2)
        alltemp3.append(temp3)
        break

    ###### average and save ######
    avpops1 = np.mean(allpops1, axis=0)
    avpops2 = np.mean(allpops2, axis=0)
    avpops3 = np.mean(allpops3, axis=0)
    avcommitted1 = np.mean(allcommitted1, axis=0)
    avcommitted2 = np.mean(allcommitted2, axis=0)
    avcommitted3 = np.mean(allcommitted3, axis=0)
    avquorum1 = np.mean(allquorum1, axis=0)
    avquorum2 = np.mean(allquorum2, axis=0)
    avquorum3 = np.mean(allquorum3, axis=0)
    avuncommitted = np.mean(alluncommitted, axis=0)
    avtemp1 = np.mean(alltemp1, axis=0)
    avtemp2 = np.mean(alltemp2, axis=0)
    avtemp3 = np.mean(alltemp3, axis=0)

    f = open(str(n)+'.csv','w')
    for i in range(len(avpops1)):
        f.write(str(i)+' '+
                str(avpops1[i])+' '+str(avcommitted1[i])+' '+str(avquorum1[i])+
                str(avpops2[i])+' '+str(avcommitted2[i])+' '+str(avquorum2[i])+
                str(avpops3[i])+' '+str(avcommitted3[i])+' '+str(avquorum3[i])+
                str(avuncommitted[i]) + ' '
        )
    f.close()

    ##### plot #####
    import matplotlib.pyplot as plt
    colors = ["r","g","b"]
    plt.plot(avpops1, label='pop1', color=colors[0])
    plt.plot(avpops2, label='pop2', color=colors[1])
    plt.plot(avpops3, label='pop3', color=colors[2])
    plt.plot(avcommitted1, label='comm1', linestyle='--', color=colors[0])
    plt.plot(avcommitted2, label='comm2', linestyle='--',color=colors[1])
    plt.plot(avcommitted3, label='comm3', linestyle='--', color=colors[2])
    # plt.plot(avquorum1, label='quorum1', linestyle=':', color=colors[0])
    # plt.plot(avquorum2, label='quorum2', linestyle=':', color=colors[1])
    # plt.plot(avquorum3, label='quroum3', linestyle=':', color=colors[2])
    plt.plot(avuncommitted, label='uncommitted', color='black')
    # plt.plot(avtemp1, label='temp', linestyle=':', color=colors[0])
    # plt.plot(avtemp2, label='temp', linestyle=':', color=colors[1])
    # plt.plot(avtemp3, label='temp', linestyle=':', color=colors[2])
    plt.legend()

    plt.show()
