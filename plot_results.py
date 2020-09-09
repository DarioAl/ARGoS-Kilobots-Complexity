from os import walk
import csv
import sys
import numpy as np

# save numpy array as csv file
from numpy import asarray
from numpy import savetxt


# USAGE: provide folder to scan as input. The folder should contain subfolders for different n
# FILENAME: NUMBEROFAGENT_*_SEED.txt

def readData(f):
    # experiments data
    allpops1,allpops2,allpops3 = [],[],[] # all three pops
    allcommitted1,allcommitted2,allcommitted3 = [],[],[] # all three committed
    allquorum1,allquorum2,allquorum3 = [],[],[] # all three quorum
    alluncommitted = []
    alltemp1, alltemp2, alltemp3 = [],[],[] # debug purpose
    alluts = []

    # find all files in the folder
    foldername = sys.argv[1]+f
    print(foldername)
    (_, _, filenames) = next(walk(foldername))
    for filename in filenames:
        # run data
        pops1,pops2,pops3 = [],[],[] # all three pops
        committed1,committed2,committed3 = [],[],[] # all three committed
        quorum1,quorum2,quorum3 = [],[],[] # all three quorum
        uncommitted = []
        temp1, temp2, temp3 = [],[],[] # debug purpose
        uts = []

        # open file and get data
        with open(foldername+"/"+filename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=' ')
            for row in csv_reader:
                pops1.append(float(row[0]))
                pops2.append(float(row[3]))
                pops3.append(float(row[6]))
                committed1.append(float(row[1])/90.0)
                committed2.append(float(row[4])/90.0)
                committed3.append(float(row[7])/90.0)
                quorum1.append(float(row[2])/90.0)
                quorum2.append(float(row[5])/90.0)
                quorum3.append(float(row[8])/90.0)
                uncommitted.append(float(row[9])/90.0)
                temp1.append(float(row[10])/90.0)
                temp2.append(float(row[11])/90.0)
                temp3.append(float(row[12])/90.0)
                uts.append(float(row[13]))

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
        alluts.append(uts)

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
    avtemp2 = np.std(alltemp1, axis=0)
    avtemp3 = np.mean(alltemp3, axis=0)
    alluts = np.mean(alluts, axis=0)
    return [avpops1, avpops2, avpops3], [avcommitted1, avcommitted2, avcommitted3], [avquorum1, avquorum2, avquorum3], avuncommitted,  [avtemp1, avtemp2, avtemp3], alluts[-1]



##################################################
## data about resources here
folders = ["90","303030","60300"]
folders = ["90nunq","90nuq","90unq","90uq"]

titles = ["No Rescaling, No Quorum", "No Rescaling, With Quorum", "With Rescaling, No Quorum", "With Rescaling, With Quorum"]
##### plot #####
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx

# colormap
colormap = cm = plt.get_cmap('viridis')
colors = [colormap(0.1), colormap(0.5), colormap(0.9)]

# latex
from matplotlib import rc
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)
plt.rc('xtick',labelsize=18)
plt.rc('ytick',labelsize=18)

# sub plots
fig, axs = plt.subplots(2, 2,figsize=(15, 8), dpi=200)

ax = axs.flat
utilities = []
# cycle over folders
for i in range(len(folders)):
    allpops, allcommitted, allquorum, alluncommitted, alltemp, sums= readData(folders[i])
    utilities.append(sums)

    # plot lines
    ax[i].plot(allpops[0], label=r'\nu_1', color=colors[0])
    ax[i].plot(allpops[1], label=r'\nu_2', color=colors[1])
    ax[i].plot(allpops[2], label=r'\nu_3', color=colors[2])
    # ax[i].plot(allcommitted[0], label=r'n_1', linestyle=':', color=colors[0])
    # ax[i].plot(allcommitted[1], label=r'n_2', linestyle=':',color=colors[1])
    # ax[i].plot(allcommitted[2], label=r'n_3', linestyle=':', color=colors[2])
    # ax[i].plot(allquorum1[0][0], label='quorum1', linestyle=':', color=colors[0])
    # ax[i].plot(allquorum2[0][0], label='quorum2', linestyle=':', color=colors[1])
    # ax[i]plot(allquorum3[0][0], label='quroum3', linestyle=':', color=colors[2])
    # ax[i].plot(alluncommitted, label=r'u', linestyle=':', color='grey')

    ##### use this for estimation variation when you have more runs
    # ax[i].plot(alltemp[0], label='temp', linestyle=':', color=colors[2])
    # ax[i].fill_between(alltemp[0], alltemp[0]-alltemp[1], alltemp[0]+alltemp[1] ,alpha=0.3, facecolor=colors[2])

    # ax[i].plot(alltemp[0], label='temp', linestyle=':', color=colors[0])
    # ax[i].plot(alltemp[1], label='temp', linestyle=':', color=colors[1])
    # ax[i].plot(alltemp[2], label='temp', linestyle=':', color=colors[2])

    # labels tuning
    ax[i].set_title(titles[i], fontsize = 25);
    ax[i].set_ylabel(r"\nu_i", fontsize = 25)
    ax[i].set_xlabel(" time (s)", fontsize = 25)
    ax[i].label_outer()
    ax[i].set_xlim(0,7200)

fig.subplots_adjust(hspace=0.25)
fig.subplots_adjust(wspace=0.05)
# legend
ax[1].legend(loc='upper right', fontsize = 18)

plt.savefig("lines.pdf", format='pdf', bbox_inches = 'tight')
