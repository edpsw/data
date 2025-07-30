import numpy as np
def load_data(csv_files):


    data = np.genfromtxt(csv_files, delimiter=',')
    print(csv_files)
    print(data.shape)
    
    # return data


load_data('/home/z/code/data/LAFAN1_Visualize/g1/dance1_subject2.csv')


load_data('/home/z/code/data/LAFAN1_Visualize/h1/dance1_subject2.csv')
load_data('/home/z/code/data/LAFAN1_Visualize/h1_2/dance1_subject2.csv')