#show_pkl.py
 
import joblib
# path='/home/mycode/ASAP/humanoidverse/data/motions/g1_29dof_anneal_23dof/TairanTestbed/singles/0-TairanTestbed_TairanTestbed_CR7_video_CR7_level1_filter_amass.pkl'   #path='/root/……/aus_openface.pkl'   pkl文件所在路径
# path = '/home/mycode/omnih2o/data/h1/amass_all.pkl'
# path = '/home/mycode/omnih2o/legged_gym/resources/motions/h1/amass_phc_filtered.pkl'

# path = '/home/mycode/ASAP/humanoidverse/data/motions/g1_23to21/TairanTestbed/singles/amass_all_23.pkl'
# path ='/home/mycode/omnih2o/legged_gym/resources/motions/g1/amass_all.pkl'
# path = '/home/mycode/FALCON/humanoidverse/data/motions/g1_29dof/v1/accad_all.pkl'

# path = '/home/mycode/ASAP/humanoidverse/data/motions/g1_29dof_anneal_23dof/TairanTestbed/singles/0-TairanTestbed_TairanTestbed_CR7_video_CR7_level1_filter_amass.pkl'
# path = '/home/mycode/ASAP/humanoidverse/data/motions/g1_23to21/TairanTestbed/singles/amass_all_23_360zhuanti.pkl'
path = '/home/mycode/ASAP/humanoidverse/data/motions/q1/amass_all.pkl'
# path = '/home/mycode/data/LAFAN1_Visualize/pkl_data/h1/dance1_subject2.pkl'
# path = '/home/mycode/ASAP/humanoidverse/data/motions/q1/amass_all.pkl'
# path = '/home/mycode/omnih2o/data/q1/amass_all.pkl'
# path = '/home/mycode/omnih2o/data/g1/amass_all.pkl'
# path = '/home/mycode/omnih2o/data/g1_23/amass_all_23.pkl'
# path = '/home/mycode/omnih2o/data/h1/amass_all.pkl'

f=open(path,'rb')
data=joblib.load(f)
# print(data)


for i,j in data.items():
    # print(i,j)
    for x,y in j.items():
        try:
            print(x,y.shape)
        except:
            print("=============\n",x,y)
    print(i)
 
print("pkl type: ",type(data))
