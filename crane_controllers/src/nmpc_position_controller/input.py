import pickle

def load():
    f = open(
        '/home/lars/crane_ws/src/crane/crane_controllers/data/datain.pickle', 'rb')
    datain = pickle.load(f)
    f.close()
    f = open(
        '/home/lars/crane_ws/src/crane/crane_controllers/data/dataout.pickle', 'rb')
    dataout = pickle.load(f)
    f.close()
    return datain, dataout