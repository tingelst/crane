import scipy
import scipy.io as spio
import numpy as np
import pickle


def loadmat(filename):
    '''
    this function should be called instead of direct spio.loadmat
    as it cures the problem of not properly recovering python dictionaries
    from mat files. It calls the function check keys to cure all entries
    which are still mat-objects
    '''
    def _check_keys(d):
        '''
        checks if entries in dictionary are mat-objects. If yes
        todict is called to change them to nested dictionaries
        '''
        for key in d:
            if isinstance(d[key], spio.matlab.mio5_params.mat_struct):
                d[key] = _todict(d[key])
        return d

    def _todict(matobj):
        '''
        A recursive function which constructs from matobjects nested dictionaries
        '''
        d = {}
        for strg in matobj._fieldnames:
            elem = matobj.__dict__[strg]
            if isinstance(elem, spio.matlab.mio5_params.mat_struct):
                d[strg] = _todict(elem)
            elif isinstance(elem, np.ndarray):
                d[strg] = _tolist(elem)
            else:
                d[strg] = elem
        return d

    def _tolist(ndarray):
        '''
        A recursive function which constructs lists from cellarrays
        (which are loaded as numpy ndarrays), recursing into the elements
        if they contain matobjects.
        '''
        elem_list = []
        for sub_elem in ndarray:
            if isinstance(sub_elem, spio.matlab.mio5_params.mat_struct):
                elem_list.append(_todict(sub_elem))
            elif isinstance(sub_elem, np.ndarray):
                elem_list.append(_tolist(sub_elem))
            else:
                elem_list.append(sub_elem)
        return elem_list
    data = scipy.io.loadmat(filename, struct_as_record=False, squeeze_me=True)
    return _check_keys(data)


def main():
    filenames = ['gamma', 'k', 'L', 'last_g', 'last_gopt', 't', 'z', 'zref']
    dict_ = {}
    for filename in filenames:
        mat = loadmat(filename + '.mat')
        data = np.array(mat[filename + "_in"]['Data'])
        dict_[filename] = data
    f = open("datain.pickle", "wb")
    pickle.dump(dict_, f)
    f.close()

    filenames = ['dot_wx', 'dot_wy', 'gx', 'gy']
    dict_ = {}
    for filename in filenames:
        mat = loadmat(filename + '.mat')
        data = np.array(mat[filename]['Data'])
        dict_[filename] = data
    f = open("dataout.pickle", "wb")
    pickle.dump(dict_, f)
    f.close()



if __name__ == "__main__":
    main()
