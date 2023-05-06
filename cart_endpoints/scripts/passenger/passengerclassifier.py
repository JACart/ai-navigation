import sklearn
import numpy as np
import pandas as pd
import joblib
import os
from sklearn.model_selection import train_test_split, cross_val_score, StratifiedKFold
from sklearn.metrics import accuracy_score
from sklearn.ensemble import RandomForestClassifier
from sklearn.decomposition import PCA
from sklearn import tree
import matplotlib.pyplot as plt
'''
Passenger ML classifier, data processing, and visualization.

Author: Daniel Hassler
Version: 4/2/2023
'''

class PassengerData(object):
    '''
    PassengerData class to preprocess the data. Used in the classifier.
    '''
    def __init__(self, in_bounds_X_data, out_bounds_X_data, in_bounds_y_data, out_bounds_y_data):
        '''
        Constructor that sets model data attributes.
        '''
        self.in_bounds_X_data = in_bounds_X_data
        self.out_bounds_X_data = out_bounds_X_data
        self.in_bounds_y_data = in_bounds_y_data
        self.out_bounds_y_data = out_bounds_y_data
    
    def preprocess(self, test_size=0.2):
        '''
        Preprocess the data for training and test split.
        '''
        self.X = np.concatenate(
            (self.in_bounds_X_data, self.out_bounds_X_data), axis=0)
        self.y = np.concatenate(
            (self.in_bounds_y_data, self.out_bounds_y_data), axis=0)
        try:
            self.X = np.reshape(self.X, (self.X.shape[0], (18 * 3)))
        except:
            exit("The X_data for in_bounds and out_bounds must be of shape (num_entries, 18, 3). Refer to the skeleton_3d points for the ZED cameras.")
        
        self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(self.X, self.y, test_size=test_size)

        return self.X_train, self.X_test, self.y_train, self.y_test
    
    def save_data(self, version):
        '''
        Saves the current data object into respective .npy files.

        Parameter:
        - Version: a version number for the data files.
        '''
        with open(f'./passenger_ML_data/in_bounds_X_data{version}.npy', 'wb') as f:
            np.save(f, self.in_bounds_X_data)
        with open(f'./passenger_ML_data/in_bounds_y_data{version}.npy', 'wb') as f:
            np.save(f, self.in_bounds_y_data)
        with open(f'./passenger_ML_data/out_bounds_X_data{version}.npy', 'wb') as f:
            np.save(f, self.out_bounds_X_data)
        with open(f'./passenger_ML_data/out_bounds_y_data{version}.npy', 'wb') as f:
            np.save(f, self.out_bounds_y_data)


class PassengerRFClassifier(RandomForestClassifier):
    '''
    Passenger RandomForestClassifier wrapper class for ease of access and use.
    '''
    def __init__(self, pdata=PassengerData(np.load("./passenger_ML_data/in_bounds_X_datav04172023.npy"),
        np.load("./passenger_ML_data/out_bounds_X_datav04172023.npy"),
        np.load("./passenger_ML_data/in_bounds_y_datav04172023.npy"),
        np.load("./passenger_ML_data/out_bounds_y_datav04172023.npy")), n_estimators=5, n_jobs=5):
        '''
        This constructor takes in three optional parameters:

        Parameters:
            - pdata (PassengerData) [default = data object with model data in ./passenger_ML_data]
            - n_estimators (int) [default = 5]
            - n_jobs (int) [default = 5]
        '''
        self.pdata = pdata
        self.n_estimators = n_estimators
        self.n_jobs = n_jobs

        super().__init__(n_estimators=self.n_estimators, max_depth=2, criterion="entropy", n_jobs=self.n_jobs)

        self.X_train, self.X_test, self.y_train, self.y_test = self.pdata.preprocess()
    
    def save_model(self, fpath):
        '''
        Wrapper function to save model.
        '''
        file = f"{fpath}.joblib"
        joblib.dump(self, file)
    
    def load_model(fpath):
        '''
        Wrapper function to load model.
        '''
        print(f"Loaded model: {fpath}")
        return joblib.load(f"{fpath}.joblib")
    
    def save_and_test_models(start, end):
        '''
        Function to save a bunch of models and figure out
        which one is best for the current dataset.
        '''
        if not os.path.exists("./saved_models"):
            os.mkdir("./saved_models")

        file = open("./saved_models/results.txt", "w")

        for i in range(start, end, 2):
            saved_model_name = f"./saved_models/passengerRF_model_{i}"
            prfc = PassengerRFClassifier(n_estimators=i)
            prfc.fit(prfc.X_train, prfc.y_train)
            y_pred = prfc.predict(prfc.X_test)

            cross_val = cross_val_score(prfc, prfc.pdata.X, prfc.pdata.y, cv=StratifiedKFold(n_splits=5, shuffle=True)).mean()
            accuracy = accuracy_score(prfc.y_test, y_pred)

            file.write(
                f"passengerRF_model_{i},{cross_val},{accuracy}\n")
        
            prfc.save_model(saved_model_name)
            
        file.close()
        modelsdf = pd.read_csv("./saved_models/results.txt", header=None, index_col=0)
        modelsdf = modelsdf.sort_values(by=[1, 2]).iloc[-1]
        highest_val_acc = modelsdf[1]
        highest_acc = modelsdf[2]
        highest_name = modelsdf.name
        print(f"The best model generated was:\n\tName: {highest_name}\n\tValidation Acc %: {highest_val_acc}\n\tAcc %: {highest_acc}")
        return f"./saved_models/{highest_name}"


class PlotPassenger():
    '''
    Class designed to visualize the PassengerRFClassifier with various metrics and plots.
    '''
    def __init__(self, prfc):
        self.prfc = prfc

    def plot_PCA(self, n_dim):
        '''
        Runs Principal Component Analysis on the dataset for visualization purposes.
        '''
        if n_dim < 2 or n_dim > 3:
            print("n_dim must be 2 or 3, defaulting to n_dim=2")
            n_dim = 2

        pca = PCA(n_components=n_dim)
        X_p = pca.fit_transform(self.prfc.pdata.X)
        if n_dim == 2:
            plot = plt.scatter(X_p[:,0], X_p[:,1], c=self.prfc.pdata.y)
            plt.legend(handles=plot.legend_elements()[0], labels=["Unafe", "Safe"])
        else:
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.scatter(X_p[:,0], X_p[:,1], X_p[:,2], c=self.prfc.pdata.y)

        plt.title(f"Passenger Pose Dataset Where principal_components={n_dim}")
        plt.xlabel("PC1")
        plt.ylabel("PC2")

        plt.show()

    def plot_random_forest_trees(self):
        '''
        Plots out all the decision trees used in the ensemble learning for the RandomForestClassifier.
        '''
        fig, axes = plt.subplots(nrows = 1,ncols = self.prfc.n_estimators,figsize = (10,3), dpi=250)
        for index in range(self.prfc.n_estimators):
            tree.plot_tree(self.prfc.estimators_[index],
                        feature_names = [f"f{i}" for i in range(54)], 
                        class_names= ["Unsafe", "Safe"],
                        filled = True,
                        ax = axes[index])

            axes[index].set_title('Estimator: ' + str(index + 1), fontsize = 10)
        fig.savefig(f'rf_{self.prfc.n_estimators}trees.png')
        plt.show()

    def plot_skeleton(self, passenger_ind=0):
        '''
        Plots out a skeleton_3d entry in the X_train dataset given an index for the X_train dataset.
        '''
        person = self.prfc.X_train[passenger_ind].reshape(18, 3)
        ax = plt.axes(projection='3d')
        connections = [[0, 1], [1, 2], [1, 5], [1, 11], 
                    [1, 8], [2, 3], [3, 4], [5, 6], 
                    [6, 7], [8, 9], [9, 10], [11, 12], 
                    [12, 13], [0, 14], [14, 16], 
                    [0, 15], [15, 17]]

        for connection in connections:
            joint_one_ind = connection[0]
            joint_two_ind = connection[1]
            joint_one = person[joint_one_ind]
            joint_two = person[joint_two_ind]
            ax.plot([joint_one[0], joint_two[0]],[joint_one[1], joint_two[1]],[joint_one[2], joint_two[2]], color="blue")

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.view_init(elev=5, azim=178)
        plt.show()

if __name__ == "__main__":
    best_model = PassengerRFClassifier.save_and_test_models(101,131)
    prfc = PassengerRFClassifier.load_model(best_model)
    #print(f"The best model generated was:\n\tName: {highest_name}\n\tValidation Acc %: {highest_val_acc}\n\tAcc %: {highest_acc}")