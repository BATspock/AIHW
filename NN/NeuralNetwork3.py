import numpy as np 
import pandas as pd
import sys




class MLP(object):
    
    def __init__(self, epochs):
        self.epochs = epochs
        
    def load_data(self, X,Y, T):

        train_data = pd.read_csv(X, header=None)
        train_labels = pd.read_csv(Y, header=None)
        test_data = pd.read_csv(T, header=None)



        return train_data, train_labels, test_data

    def sigmoid(self, X):
        return 1/(1+np.exp(-X))
    
    def categorical_loss(self, X,Y):
        total = np.sum(np.multiply(X, np.log(Y)))
        Loss = (-1/X.shape[1])*total
        
        return Loss
    
    def softmax(self, X):
        return np.exp(X) / np.sum(np.exp(X), axis=0)
    
    def forward(self, X, params)->dict:
        current = dict()

        current["Z1"] = np.matmul(params["W1"], X) + params["b1"]

        current["A1"] = sigmoid(current["Z1"])

        current["Z2"] = np.matmul(params["W2"], current["A1"]) + params["b2"]

        current["A2"] = sigmoid(current["Z2"])

        current["Z3"] = np.matmul(params["W3"], current["A2"]) + params["b3"]

        current["A3"] = self.softmax(current["Z3"])

        return current
    
    def backprop(self, X,Y,para, current)->dict:
        dZ3 = current["A3"] - Y
        dW3 = (1./batchsize) * np.matmul(dZ3, current["A2"].T)
        db3 = (1./batchsize) * np.sum(dZ3, axis=1, keepdims=True)

        dA2 = np.matmul(para["W3"].T, dZ3)
        dZ2 = dA2 * self.sigmoid(current["Z2"]) * (1 - self.sigmoid(current["Z2"]))
        dW2 = (1./batchsize) * np.matmul(dZ2, current["A1"].T)
        db2 = (1./batchsize) * np.sum(dZ2, axis=1, keepdims=True)

        dA1 = np.matmul(para["W2"].T, dZ2)
        dZ1 = dA1 * self.sigmoid(current["Z1"]) * (1 - self.sigmoid(current["Z1"]))
        dW1 = (1./batchsize) * np.matmul(dZ1, X.T)
        db1 = (1./batchsize) * np.sum(dZ1, axis=1, keepdims=True)

        grads = {"dW1": dW1, "db1": db1, "dW2": dW2, "db2": db2, "dW3": dW3, "db3": db3}

        return grad
    
    def train_model(self):
        
        for i in range(self.epochs):
        #using mini batch gradient descent
            permutation = np.random.permutation(x_train.shape[1])
            Xshuffled = x_train[:, permutation]
            Yshuffled = Yonehot[:, permutation]

            for j in range(batches):

                begin = j * batch_size
                end = min(begin + batch_size, x_train.shape[1] - 1)
                X = Xshuffled[:, begin:end]
                Y = Yshuffled[:, begin:end]
                batchsize = end - begin

                current = self.forward(X, parameters)
                gradient = self.backprop(X, Y, parameters, current)

                wtavgdW1 = (beta * wtavgdW1 + (1. - beta) * gradient["dW1"])
                wtavgdb1 = (beta * wtavgdb1 + (1. - beta) * gradient["db1"])
                wtavgdW2 = (beta * wtavgdW2 + (1. - beta) * gradient["dW2"])
                wtavgdb2 = (beta * wtavgdb2 + (1. - beta) * gradient["db2"])
                wtavgdW3 = (beta * wtavgdW3 + (1. - beta) * gradient["dW3"])
                wtavgdb3 = (beta * wtavgdb3 + (1. - beta) * gradient["db3"])

                parameters["W1"] = parameters["W1"] - lr* wtavgdW1
                parameters["b1"] = parameters["b1"] - lr * wtavgdb1
                parameters["W2"] = parameters["W2"] - lr * wtavgdW2
                parameters["b2"] = parameters["b2"] - lr * wtavgdb2
                parameters["W3"] = parameters["W3"] - lr * wtavgdW3
                parameters["b3"] = parameters["b3"] - lr * wtavgdb3
            
        return parameters




parameters = { "W1": np.random.randn(512, 784) * np.sqrt(1. / 784),
           "b1": np.zeros((512, 1)) * np.sqrt(1. / 784),
           "W2": np.random.randn(64, 512) * np.sqrt(1. / 512),
           "b2": np.zeros((64, 1)) * np.sqrt(1. / 512),
           "W3": np.random.randn(10, 64) * np.sqrt(1. / 64),
           "b3": np.zeros((10, 1)) * np.sqrt(1. / 64)}

wtavgdW1 = np.zeros(parameters["W1"].shape)
wtavgdb1 = np.zeros(parameters["b1"].shape)
wtavgdW2 = np.zeros(parameters["W2"].shape)
wtavgdb2 = np.zeros(parameters["b2"].shape)
wtavgdW3 = np.zeros(parameters["W3"].shape)
wtavgdb3 = np.zeros(parameters["b3"].shape)

epochs = 20
lr = 4
beta = .9
batch_size = 32
batches = x_train.shape[0]//batch_size




obj = MLP(epochs)

dataX = sys.argv[1]
dataY = sys.argv[2]
TEST = sys.argv[3]

X , Y, test = obj.load_data(dataX, dataY, TEST)
X, Y, test = np.asarray(X), np.asarray(Y), np.asarray(test)
X = X/255 #normalize
test = test/255

Y = Y.reshape(1, X.shape[0])
Yonehot = np.eye(10)[Y.astype('int32')]
Yonehot = Yonehot.T.reshape(10, X.shape[0])

x_train = X.T
x_test = test.T

parameters = obj.train_model()


out = forward(x_test, parameters)
pred = np.argmax(out["A3"], axis=0)

np.savetxt("test_predictions.csv", pred, delimiter=',', fmt='%d' )