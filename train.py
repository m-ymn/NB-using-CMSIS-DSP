from sklearn.naive_bayes import GaussianNB
import random
import numpy as np
import math
# Import LabelEncoder
from sklearn import preprocessing



# Assigning features and label variables
# 'Overcast', 'Rainy', 'Sunny' as 0, 1, 2
wheather = ['Sunny', 'Sunny', 'Overcast', 'Rainy', 'Rainy', 'Rainy', 'Overcast', 'Sunny', 'Sunny',
           'Rainy', 'Sunny', 'Overcast', 'Overcast', 'Rainy']
temp = ['Hot', 'Hot', 'Hot', 'Mild', 'Cool', 'Cool', 'Cool', 'Mild', 'Cool', 'Mild', 'Mild', 'Mild', 'Hot', 'Mild']

play = ['No', 'No', 'Yes', 'Yes', 'Yes', 'No', 'Yes', 'No', 'Yes', 'Yes', 'Yes', 'Yes', 'Yes', 'No']


def fxn():
    # creating labelEncoder
    le = preprocessing.LabelEncoder()
    # Converting string labels into numbers.
    wheather_encoded = le.fit_transform(wheather)
    temp_encoded = le.fit_transform(temp)
    label = le.fit_transform(play)
    # print("wheather:", wheather_encoded)   # 0: Overcast , 1: Rainy , 2: Sunny
    #print("Temp:", temp_encoded)           # 0:Cool 1: HOT , 2: Mild

    #print("Play:", label)           # 0: No  , 1 : Yes

    features = [[el] for el in wheather_encoded ]
    for i in range(len(features)):
        features[i].append(temp_encoded[i])

    #print(features)  # whether, Temperature

    model = GaussianNB()

    # Train the model using the training sets
    model.fit(features, label)
    predicted = model.predict([[2,1]])  # give two values whether,temperature
    #print(model.predict_log_proba([[2,1]]))
    print("Predicted Value:", predicted[0],(lambda x: "No" if x == 0 else "Yes")(predicted[0])+"\n\r")

    # Gaussian averages
    print("Theta = ", list(np.reshape(model.theta_, np.size(model.theta_))))

    # Gaussian variances
    print("Sigma = ", list(np.reshape(model.sigma_, np.size(model.sigma_))))

    # Class priors
    print("Prior = ", list(np.reshape(model.class_prior_, np.size(model.class_prior_))))

    print("Epsilon = ", model.epsilon_)

fxn()