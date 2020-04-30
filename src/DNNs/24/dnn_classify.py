import numpy as np
# Solution found at https://answers.ros.org/question/289855/import-tensorflow-in-ros-kinetic/ by Oscar Pang
import tensorflow as tf
import scipy.io as spio
import keras

#Load data from matlab file
mat = spio.loadmat('dnn_data.mat', squeeze_me=True)
fc_1_params = [mat['fc_1_w'].T, mat['fc_1_b']]
fc_2_params = [mat['fc_2_w'].T, mat['fc_2_b']]
fc_3_params = [mat['fc_3_w'].T, mat['fc_3_b']]
bn_1_params = [mat['bn_1_scale'], mat['bn_1_offset'], mat['bn_1_mean'], mat['bn_1_variance']]
bn_2_params = [mat['bn_2_scale'], mat['bn_2_offset'], mat['bn_2_mean'], mat['bn_2_variance']]
#systems = mat['systems']

#Defining layers
fc_layer_1 = tf.keras.layers.Dense(np.array(fc_1_params[0]).shape[1],input_shape=(1,np.array(fc_1_params[0]).shape[0]))
relu_1 = tf.keras.layers.ReLU()
batchnorm_1 = tf.keras.layers.BatchNormalization()
dropout_1 = tf.keras.layers.Dropout(rate=0.4)
fc_layer_2 = tf.keras.layers.Dense(np.array(fc_2_params[0]).shape[1])
relu_2 = tf.keras.layers.ReLU()
batchnorm_2 = tf.keras.layers.BatchNormalization()
dropout_2 = tf.keras.layers.Dropout(rate=0.4)
final_layer = tf.keras.layers.Dense(np.array(fc_3_params[0]).shape[1])
softmax = tf.keras.layers.Softmax()

#Adding layers to model
nn_model = tf.keras.models.Sequential()
nn_model.add(fc_layer_1)
nn_model.add(relu_1)
nn_model.add(batchnorm_1)
nn_model.add(dropout_1)
nn_model.add(fc_layer_2)
nn_model.add(relu_2)
nn_model.add(batchnorm_2)
nn_model.add(dropout_2)
nn_model.add(final_layer)
nn_model.add(softmax)

#Compiling the model
nn_model.compile(loss='mse', optimizer='adam')

#Updating the parameters with the ones from the trained network
nn_model_fc_1 = nn_model.layers[0]
nn_model_fc_2 = nn_model.layers[4]
nn_model_fc_3 = nn_model.layers[8]
nn_model_bn_1 = nn_model.layers[2]
nn_model_bn_2 = nn_model.layers[6]

nn_model_fc_1.set_weights(fc_1_params)
nn_model_fc_2.set_weights(fc_2_params)
nn_model_fc_3.set_weights(fc_3_params)
nn_model_bn_1.set_weights(bn_1_params)
nn_model_bn_2.set_weights(bn_2_params)

nn_model.save('model.h5')
print("MODEL SAVED TO model.h5")

# save to csv file
#np.savetxt('systems_truth_table.csv', systems, delimiter=',')
#print("Systems Truth Table saved to systems_truth_table.csv")

        

