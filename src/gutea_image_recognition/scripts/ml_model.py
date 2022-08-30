#!/home/sbc-07/catkin_ws/src/gutea_image_recognition/scripts/venv/bin/python
import keras
import numpy as np
from keras.applications import mobilenetv2
from keras.models import load_model
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
from scipy.misc import imresize
import threading
print("BEGIN: model.py")
path = '/home/sbc-07/catkin_ws/src/gutea_image_recognition'
# DEBUG
# filename = '/tmp/cat.jpg'


# load models
print("THREAD (GLOBAL):", threading.get_ident())
print("Loading pretrained...")
pretrained_model = mobilenetv2.MobileNetV2(weights='imagenet', include_top=False, input_shape=(224, 224, 3))
pretrained_model._make_predict_function()
print("Loading back model...")
model = load_model(path + '/model/my_model.h5')
# model._make_predict_function()
print("Loaded!")


def run_classification(input_image):
    #global pretrained_model
    #global model

    print("THREAD (CALLBACK):", threading.get_ident())
    # process image
    # original = load_img(filename, target_size=(224, 224)) #DEBUG
    # numpy_image = img_to_array(original) #DEBUG
    numpy_image = imresize(input_image / 255.0, (224,224,3)).astype(np.float32)
    print(numpy_image.dtype, numpy_image.shape)
    image_batch = np.expand_dims(numpy_image, axis=0)
    processed_image = mobilenetv2.preprocess_input(image_batch.copy())

    # get features through pretrained model
    features = pretrained_model.predict(processed_image)
    features = np.reshape(features, (1, np.prod(features.shape)))
    print(features, features.shape)
    
    # apply finetuning
    prob = model.predict(features)[0]
    order = [4,2,1,0,3,5]
    ordered_prob = [prob[i] for i in order]
    
    # print results
    labels = ['male_person', 'female_person', 'dog', 'cat', 'luggage', 'stroller']
    for l, p in zip(labels, ordered_prob):
        print("{:13} : {:2.2f}".format(l, p))

    return labels[np.argmax(ordered_prob)]

print("END: model.py")
