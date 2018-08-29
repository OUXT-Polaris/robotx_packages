#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
reference:
https://qiita.com/yukiB/items/1ea109eceda59b26cd64#4-kerastensorflow%E3%81%A7%E4%BD%9C%E6%88%90%E3%81%97%E3%81%9F%E3%83%A2%E3%83%87%E3%83%AB%E3%82%92c%E3%81%8B%E3%82%89%E5%AE%9F%E8%A1%8C
"""
import tensorflow as tf
from tensorflow.python.framework import graph_util, graph_io
import numpy as np
import cv2
import rospkg


class Predicdion():

    def __init__(self):

        # もし重み(pbファイル)がなければgdriveからダウンロードする
        # ../data/trained_weight.jsonに最新の重みのurlを記載する
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('robotx_recognition')
        filename = pkg_path + '/data/trained_model.pb'
        settingFileName = pkg_path + '/data/trained_model.json'

        import json
        import os

        with open(settingFileName, 'r') as f:
            datasetInfo = json.load(f)

        if not os.path.isfile(filename):
            print("downloading weight file...")
            import urllib.request
            #print(datasetInfo['url'])
            #print(filename)
            urllib.request.urlretrieve(datasetInfo['url'], filename)
            # import urllib
            # urllib.urlretrieve(datasetInfo['url'], filename)
            print("file download finished!")

        self.graph = tf.Graph()
        graph_def = tf.GraphDef()

        with open(filename, 'rb') as f:
            graph_def.ParseFromString(f.read())
        with self.graph.as_default():
            tf.import_graph_def(graph_def)

        self.inLayer    = self.graph.get_operation_by_name('import/conv2d_1_input')
        self.learnPhase = self.graph.get_operation_by_name('import/dropout_1/keras_learning_phase')
        self.outLayer   = self.graph.get_operation_by_name('import/output0')
        self.sess = tf.Session(graph=self.graph)


    def __call__(self, image):

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.
        image = cv2.resize(image, (128,128))
        image = image.reshape((1,128,128,3))

        y_pred = self.sess.run(self.outLayer.outputs[0],
                          {self.inLayer.outputs[0]: image,
                           self.learnPhase.outputs[0]: 0})
        return y_pred[0]
