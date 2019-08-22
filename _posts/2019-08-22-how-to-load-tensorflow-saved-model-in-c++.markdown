---
layout: post
title:  "How to Load Tensorflow Saved Model"
date:   2019-08-22 13:31:00
categories: tensorflow
brief: "Tensorflow offical website only have partial example. In this tutorial I have a full example and explan everything detail"
---

# Prerequisite
* Ubuntu 16.04
* Installed Tensorflow C++ Library r1.13
* A Tensorflow Saved Model
  
# Overview
The Tensorflow C++ API is some-how under documented. In this tutorial I'll show you how to load a tensorflow saved model in C++.

# Load Saved Model From Filesystem
The first thing we need to do is to call LoadSaveModel function as following:
Here is sample code:
```
#include "tensorflow/cc/saved_model/loader.h"
#include "tensorflow/cc/saved_model/constants.h"
#include "tensorflow/cc/saved_model/signature_constants.h"
#include "tensorflow/cc/saved_model/tag_constants.h"
...

{
    SavedModelBundle bundle;
    SessionOptions session_options;
    RunOptions run_options;

    const string export_dir = "./models/saved_model";
    Status st = LoadSavedModel(session_options, run_options, export_dir,
                    {kSavedModelTagServe}, &bundle);

    ...
}
```
The LoadSaveModel function do the following things:
1. Parsing file to GrahDef object
2. Create session
3. Import GraphDef into the session
4. Return the session and GraphDef in SavedModelBundle

# Inference
After loading the model in session, we can now inference using the session
```
    std::vector<std::pair<string, tensorflow::Tensor>> inputs = {
            { "input_1:0", input_tensor }
    };
    std::vector<tensorflow::Tensor> outputs;
    st = bundle.session->Run(inputs, {"conv2d_19/truediv:0"}, {}, &outputs);
```

Here the "input_1:0" is the input node name and "conv2d_19/truediv:0" is the output node name. "input_tensor" has populated with the input data.
After this, the result will be in the "outputs" variable.

# Performance
As far as I tested on my GTX 1080ti, c++ api is about 2% faster than python.