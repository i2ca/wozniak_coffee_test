# Wozniak_coffee_test

This document provides step-by-step instructions to activate the **MolmoAi** model on another computer using SSH on the **Atena** machine. **MolmoAi** refers to the **Molmo 7B-D** model, a multimodal language model developed by the [Allen Institute for AI](https://huggingface.co/allenai/Molmo-7B-D-0924).

## Step-by-Step Guide

### 1. Connect via SSH to Atena

    ssh user@atena

### 2. Set the GPU to be used (0, 1, or 2)

Set the CUDA_VISIBLE_DEVICES environment variable to specify the GPU you want to use:

    export CUDA_VISIBLE_DEVICES=0

Replace 0 with the GPU number of your choice.

### 3. Load the environment configurations

    source ~/.bashrc
    conda_init
    conda activate vllm

### 4. Start the vLLM server with the Molmo 7B-D model

    vllm serve allenai/Molmo-7B-D-0924 --served-model-name lcad-ica --gpu-memory-utilization 0.50 --trust-remote-code

This command starts an OpenAI-compatible API server hosting the specified model. (vLLM Documentation)

### 5. Access the server interface (Swagger UI)

After starting the server, the interactive API documentation interface (Swagger UI) will be available at:

    http://0.0.0.0:8000/docs

Open this URL in your browser to interact with the API and test the model's features.

By following these steps, you should be able to successfully activate and use the Molmo 7B-D model via SSH on the Atena machine.
