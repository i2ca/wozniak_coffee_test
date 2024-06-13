# wozniak_coffee_test

# Setup environment
It is recommended to install poetry to make the setup easier.
```
sudo apt install pipx
pipx ensurepath
```
Restart the terminal
```
pipx install poetry
```
If your python version is not the required by the project (^3.10), you can install different python versions in your system without comprimising current applications.
```
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install python3.xx
```
The new python installed will not replace your current python, but will be available to be used by Poetry.

To install all the dependecies, navigate to the root of the project and run
```
poetry install --no-root
```
To open a shell in the newly created python environment:
```
poetry shell
```

# Terminal Chat
For a quick experiment with the tool, you can run
```
cd robot_control_language
python3 terminal_chat.py
```

# Web Interface
To use the more complete web interface developed with Gradio, you can run
```
cd robot_control_language
python3 chat_hercules.py
```

# Development
Due to the unecessary complexity added by frameworks such as Langchain, Llamaindex and Haystack, it was developed a simple class (robot_control_language/llm.py -> LLM) to interface with the OpenAI API using the openai official library. The class stores the state of the conversation (chat_history), allowing the user to worry only with calling the `invoke` method.

During the initialization, LLM requires a list of available functions (defined in robot_control_language/tools.py -> hercules_functions) and a configuration file (robot_control_language/settings.yaml). Take a look at them as examples for future development.

## Roadmap
- Implement streaming response in llm.py and chat_hercules.py
