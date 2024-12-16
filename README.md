# Wozniak Coffee Test
This project aims to solve the wozniak coffee test. This test consists of developing an inteligence which is able to make a cup of coffee at an arbitrary kitchen without assistance. According to Wozniak, when this test is concluded we will have achieved Artificial General Inteligence (AGI).
The problem is divided in two parts. First, an assistant will be developed in a way that is able to comunicate with a person using natural language and the person will behave as ordered by the AI. In the second moment the AI will receive a humanoid body and must control it to make the cup of coffee unassistedly.
If you are a developer, check the [Development section](#development).


# Install
To correctly install the dependencies of the project, follow all the instructions carefully.
The project was developed in ~/I2CA/wozniak_coffee_test. The developers tried to maintain the code compatible to other installation paths, but it is recommended to use the same path to avoid problems. If you choose to clone in a different location, take extra care when copying the commands suggested in this README.

## System Dependencies
To install system dependencies, use rosdep.
```bash
cd ~/I2CA/wozniak_coffee_test
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

## Intel Realsense
Go to the `realsense-ros` folder and follow the README installation instructions.

## Python Dependencies
ROS has some problems dealing with virtual environments. If you wish to use one, it is highly recommended to go with venv. If not, you can skip the subitem.

<details>
<summary>Venv</summary>
To create the virtual environment:
```bash
cd ~/I2CA/wozniak_coffee_test
python3 -m venv --system-site-packages venv
source venv/bin/activate
```
</details>

To install the python dependencies, run:
```bash
pip install -r requirements.txt
```

## Build
```bash
cd ~/I2CA/wozniak_coffee_test
colcon build --symlink-install
```

# Usage
For all terminals opened during the instructions, remember to source the installation before running the commands.
```bash
cd ~/I2CA/wozniak_coffee_test
source install/setup.bash
```
## Coffee Assistant
Connect the realsense to the computer and run:
```bash
ros2 launch robot_control_language realsense.launch.py
```


# Development
Due to the unecessary complexity added by frameworks such as Langchain, Llamaindex and Haystack, it was developed a simple class (robot_control_language/llm.py -> LLM) to interface with the OpenAI API using the openai official library. The class stores the state of the conversation (chat_history), allowing the user to worry only with calling the `invoke` method.

During the initialization, LLM requires a list of available functions (defined in robot_control_language/tools.py -> hercules_functions) and a configuration file (robot_control_language/settings.yaml). Take a look at them as examples for future development.

For a quick experiment that will make things more clear, try reading the robot_control_language/terminal_chat.py code and run it with:
```
cd robot_control_language
python3 terminal_chat.py
```