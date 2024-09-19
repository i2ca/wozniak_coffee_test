import argparse
from termcolor import colored
from robot_control_language.coffee.tools import hercules_functions
from robot_control_language.llm import OpenAIAgent


def main():
    agent = OpenAIAgent("gpt-4o", hercules_functions, "/home/breno/I2CA/wozniak_coffee_test/src/robot_control_language/robot_control_language/coffee/settings.yaml")
    while True:
        user_message = input("User: ")
        print(colored(agent.invoke(user_message, None), "green"))
