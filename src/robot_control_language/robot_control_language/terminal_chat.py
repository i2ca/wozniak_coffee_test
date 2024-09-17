import argparse
from termcolor import colored
from coffee.tools import hercules_functions
from llm import OpenAIAgent


def main(args):
    agent = OpenAIAgent(args.model, hercules_functions, args.settings_file_path)
    while True:
        user_message = input("User: ")
        print(colored(agent.invoke(user_message, None), "green"))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Chat with an LLM with tools.')
    parser.add_argument('--model', type=str, default="gpt-4o")
    parser.add_argument('--settings-file-path', type=str, default="coffee/settings.yaml")
    args = parser.parse_args()
    main(args)
