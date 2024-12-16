import argparse
from termcolor import colored
from robot_control_language.coffee.tools import hercules_functions
from robot_control_language.llm import OpenAIAgent
from openai.types.chat.chat_completion_message import ChatCompletionMessage


def main():
    agent = OpenAIAgent("gpt-4o", hercules_functions, "/home/breno/I2CA/wozniak_coffee_test/src/robot_control_language/robot_control_language/coffee/settings.yaml")
    while True:
        user_message = input("User: ")
        response = agent.invoke(user_message, None)
        for item in response:
            if isinstance(item, ChatCompletionMessage):
                print(colored(item.content, "green"))
                if item.tool_calls is None:
                    continue
                for tool in item.tool_calls:
                    print(colored(f"Tool called: {tool.function.name}, Args: {tool.function.arguments}, Call ID: {tool.id}", "yellow"))
            else:
                print(colored(f"Tool responded: {item['name']}, Response: {item['content']}, Call ID: {item['tool_call_id']}", "yellow"))
