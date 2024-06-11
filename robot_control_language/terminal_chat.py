import argparse
from openai import OpenAI
from tenacity import retry, wait_random_exponential, stop_after_attempt
from termcolor import colored
import yaml
import json
from tools import available_functions


def load_settings(settings_file):
    with open(settings_file, 'r') as file:
        settings = yaml.safe_load(file)
    return settings["system_prompt"], settings["tools"]


@retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
def chat_completion_request(client, model, messages, tools=None, tool_choice=None):
    try:
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            tools=tools,
            tool_choice=tool_choice,
        )
        return response
    except Exception as e:
        print("Unable to generate ChatCompletion response")
        print(f"Exception: {e}")
        return e
    

def pretty_print_message(message):
    role_to_color = {
        "assistant": "blue",
        "function": "magenta",
    }
    print(colored(f"Assistant: {message.content}", role_to_color["assistant"]))
    print(colored(f"Function: {message.tool_calls}", role_to_color["function"]))


def main(args):
    client = OpenAI()
    system_prompt, tools = load_settings(args.settings_file_path)
    messages = []
    messages.append({"role": "system", "content": system_prompt})
    
    while True:
        user_message = input("User: ")
        messages.append({"role": "user", "content": user_message})

        response_message = chat_completion_request(
            client, args.model, messages, tools=tools
        ).choices[0].message
        pretty_print_message(response_message)
        tool_calls = response_message.tool_calls
        messages.append(response_message)
        while tool_calls:
            # Send the info for each function call and function response to the model
            for tool_call in tool_calls:
                function_name = tool_call.function.name
                function_to_call = available_functions[function_name]
                function_args = json.loads(tool_call.function.arguments)
                function_response = function_to_call(
                    location=function_args.get("location"),
                    unit=function_args.get("unit"),
                )
                messages.append(
                    {
                        "tool_call_id": tool_call.id,
                        "role": "tool",
                        "name": function_name,
                        "content": function_response,
                    }
                )
            response_message = chat_completion_request(
                client, args.model, messages, tools=tools
            ).choices[0].message
            pretty_print_message(response_message)
            tool_calls = response_message.tool_calls
            messages.append(response_message)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Chat with an LLM with tools.')
    parser.add_argument('--model', type=str, default="gpt-4o")
    parser.add_argument('--settings-file-path', type=str, default="settings.yaml")
    args = parser.parse_args()
    main(args)
