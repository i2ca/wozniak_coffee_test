import openai
import yaml
from tenacity import retry, wait_random_exponential, stop_after_attempt
from tools import available_functions
import json


def load_settings(settings_file):
    with open(settings_file, 'r') as file:
        settings = yaml.safe_load(file)
    return settings["system_prompt"], settings["tools"]


def set_pre_configuration(api_base, system_prompt, talk=None):
    if api_base is not None:
        openai_api_key = "EMPTY"
        openai_api_base = api_base
        openai_client = openai.Client(api_key=openai_api_key, base_url=openai_api_base)
    else:
        openai_client = openai.Client()

    if talk is None:
        talk = [{'role': 'system', 'content': system_prompt}]        

    return openai_client, talk


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


def completion_request(current_text, messages, client, model, temperature=0.0, max_tokens=None, tools=None, tool_choice=None):
    messages.append({'role': 'user', 'content': current_text})
    new_messages = []
    response_message = chat_completion_request(
        client, model, messages, tools=tools
    ).choices[0].message
    tool_calls = response_message.tool_calls
    messages.append(response_message)
    new_messages.append(response_message)
    while tool_calls:
        new_messages.append(response_message)
        messages.append(response_message)
        # Send the info for each function call and function response to the model
        for tool_call in tool_calls:
            print(tool_call)
            function_name = tool_call.function.name
            function_to_call = available_functions[function_name]
            function_args = json.loads(tool_call.function.arguments)
            function_response = function_to_call(
                location=function_args.get("location"),
                unit=function_args.get("unit"),
            )
            tool_response = {
                "tool_call_id": tool_call.id,
                "role": "tool",
                "name": function_name,
                "content": function_response,
            }
            new_messages.append(tool_response)
            messages.append(tool_response)
        response_message = chat_completion_request(
            client, model, messages, tools=tools
        ).choices[0].message
        tool_calls = response_message.tool_calls
        messages.append(response_message)
        new_messages.append(response_message)   
    return new_messages
