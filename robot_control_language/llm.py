import openai
import re
import yaml
from tenacity import retry, wait_random_exponential, stop_after_attempt


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
def chat_completion_request(current_text, messages, client, model, temperature=0.0, max_tokens=None, tools=None, tool_choice=None):
    messages.append({'role': 'user', 'content': current_text})
    try:
        response = client.chat.completions.create(
            model=model,
            messages=messages,
            tools=tools,
            tool_choice=tool_choice,
            temperature=temperature,
            max_tokens=None,
        )
        messages.append(response.choices[0].message)
        return response
    except Exception as e:
        print("Unable to generate ChatCompletion response")
        print(f"Exception: {e}")
        return e
