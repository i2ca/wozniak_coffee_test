from typing import List, Dict
from openai import OpenAI
import yaml
from tenacity import retry, wait_random_exponential, stop_after_attempt
import json
import base64
from rclpy.node import Node


class OpenAIAgent():

    chat_history: List[Dict]
    openai_client: OpenAI
    model: str
    system_prompt: str
    tools: dict
    available_functions: dict
    node: Node


    def __init__(self, model, available_functions, settings_file_path, node, api_key=None, api_base='https://api.openai.com/v1'):
        if api_key is None:
            self.openai_client = OpenAI()
        else:
            self.openai_client = OpenAI(api_key=api_key, base_url=api_base)
        self.node = node
        self.model = model
        self.system_prompt, self.tools = self._load_settings(settings_file_path)
        self.available_functions = available_functions
        self.chat_history = [{'role': 'system', 'content': self.system_prompt}]
    

    def _load_settings(self, settings_file):
        with open(settings_file, 'r') as file:
            settings = yaml.safe_load(file)
        return settings["system_prompt"], settings["tools"]
    

    @retry(wait=wait_random_exponential(multiplier=1, max=40), stop=stop_after_attempt(3))
    def _chat_completion_request(self):
        try:
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=self.chat_history,
                tools=self.tools,
            )
            return response
        except Exception as e:
            self.node.get_logger().error("Unable to generate ChatCompletion response")
            self.node.get_logger().error(f"Exception: {e}")
            return e


    def _add_user_message_to_chat_history(self, text, image_path):
        message = {
            "role": "user",
            "content": [],
        }
        if text is not None:
            message["content"].append({"type": "text", "text": text})
        if image_path is not None:
            with open(image_path, "rb") as image_file:
                base64_image = base64.b64encode(image_file.read()).decode('utf-8')
            message["content"].append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}})
        
        if (len(message["content"]) == 0):
            raise ValueError("No content provided for user message")
        self.chat_history.append(message)


    def _execute_tool_call(self, tool_call):
        self.node.get_logger().info(f"Executing tool call: {tool_call}")
        function_name = tool_call.function.name
        function_to_call = self.available_functions[function_name]
        function_args = json.loads(tool_call.function.arguments)
        function_args["node"] = self.node
        function_response = function_to_call(**function_args)
        self.node.get_logger().info(f"Function response: {function_response}")
        tool_response = {
            "tool_call_id": tool_call.id,
            "role": "tool",
            "name": function_name,
            "content": function_response,
        }
        self.chat_history.append(tool_response)


    def invoke(self, text, image_path):
        self.node.get_logger().info(f"Received text: {text}")
        self._add_user_message_to_chat_history(text, image_path)
        new_message_index = len(self.chat_history)
        while True:
            self.node.get_logger().info("Requesting chat completion")
            response_message = self._chat_completion_request().choices[0].message
            tool_calls = response_message.tool_calls
            self.chat_history.append(response_message)
            if tool_calls is None:
                break
            for tool_call in tool_calls:
                self._execute_tool_call(tool_call)
        return self.chat_history[new_message_index:]
