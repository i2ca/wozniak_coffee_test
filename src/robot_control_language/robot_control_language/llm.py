from typing import List, Dict
from openai import OpenAI
import yaml
from tenacity import retry, wait_random_exponential, stop_after_attempt
import json
import base64
import requests
import time


class OpenAIAgent():

    chat_history: List[Dict]
    openai_client: OpenAI
    model: str
    system_prompt: str
    tools: dict
    available_functions: dict
    node: any
    base_url: str


    # Versão original do __init__
    # def __init__(self, model, available_functions, settings_file_path, api_key=None, api_base='https://api.openai.com/v1'):
    #     if api_key is None:
    #         self.openai_client = OpenAI()
    #     else:
    #         self.openai_client = OpenAI(api_key=api_key, base_url=api_base)
    #     self.model = model
    #     self.system_prompt, self.tools = self._load_settings(settings_file_path)
    #     self.available_functions = available_functions
    #     self.chat_history = [{'role': 'system', 'content': self.system_prompt}]
    
    # Versão MolmoAI do __init__
    def __init__(self, model="lcad-ica", available_functions=None, settings_file_path=None, node=None):
        # Configuração específica para MolmoAI
        self.base_url = "http://10.9.8.252:8000/v1"
        self.openai_client = OpenAI(
            base_url=self.base_url,
            api_key="not-needed",  # MolmoAI não requer API key
            timeout=30.0  # Aumentando o timeout para 30 segundos
        )
        self.model = model
        self.node = node
        
        # Verifica a conexão com o servidor
        self._check_server_connection()
        
        if settings_file_path:
            self.system_prompt, self.tools = self._load_settings(settings_file_path)
        else:
            # Configuração padrão para MolmoAI
            self.system_prompt = "You are an assistant that helps identify and locate objects in images."
            self.tools = []
            
        self.available_functions = available_functions if available_functions else {}
        self.chat_history = [{'role': 'system', 'content': self.system_prompt}]

    def _check_server_connection(self):
        """Verifica se o servidor MolmoAI está acessível"""
        try:
            if self.node:
                self.node.get_logger().info(f'Verificando conexão com MolmoAI em {self.base_url}...')
            
            # Tenta fazer uma requisição simples para o servidor
            response = requests.get(self.base_url + "/health", timeout=5)
            if response.status_code == 200:
                if self.node:
                    self.node.get_logger().info('Conexão com MolmoAI estabelecida com sucesso!')
            else:
                raise Exception(f"Servidor respondeu com status code {response.status_code}")
                
        except requests.exceptions.RequestException as e:
            error_msg = f"Não foi possível conectar ao servidor MolmoAI: {str(e)}"
            if self.node:
                self.node.get_logger().error(error_msg)
            raise Exception(error_msg)

    def _load_settings(self, settings_file):
        with open(settings_file, 'r') as file:
            settings = yaml.safe_load(file)
        return settings["system_prompt"], settings.get("tools", [])
    

    @retry(wait=wait_random_exponential(multiplier=1, max=10), stop=stop_after_attempt(2))
    def _chat_completion_request(self):
        try:
            # Formato específico para MolmoAI
            messages = []
            for msg in self.chat_history:
                if isinstance(msg['content'], list):
                    # Já está no formato correto para mensagens com imagem
                    messages.append(msg)
                else:
                    # Converte mensagens de texto simples
                    messages.append({
                        "role": msg['role'],
                        "content": [{"type": "text", "text": msg['content']}]
                    })
            
            if self.node:
                self.node.get_logger().info('Enviando requisição para MolmoAI...')
            
            start_time = time.time()
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=300,
            )
            elapsed_time = time.time() - start_time
            
            if self.node:
                self.node.get_logger().info(f'Resposta recebida da MolmoAI em {elapsed_time:.2f} segundos')
            
            return response
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro na requisição para MolmoAI: {str(e)}")
            raise e


    def _add_user_message_to_chat_history(self, text, image_path):
        message = {
            "role": "user",
            "content": [],
        }
        if text is not None:
            message["content"].append({"type": "text", "text": text})
        if image_path is not None:
            try:
                with open(image_path, "rb") as image_file:
                    base64_image = base64.b64encode(image_file.read()).decode('utf-8')
                message["content"].append(
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                )
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Erro ao processar imagem: {str(e)}")
                raise e
        
        if len(message["content"]) == 0:
            raise ValueError("No content provided for user message")
        
        self.chat_history.append(message)


    def _execute_tool_call(self, tool_call):
        function_name = tool_call.function.name
        function_to_call = self.available_functions[function_name]
        function_args = json.loads(tool_call.function.arguments)
        function_response = function_to_call(**function_args)
        tool_response = {
            "tool_call_id": tool_call.id,
            "role": "tool",
            "name": function_name,
            "content": function_response,
        }
        self.chat_history.append(tool_response)

    # Versão original do invoke
    # def invoke(self, text, image_path):
    #     self._add_user_message_to_chat_history(text, image_path)
    #     new_message_index = len(self.chat_history)
    #     while True:
    #         response_message = self._chat_completion_request().choices[0].message
    #         tool_calls = response_message.tool_calls
    #         self.chat_history.append(response_message)
    #         if tool_calls is None:
    #             break
    #         for tool_call in tool_calls:
    #             self._execute_tool_call(tool_call)
    #     return self.chat_history[new_message_index:]

    # Versão MolmoAI do invoke
    def invoke(self, text, image_path=None):
        try:
            if self.node:
                self.node.get_logger().info(f'Processando mensagem: {text}')
            
            self._add_user_message_to_chat_history(text, image_path)
            new_message_index = len(self.chat_history)
            
            try:
                response = self._chat_completion_request()
                response_message = response.choices[0].message
                
                # Adiciona a resposta ao histórico
                self.chat_history.append({
                    "role": "assistant",
                    "content": response_message.content
                })
                
                if self.node:
                    self.node.get_logger().info(f'Resposta processada: {response_message.content}')
                
                return self.chat_history[new_message_index:]
            except Exception as e:
                error_msg = "Desculpe, não consegui me comunicar com o servidor MolmoAI. Por favor, tente novamente em alguns instantes."
                if self.node:
                    self.node.get_logger().error(f"Erro ao invocar LLM: {str(e)}")
                return [{
                    "role": "assistant",
                    "content": error_msg
                }]
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro ao processar requisição: {str(e)}")
            return [{
                "role": "assistant",
                "content": f"Desculpe, ocorreu um erro ao processar sua solicitação: {str(e)}"
            }]
