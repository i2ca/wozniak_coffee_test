from typing import List, Dict
from openai import OpenAI
import yaml
from tenacity import retry, wait_random_exponential, stop_after_attempt
import json
import os


class OpenAIAgent():
    def __init__(self, model="gpt4o-mini", available_functions=None, settings_file_path=None, node=None):
        # Configuração do OpenAI
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY não encontrada nas variáveis de ambiente")
            
        self.openai_client = OpenAI(
            api_key=self.api_key
        )
        self.model = model
        self.node = node
        self.message_history = []  # Adiciona histórico de mensagens
        self.current_item_index = 0  # Índice do item atual na lista
        self.item_list = []  # Lista de itens a serem processados
        
        if settings_file_path:
            self.system_prompt, self.tools = self._load_settings(settings_file_path)
        else:
            self.system_prompt = "You are an assistant that helps identify and locate objects in images."
            self.tools = []
            
        self.available_functions = available_functions if available_functions else {}
        
        # Adiciona o prompt do sistema ao histórico
        self.message_history.append({
            "role": "system",
            "content": self.system_prompt
        })

    def _load_settings(self, settings_file):
        with open(settings_file, 'r') as file:
            settings = yaml.safe_load(file)
        return settings["system_prompt"], settings.get("tools", [])

    @retry(wait=wait_random_exponential(multiplier=1, max=10), stop=stop_after_attempt(2))
    def _chat_completion_request(self, messages):
        try:
            if self.node:
                self.node.get_logger().info('Enviando requisição para GPT-4...')
                self.node.get_logger().info(f'Mensagens: {json.dumps(messages, indent=2)}')
            
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7
            )
            
            return response
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro na requisição: {str(e)}")
            raise e

    def _process_function_call(self, function_call):
        """Processa uma chamada de função da LLM"""
        if not function_call or not self.available_functions:
            return None

        try:
            # Se a LLM menciona um objeto para procurar, chama pick_object
            if isinstance(function_call, str):
                import re
                import unicodedata
                
                def normalize_text(text):
                    """Remove acentos e converte para minúsculas"""
                    # Converte para minúsculo
                    text = text.lower()
                    # Remove acentos
                    text = unicodedata.normalize('NFKD', text).encode('ASCII', 'ignore').decode('ASCII')
                    return text
                
                # Procura por menções a objetos no texto
                normalized_text = function_call.lower()
                
                # Verifica se é uma solicitação para listar itens
                if "lista de itens:" in normalized_text.lower():
                    # Extrai a lista de itens do texto
                    items = []
                    lines = normalized_text.split('\n')
                    for line in lines:
                        if '-' in line:
                            item = line.split('-')[1].strip()
                            # Normaliza o item (remove acentos e converte para minúsculas)
                            normalized_item = normalize_text(item)
                            items.append(normalized_item)
                    
                    if items:
                        self.item_list = items
                        self.current_item_index = 0
                        # Procura o primeiro item
                        if self.node:
                            self.node.get_logger().info(f"Iniciando busca sequencial. Primeiro item: {items[0]}")
                        
                        # Processa todos os itens
                        responses = []
                        for item in items:
                            if self.node:
                                self.node.get_logger().info(f"Procurando {item}...")
                            try:
                                response = self.available_functions['pick_object'](item)
                                responses.append(f"\nVou procurar {item}...\n{response}")
                            except Exception as e:
                                responses.append(f"\nVou procurar {item}...\nErro ao procurar {item}: {str(e)}")
                        
                        return "\n".join(responses)
                
                # Procura por padrões comuns de menção a objetos
                patterns = [
                    # Padrões diretos (objeto mencionado diretamente)
                    r'(?:o|a|um|uma|aquele|aquela|este|esta|esse|essa)\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    # Padrões com verbos
                    r'procurar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'localizar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'encontrar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'identificar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'mostrar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'achar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'buscar\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'ver\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    # Padrões com "está"
                    r'onde\s+esta\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    r'cadê\s+(?:o|a|um|uma)?\s+([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)',
                    # Padrões de lista numerada
                    r'\d+\s*[-\)]\s*([^\s\.,]+(?:\s+de\s+[^\s\.,]+)?)'
                ]
                
                for pattern in patterns:
                    matches = re.finditer(pattern, normalized_text)
                    for match in matches:
                        object_name = match.group(1).strip()
                        # Normaliza o nome do objeto
                        normalized_name = normalize_text(object_name)
                        if self.node:
                            self.node.get_logger().info(f"Chamando pick_object para: {normalized_name}")
                        return self.available_functions['pick_object'](normalized_name)
            
            return None
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro ao processar função: {str(e)}")
            return None

    def invoke(self, text, image_path=None):
        try:
            if self.node:
                self.node.get_logger().info(f'Processando mensagem: {text}')
            
            # Adiciona a mensagem do usuário ao histórico
            user_message = {
                "role": "user",
                "content": text
            }
            self.message_history.append(user_message)
            
            try:
                if self.node:
                    self.node.get_logger().info('Enviando requisição para GPT-4...')
                response = self._chat_completion_request(self.message_history)
                response_content = response.choices[0].message.content
                if self.node:
                    self.node.get_logger().info(f'Resposta do GPT-4: {response_content}')
                
                # Verifica se há alguma função a ser chamada
                function_response = self._process_function_call(response_content)
                if function_response:
                    if self.node:
                        self.node.get_logger().info(f'Função executada com sucesso: {function_response}')
                    
                    # Se uma função foi chamada e temos mais itens na lista, continua automaticamente
                    if self.item_list and self.current_item_index < len(self.item_list) - 1:
                        # Continua procurando os próximos itens independentemente do resultado
                        responses = [function_response]
                        while self.current_item_index < len(self.item_list) - 1:
                            self.current_item_index += 1
                            next_item = self.item_list[self.current_item_index]
                            responses.append(f"\n\nVou procurar {next_item}...")
                            try:
                                next_response = self.available_functions['pick_object'](next_item)
                                responses.append(next_response)
                            except Exception as e:
                                responses.append(f"Erro ao procurar {next_item}: {str(e)}")
                        
                        # Combina todas as respostas
                        combined_response = "\n".join(responses)
                        assistant_message = {
                            "role": "assistant",
                            "content": f"Entendi. {combined_response}"
                        }
                    else:
                        # Se não há mais itens ou não estamos processando uma lista
                        assistant_message = {
                            "role": "assistant",
                            "content": f"Entendi. {function_response}"
                        }
                else:
                    if self.node:
                        self.node.get_logger().info('Nenhuma função identificada na resposta')
                    # Se não houve chamada de função, usa a resposta normal
                    assistant_message = {
                        "role": "assistant",
                        "content": response_content
                    }
                
                self.message_history.append(assistant_message)
                return [assistant_message]
            except Exception as e:
                error_msg = "Erro ao se comunicar com o GPT-4."
                if self.node:
                    self.node.get_logger().error(f"Erro ao se comunicar com o GPT-4. Detalhes: {str(e)}")
                return [{
                    "role": "assistant",
                    "content": error_msg
                }]
            
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Erro ao processar requisição: {str(e)}")
            return [{
                "role": "assistant",
                "content": f"Erro ao processar sua solicitação: {str(e)}"
            }]

