

import gradio as gr
import llm
import time
import whisper
import argparse


########################################################################################################################
########################################################################################################################
_title_markdown = ("""
# Hércules LCAD 💪
Você pode conversar com o Hércules via texto ou áudio, basta usar as entradas abaixo. A prioridade é sempre via áudio. 
Logo, se enviar uma mensagem usando ambas as entradas, o Hércules irá considerar a mensagem de áudio.
""")

# Variavel global que armazena o texto que vai vim do audio
_text_from_audio_inference = ""

# Carrega o modelo do whisper. A primeira vez que roda ele baixa o modelo para a máquina. Pode alterar para outros
# modelos Se rodar `whisper.available_models()`, é possível ver os modelos atuais. O segundo parâmetro define o device
# que vai ser realizada a inferência. Se usar GPU, o CUDA tem que estar configurado corretamente.
_model = whisper.load_model("small", "cpu")

# Variavel com o texto da inferencia e o historico da conversa com o modelo (considerando a pré configuração)
_llm_response = None
_history_llm_talk = ""
_openai_client: llm.openai.Client = None
_language_model: str = ""
_tools = None
########################################################################################################################
########################################################################################################################


def audio_inference(audio):
    global _text_from_audio_inference

    audio = whisper.load_audio(audio)
    audio = whisper.pad_or_trim(audio)
    mel = whisper.log_mel_spectrogram(audio).to(_model.device)
    _, probs = _model.detect_language(mel)
    options = whisper.DecodingOptions(fp16=False)
    result = whisper.decode(_model, mel, options)

    _text_from_audio_inference = result.text

    # Chama a inferencia do audio e retorna o texto
    return _text_from_audio_inference


def chat_input_message_update(audio_input, text_input, history):
    global _text_from_audio_inference

    resp = text_input
    _text_from_audio_inference = resp
    if audio_input is not None:
        resp = audio_inference(audio_input)

    if resp != "":
        return "", history + [[resp, None]]
    else:
        return "", history


# Aqui que a inferencia da llm entra
def update_text_from_llm():
    global _llm_response

    _llm_response = llm.chat_completion_request(_text_from_audio_inference, _history_llm_talk, _openai_client, _language_model, tools=_tools)


def write_hercules_answer_on_chat(history):
    global _llm_response
    llm_response = ""
    llm_response += "Text: "
    if _llm_response.choices[0].message.content is not None:
        llm_response += _llm_response.choices[0].message.content
    
    llm_response += "\nFunctions: "
    if _llm_response.choices[0].message.tool_calls is not None:
        llm_response += str(_llm_response.choices[0].message.tool_calls)
    
    return [(history[-1][0], llm_response)]


def update_text_from_audio():
    return gr.Textbox(label="Digite sua mensagem para o Hércules",
                      value=_text_from_audio_inference,
                      scale=2)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Hércules LCAD')
    parser.add_argument('--api-base', default=None, help='URL da API base')
    parser.add_argument('--model', default='gpt-4o', help='Modelo de chat')
    parser.add_argument('--settings-file-path', type=str, default="settings.yaml")
    args = parser.parse_args()

    system_prompt, _tools = llm.load_settings(args.settings_file_path)

    _openai_client, _history_llm_talk = llm.set_pre_configuration(args.api_base, system_prompt)
    _language_model = args.model

    with gr.Blocks(title="Hercules LCAD",
                   theme=gr.themes.Soft(primary_hue="blue"),
                   css=".gradio-container {background-color: #f5f5f5;}") as demo:


        gr.Markdown(_title_markdown)
        chatbot = gr.Chatbot(label="Histórico da conversa")

        with gr.Row():
            audio_input = gr.Audio(label="Fale sua mensagem para o Hércules",
                                   sources=["microphone"],
                                   type="filepath",
                                   scale=1)

            text_input = gr.Textbox(label="Texto da mensagem para o Hércules",
                                    scale=2)

            # image_input = gr.Image(label="Envie sua imagem para o Hércules")

        with gr.Row():
            submit_btn = gr.Button("Enviar ↪️", scale=2)
            clear_history_btn = gr.Button("Limpar histórico 🗑️")
            clear_inputs_btn = gr.Button("♻️", scale=0)


        submit_btn.click(chat_input_message_update,
                         [audio_input, text_input, chatbot],
                         [text_input, chatbot], queue=False)\
            .then(update_text_from_audio, None, text_input)\
            .then(update_text_from_llm, None, None)\
            .then(write_hercules_answer_on_chat, chatbot, chatbot)

        clear_history_btn.click(lambda: None, None, chatbot, queue=False)

        clear_inputs_btn.click(lambda: None, None, text_input, queue=False)


    demo.queue()
    demo.launch()
