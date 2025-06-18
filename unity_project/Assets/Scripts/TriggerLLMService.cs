using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Meta.XR.MRUtilityKit;
using RosSharp.RosBridgeClient;
using WozniakInterfaces = RosSharp.RosBridgeClient.MessageTypes.WozniakInterfaces;
using Newtonsoft.Json;
using Oculus.Interaction.Input;

namespace RosSharp.RosBridgeClient.MessageTypes
{
    public class TriggerLLMService : UnityServiceProvider<WozniakInterfaces.TriggerLLMRequest, WozniakInterfaces.TriggerLLMResponse>
    {
        // Scripts 
        [SerializeField] GameControl _gameControl;
        [SerializeField] CanvasHandler _canvas;
        [SerializeField] global::InstructionService _instructionService;

        void Start()
        {
            base.Start();
            if (_instructionService == null)
            {
                Debug.LogError("[TriggerLLMService] _instructionService não está atribuído no Inspector!");
            }
            if (_canvas == null)
            {
                Debug.LogWarning("[TriggerLLMService] _canvas (para fallback) não está atribuído no Inspector.");
            }
        }

        public void TriggerLLMSendRequest(string message)
        {
            var request = new WozniakInterfaces.TriggerLLMRequest
            {
                message = message 
            };
            Debug.Log("[TriggerLLMService] Enviando requisição para /TriggerLLM (ROS): " + message);
            GetComponent<RosConnector>().RosSocket.CallService<WozniakInterfaces.TriggerLLMRequest, WozniakInterfaces.TriggerLLMResponse>("/TriggerLLM", RosServiceResponseHandler, request);
        }

        protected override bool ServiceCallHandler(WozniakInterfaces.TriggerLLMRequest request, out WozniakInterfaces.TriggerLLMResponse response)
        {
            string message = request.message;
            Debug.Log("[TriggerLLMService as Server] ServiceCallHandler recebeu: " + message);
            
            TriggerLLMSendRequest(message); 

            response = new WozniakInterfaces.TriggerLLMResponse("Requisição para /TriggerLLM (ROS) encaminhada.", true);
            return true; 
        }

        private void RosServiceResponseHandler(WozniakInterfaces.TriggerLLMResponse rosResponse)
        {
            if (rosResponse.success)
            {
                Debug.Log("[TriggerLLMService] Resposta recebida de /TriggerLLM (ROS): " + rosResponse.message);

                if (_instructionService != null)
                {
                    _instructionService.DisplayInstruction(rosResponse.message);
                }
                else
                {
                    Debug.LogError("[TriggerLLMService] _instructionService não está atribuído! Tentando fallback para _canvas.");
                    if (_canvas != null)
                    {
                        _canvas.UpdateInstruction("Fallback (IS null): " + rosResponse.message);
                    }
                    else
                    {
                        Debug.LogError("[TriggerLLMService] _canvas também não está atribuído! Não é possível exibir a mensagem.");
                    }
                }
            }
            else
            {
                Debug.LogError("[TriggerLLMService] Falha na resposta de /TriggerLLM (ROS): " + rosResponse.message);
                if (_canvas != null)
                {
                    _canvas.UpdateInstruction("Erro ao comunicar com a LLM: " + rosResponse.message); 
                }
            }
        }
    }
}