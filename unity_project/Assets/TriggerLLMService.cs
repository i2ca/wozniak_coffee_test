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
        // Control variables
        private bool sceneLoaded = false;
        private bool started = false;
        private bool done = false;

        // Queue to handle the service requests 
        private Queue<WozniakInterfaces.TriggerLLMRequest> requestsQueue = new Queue<WozniakInterfaces.TriggerLLMRequest>();

        // Scripts 
        [SerializeField] GameControl _gameControl;
        [SerializeField] CanvasHandler _canvas;

        void Start()
        {
            base.Start();
        }

        public void TriggerLLMSendRequest(string message)
        {
            var request = new WozniakInterfaces.TriggerLLMRequest
            {
                message = message 
            };

            GetComponent<RosConnector>().RosSocket.CallService<WozniakInterfaces.TriggerLLMRequest, WozniakInterfaces.TriggerLLMResponse>("/TriggerLLM", ServiceResponseHandler, request);
        }

        protected override bool ServiceCallHandler(WozniakInterfaces.TriggerLLMRequest request, out WozniakInterfaces.TriggerLLMResponse response)
        {
            string message = request.message;

            Debug.Log("Service message received. Message: " + message);

            response = new WozniakInterfaces.TriggerLLMResponse(message + " processed succesfully.", true);

            return true; 
        }

        private void ServiceResponseHandler(WozniakInterfaces.TriggerLLMResponse response)
        {
            if (response.success)
            {
                Debug.Log("Object successfully picked! Message: " + response.message);
            }
            else
            {
                Debug.Log("Failed to pick the object. Message: " + response.message);
            }
        }


        /*public void NextTask()
        {
            if (!started)
            {
                Debug.Log("I am ready to start.");

                // Criação da requisição para o serviço `PickObject`
                var request = new PickObject.PickObjectRequest
                {
                    target_object = "I am ready :)" // Defina o nome do objeto a ser pego
                };

                GetComponent<RosConnector>().RosSocket.CallService<PickObject.PickObjectRequest, PickObject.PickObjectResponse>("/pick_object", ServiceResponseHandler, request);

                started = true;
            }
            else
            {
                // Criação da requisição para o serviço `PickObject`
                var request = new PickObject.PickObjectRequest
                {
                    target_object = "Okay" // Defina o nome do objeto a ser pego
                };

                GetComponent<RosConnector>().RosSocket.CallService<PickObject.PickObjectRequest, PickObject.PickObjectResponse>("/pick_object", ServiceResponseHandler, request);
            }
        }*/
    }

}

