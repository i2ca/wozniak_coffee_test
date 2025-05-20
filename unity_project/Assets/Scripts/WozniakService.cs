using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Meta.XR.MRUtilityKit;
using TMPro;
using RosSharp.RosBridgeClient;
using WozniakInterfaces = RosSharp.RosBridgeClient.MessageTypes.WozniakInterfaces;
using PickObject = RosSharp.RosBridgeClient.MessageTypes.PickObject;
using Newtonsoft.Json;
using Oculus.Interaction.Input;

namespace RosSharp.RosBridgeClient.MessageTypes
{
    public class WozniakService : UnityServiceProvider<WozniakInterfaces.HighlightObjectRequest, WozniakInterfaces.HighlightObjectResponse>
    {

        // Control variables
        private bool sceneLoaded = false;
        private bool started = false;
        private bool done = false;

        // Queue to handle the service requests 
        private Queue<WozniakInterfaces.HighlightObjectRequest> requestsQueue = new Queue<WozniakInterfaces.HighlightObjectRequest>();

        // Scripts 
        [SerializeField] GameControl gameControl;
        [SerializeField] CanvasHandler canvas;

        void Start()
        {
            base.Start(); 
        }

        private void Update()
        {

            // You can also start the application using the keyboard
            if (Input.GetKeyDown(KeyCode.Space))
            {
                NextTask();
            }

            while (requestsQueue.Count > 0)
            {
                var request = requestsQueue.Dequeue(); // dequeuing to process request
                StartCoroutine(ProcessRequest(request));
            }

        }

        protected override bool ServiceCallHandler(WozniakInterfaces.HighlightObjectRequest request, out WozniakInterfaces.HighlightObjectResponse response)
        {
            Debug.Log("REQUEST TARGET OBJECT:" + request.target_object);
            Debug.Log("REQUEST INSTRUCTION:" + request.instruction);

            // Update instruction as a global variable 

            response = new WozniakInterfaces.HighlightObjectResponse(); // Creating a new response 
            requestsQueue.Enqueue(request); // queueing requests

            response.message = "Request sent successfully"; // This will be done with 2 services. The first response is only to client to know that the request was done
            response.success = true;
            return true;
        }

        IEnumerator ProcessRequest(WozniakInterfaces.HighlightObjectRequest request)
        {
            yield return new WaitForSeconds(1);

            if (request == null || request.instruction == null || request.target_object == null)
            {
                Debug.LogError("Request has some null value");
                yield return null;
            }
                
            if (!started)
            {
                yield return null;
            }

            // Verify if the task has ended
            if (request.target_object == "done")
            {
                FinishGame();
                yield return null;
            }
            else
            {
                // Update the canvas with request instruction
                canvas.UpdateInstruction(request.instruction);
            }
            
            Debug.LogError("Scene not loaded");
            yield return null;
        }

        public void NextTask()
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
        }

        // Callback to handle the response from the PickObject service
        private void ServiceResponseHandler(PickObject.PickObjectResponse response)
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

        // Lembrar de colocar essa dinâmica no gamecontrol, e não nesse script 
        void FinishGame()
        {
            done = true;

            canvas.UpdateHandCoordinate(null);
            canvas.UpdatePlayerCoordinate(null);
            canvas.UpdateObjectCoordinate(null);

            //Instantiate(confetti, canvas.transform.position, Quaternion.identity);
        }
    }

}
