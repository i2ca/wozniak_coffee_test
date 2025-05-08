using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Meta.XR.MRUtilityKit;
using TMPro;
using RosSharp.RosBridgeClient;
using PickObject = RosSharp.RosBridgeClient.MessageTypes.PickObject; 
using Newtonsoft.Json; 

namespace RosSharp.RosBridgeClient.MessageTypes.PickObject
{
    public class WozniakObjectCall : MonoBehaviour
    {

        private bool sceneLoaded = false;

        void Start()
        {
            StartCoroutine(InitializeAfterSceneLoad());
        }

        IEnumerator InitializeAfterSceneLoad()
        {
            yield return new WaitForSeconds(1f);
            InitializeProgram();
            //UpdateText();
            sceneLoaded = true;
            Debug.Log("Scene loaded");
        }

        void InitializeProgram()
        {
            Debug.Log("Program Initialized");

        }

        private void Update()
        {
            if(sceneLoaded){

                if (Input.GetKeyDown(KeyCode.Space))
                {
                Debug.Log("Espaço pressionado! Ação disparada.");

                // Criação da requisição para o serviço `PickObject`
                var request = new PickObject.PickObjectRequest
                {
                    target_object = "I am ready" // Defina o nome do objeto a ser pego
                };

                GetComponent<RosConnector>().RosSocket.CallService<PickObject.PickObjectRequest, PickObject.PickObjectResponse>("/pick_object", ServiceResponseHandler, request);
                
                }

            }
            
        }

        

        // Callback para processar a do PickObject
        private void ServiceResponseHandler(PickObject.PickObjectResponse response)
        {



            if (response.success)
            {
                Debug.Log("Objeto selecionado com sucesso! Mensagem: " + response.message);
            }
            else
            {
                Debug.Log("Falha ao selecionar o objeto. Mensagem: " + response.message);
            }
        }

        
    }       
}
