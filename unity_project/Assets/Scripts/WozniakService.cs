
/*
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
        [SerializeField] string obj1, obj2, obj3;
        int totalObjects = 3;

        private string[] sceneObjects;
        private GameObject[] sceneGameObjects;
        private bool sceneLoaded = false;
        private bool started = false;
        private bool done = false;

        public GameObject textPrefab;
        //private String objectRequest
 
 
        [SerializeField] Material selected;

        private Vector3 offset, scale;
        private Quaternion additionalRotation;

        private Queue<WozniakInterfaces.HighlightObjectRequest> requestsQueue = new Queue<WozniakInterfaces.HighlightObjectRequest>();

        // Declare `foundObject` as a class-level variable
        private Transform foundObject;

        // UI elements
        private string instruction;
        public GameObject confetti;
        private GameObject canvas;
        public GameObject textCoordinates;

        public TextMeshProUGUI instructionText;
        public TextMeshProUGUI curCoordinate;
        public TextMeshProUGUI handCoordinate;
        public TextMeshProUGUI objCoordinate;

        public Hand hand;
        public Transform OVRCamera;
        private Pose currentPose;
        private HandJointId handStart = HandJointId.HandStart;

        void Start()
        {
            base.Start(); // Inicializa a classe base
            StartCoroutine(InitializeAfterSceneLoad());
            canvas = GameObject.Find("Canvas");
        }

        IEnumerator InitializeAfterSceneLoad()
        {
            yield return new WaitForSeconds(1f);
            InitializeProgram();
            sceneLoaded = true;
            Debug.Log("Scene loaded");
        }

        void InitializeProgram()
        {
            // Initialize variables
            sceneObjects = new string[] { obj1, obj2, obj3 };
            sceneGameObjects = new GameObject[totalObjects];
            additionalRotation = Quaternion.Euler(90, 0, 0);
            offset = new Vector3(0, 0.12f, 0);
            scale = new Vector3(0.15f, 0.15f, 0.15f);
        }

        protected override bool ServiceCallHandler(WozniakInterfaces.HighlightObjectRequest request, out WozniakInterfaces.HighlightObjectResponse response)
        {
            Debug.Log("REQUEST TARGET OBJECT:" + request.target_object);
            Debug.Log("REQUEST INSTRUCTION:" + request.instruction);
            instruction = request.instruction;

            response = new WozniakInterfaces.HighlightObjectResponse(); // Creating a new response 
            requestsQueue.Enqueue(request); // queueing requests

            response.message = "Request sent successfully"; // This will be done with 2 services. The first response is only to client to know that the request was done
            response.success = true;
            return true; 
        }

        private void Update()
        {
            while (requestsQueue.Count > 0)
            {
                var request = requestsQueue.Dequeue(); // dequeuing to process request
                StartCoroutine(ProcessRequest(request));
            }
            

            // Start the application by the keyboard
            if (Input.GetKeyDown(KeyCode.Space))
            {
                StartTask();
            }


            if (foundObject != null && foundObject.gameObject != null)
            {
                // Call to pick_object server
                if (foundObject.gameObject.GetComponent<Rigidbody>().isKinematic)
                {
                        
                    // Create a request for the `PickObject` service
                    var request = new PickObject.PickObjectRequest
                    {
                        //target_object = objectRequest // Define the name of the object to pick
                        target_object = "okay"
                    };

                    GetComponent<RosConnector>().RosSocket.CallService<PickObject.PickObjectRequest, PickObject.PickObjectResponse>("/pick_object", ServiceResponseHandler, request);

                    Destroy(foundObject.gameObject);
                }

            }

            hand.GetJointPose(handStart, out currentPose);
            Vector3 bonePosition = currentPose.position;
            Vector3 bonePositionWorld = hand.transform.TransformPoint(bonePosition);


            // Update Coordinates in canvas
            if (!done && started)
            {
                curCoordinate.text = OVRCamera.position.ToString();
                handCoordinate.text = bonePositionWorld.ToString();
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

        IEnumerator ProcessRequest(WozniakInterfaces.HighlightObjectRequest request)
        {
            yield return new WaitForSeconds(4);

            if (sceneLoaded)
            {

                instructionText.text = request.instruction;

                if (!started) 
                {
                    yield return null;
                }

                // Verify if the task has ended
                if (started && (request.target_object == "done" || request.target_object == "null"))
                {
                    done = true;

                    // Finish the program
                    curCoordinate.text = "--";
                    objCoordinate.text = "--";
                    handCoordinate.text = "--";

                    Instantiate(confetti, canvas.transform.position, Quaternion.identity);

                    yield return null;
                }
                else
                {
                    foundObject = GameObject.FindGameObjectWithTag(request.target_object)?.transform.root;

                    if (request == null)
                    {
                        Debug.Log("Request is null");
                        yield return null;
                    }
                    else if (string.IsNullOrEmpty(request.target_object))
                    {
                        Debug.Log("target_object is empty");
                        yield return null;
                    }

                    if (foundObject != null)
                    {
                        // If there is an object, highlight it with a red box
                        Material materialToApply = selected;
                        foundObject.GetChild(0).GetChild(0).gameObject.GetComponent<MeshRenderer>().material = materialToApply;

                        // Create a text component
                        GameObject text = Instantiate(textPrefab, foundObject.position, foundObject.rotation);
                        text.transform.rotation = text.transform.rotation * additionalRotation;
                        text.transform.position = text.transform.position + offset;
                        text.transform.localScale = scale;

                        text.gameObject.GetComponent<TextMeshPro>().text = request.target_object;
                        text.gameObject.transform.SetParent(foundObject, true);

                        Debug.Log($"Object '{request.target_object}' highlighted successfully");

                       // Update the canvas text with the object position

                        objCoordinate.text = foundObject.transform.position.ToString();

                        yield return null;
                    }
                    else
                    {
                        Debug.Log($"No object found with tag '{request.target_object}'");
                        yield return null;
                    }
                }
            }

            Debug.LogError("Scene not loaded");
            yield return null;
        }

        public void StartTask()
        {
            if (!started)
            {
                Debug.Log("I am ready to start.");

                // Criação da requisição para o serviço `PickObject`
                var request = new PickObject.PickObjectRequest
                {
                    target_object = "I am ready" // Defina o nome do objeto a ser pego
                };

                GetComponent<RosConnector>().RosSocket.CallService<PickObject.PickObjectRequest, PickObject.PickObjectResponse>("/pick_object", ServiceResponseHandler, request);

                textCoordinates.SetActive(true);

                started = true;
            }
        }


    }       
    
}*/
