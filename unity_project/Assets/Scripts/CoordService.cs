using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Coord = RosSharp.RosBridgeClient.MessageTypes.Coord; 
using Newtonsoft.Json;


namespace RosSharp.RosBridgeClient.MessageTypes
{
    public class CoordService : UnityServiceProvider<Coord.CoordRequest, Coord.CoordResponse>
    {
        public GameObject ball;
        public Transform OVR;
        public GameObject fakeObj;

        private string[] sceneObjects;

        //private String objectRequest

        [Range(0f, 1f)]
        public float offsetX;

        [Range(0f, 1f)]
        public float offsetY;

        [Range(0f, 1f)]
        public float offsetZ;

        private Queue<Coord.CoordRequest> requestsQueue = new Queue<Coord.CoordRequest>();

        // Declare `foundObject` as a class-level variable
        private Transform foundObject;

        void Start()
        {
            base.Start(); // Inicializa a classe base
        }

        protected override bool ServiceCallHandler(Coord.CoordRequest request, out Coord.CoordResponse response)
        {
            Debug.Log("REQUEST X:" + request.x);
            Debug.Log("REQUEST Y:" + request.y);
            Debug.Log("REQUEST Z:" + request.z);

            float coordX = request.x;

            response = new Coord.CoordResponse(); // Creating a new response 
            requestsQueue.Enqueue(request); // queueing requests

            response.success = true;
            return true; 
        }

        private void Update()
        {
            while (requestsQueue.Count > 0)
            {
                var request = requestsQueue.Dequeue(); // dequeuing to process request
                ProcessRequest(request);
            }
            
        }

        private void ProcessRequest(Coord.CoordRequest request)
        {
            Vector3 posReceived = new Vector3(request.x, -request.y, request.z);

            Vector3 newPos = posReceived + new Vector3(-offsetX, -offsetY, offsetZ);

            fakeObj.transform.localPosition = newPos;

            ball.transform.position = fakeObj.transform.position;
        }


    }       
    
}
