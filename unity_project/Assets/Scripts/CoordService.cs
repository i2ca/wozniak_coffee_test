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

        private Queue<Coord.CoordRequest> requestsQueue = new Queue<Coord.CoordRequest>();

        // Red dot calibration offset 
        [Range(-1f, 1f)]
        public float offsetX;

        [Range(-1f, 1f)]
        public float offsetY;

        [Range(-1f, 1f)]
        public float offsetZ;


        // Scripts 
        [SerializeField] CanvasHandler _canvas;

        void Awake()
        {
            Debug.Log("CoordService: Awake chamado");
        }


        void Start()
        {
            Debug.Log("CoordService: Start chamado");
            base.Start(); // Inicializa a classe base
        }

        protected override bool ServiceCallHandler(Coord.CoordRequest request, out Coord.CoordResponse response)
        {
            Debug.Log("REQUEST COORDINATES:" + "(" + request.x + ", " + request.y + ", " + request.z + ");");

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
            // Update red dot position
            Vector3 posReceived = new Vector3(request.x, -request.y, request.z);

            Vector3 offset = new Vector3(-offsetX, -offsetY, offsetZ);

            Vector3 newPos = posReceived + offset;

            fakeObj.transform.localPosition = newPos;

            ball.transform.position = fakeObj.transform.position;

            _canvas.UpdateObjectCoordinate(ball.transform.position);
        }

    }       
    
}
