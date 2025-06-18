using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using Coord = RosSharp.RosBridgeClient.MessageTypes.Coord;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace RosSharp.RosBridgeClient.MessageTypes
{
    public class CoordService : UnityServiceProvider<Coord.CoordRequest, Coord.CoordResponse>
    {
        [Header("Object Creation")]
        public GameObject ballPrefab;
        private GameObject currentBall;

        [Header("Manual Offsets")]
        [Tooltip("Offset de Translação no eixo X")]
        [Range(-1f, 1f)]
        public float offsetX = 0f;

        [Tooltip("Offset de Translação no eixo Y")]
        [Range(-1f, 1f)]
        public float offsetY = 0f;

        [Tooltip("Offset de Translação no eixo Z")]
        [Range(-1f, 1f)]
        public float offsetZ = 0f;

        [Tooltip("Offset de Rotação (Roll) em graus")]
        [Range(-180f, 180f)]
        public float offsetRoll = 0f;

        [Tooltip("Offset de Rotação (Pitch) em graus")]
        [Range(-180f, 180f)]
        public float offsetPitch = 0f;

        [Tooltip("Offset de Rotação (Yaw) em graus")]
        [Range(-180f, 180f)]
        public float offsetYaw = 0f;

        private Queue<Coord.CoordRequest> requestsQueue = new Queue<Coord.CoordRequest>();
        public UnityEngine.Transform OVR; // Referência ao OVRCameraRig ou similar

        void Start()
        {
            base.Start();
            
            if (ballPrefab == null)
            {
                Debug.LogError("O prefab da bolinha (ballPrefab) não foi atribuído no Inspector!");
            }
            if (OVR == null)
            {
                Debug.LogError("A referência do OVR (câmera do headset) não foi atribuída no Inspector!");
            }
        }

        protected override bool ServiceCallHandler(Coord.CoordRequest request, out Coord.CoordResponse response)
        {
            Debug.Log($"Request received: X={request.x:F3}, Y={request.y:F3}, Z={request.z:F3}");

            response = new Coord.CoordResponse { success = true };
            requestsQueue.Enqueue(request);

            return true; 
        }

        private void Update()
        {
            // Processa a fila de requisições.
            while (requestsQueue.Count > 0)
            {
                var request = requestsQueue.Dequeue();
                ProcessRequest(request);
            }
        }

        private void ProcessRequest(Coord.CoordRequest request)
        {
            // Cria a bolinha na posição recebida, aplicando os offsets manuais.
            CreateBallAtPosition(request, offsetX, offsetY, offsetZ, offsetRoll, offsetPitch, offsetYaw);
        }

        // Função que calcula a posição final e cria a bolinha
        private void CreateBallAtPosition(Coord.CoordRequest request, float oX, float oY, float oZ, float oRoll, float oPitch, float oYaw)
        {
            if (ballPrefab == null || OVR == null) return;
            
            // Destrói a bolinha anterior, se existir.
            if (currentBall != null)
            {
                Destroy(currentBall);
            }
            
            // Converte as coordenadas recebidas do ROS para o sistema do Unity.
            UnityEngine.Vector3 posReceived = new UnityEngine.Vector3(request.x, -request.y, request.z);

            // Calcula a posição e rotação final no sistema de coordenadas local do OVR
            UnityEngine.Vector3 finalLocalPos = CalculateFinalPosition(posReceived, oX, oY, oZ, oRoll, oPitch, oYaw);
            UnityEngine.Quaternion finalLocalRot = UnityEngine.Quaternion.Euler(-oRoll, -oPitch, oYaw);

            // Converte a posição e rotação local para o sistema de coordenadas global do Unity
            UnityEngine.Vector3 finalWorldPos = OVR.TransformPoint(finalLocalPos);
            UnityEngine.Quaternion finalWorldRot = OVR.rotation * finalLocalRot;

            // Instancia a nova bolinha e a armazena como a bolinha atual
            currentBall = Instantiate(ballPrefab, finalWorldPos, finalWorldRot);
            currentBall.SetActive(true);
            
            Debug.Log($"[INFO] Nova bolinha criada na posição global: {finalWorldPos.ToString("F4")}");
        }

        private UnityEngine.Vector3 CalculateFinalPosition(UnityEngine.Vector3 receivedPos, float oX, float oY, float oZ, float oRoll, float oPitch, float oYaw)
        {
            UnityEngine.Quaternion rotationOffset = UnityEngine.Quaternion.Euler(-oRoll, -oPitch, oYaw);
            UnityEngine.Vector3 translationOffset = new UnityEngine.Vector3(-oX, -oY, oZ);
            
            // Aplica primeiro a rotação e depois a translação
            UnityEngine.Vector3 rotatedPos = rotationOffset * receivedPos;
            return rotatedPos + translationOffset;
        }

        public void ClearAllBalls()
        {
            if (currentBall != null)
            {
                Destroy(currentBall);
                currentBall = null;
            }
        }
    }       
}