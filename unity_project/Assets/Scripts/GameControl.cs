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


public class GameControl : MonoBehaviour
{
    // Control variables 
    bool started = true;
    bool done = false;


    // Scene objects
    public GameObject redDot;
    public GameObject confetti;

    // Hand parameters
    public Hand hand;
    public Transform OVRCamera;
    private Vector3 handPosition;

    // Scripts
    [SerializeField] CanvasHandler canvas;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        // Updante hand position 
        UpdateHandParameters();
        
        // Update Coordinates in canvas
        if (!done && started)
        {
            canvas.UpdatePlayerCoordinate(OVRCamera.transform.position);
            canvas.UpdateHandCoordinate(handPosition);
            canvas.UpdateObjectCoordinate(redDot.transform.position);
        }
    }

    void UpdateHandParameters()
    {
        Pose currentPose;
        HandJointId handStart = HandJointId.HandStart;
        hand.GetJointPose(handStart, out currentPose);
        Vector3 bonePosition = currentPose.position;
        Vector3 bonePositionWorld = hand.transform.TransformPoint(bonePosition);

        handPosition = bonePositionWorld;
    }

}
