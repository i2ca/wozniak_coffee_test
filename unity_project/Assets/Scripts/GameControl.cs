using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Meta.XR.MRUtilityKit;
using RosSharp.RosBridgeClient;
using WozniakInterfaces = RosSharp.RosBridgeClient.MessageTypes.WozniakInterfaces;
using WozniakServiceHandle = RosSharp.RosBridgeClient.MessageTypes;
using Oculus.Interaction.Input;


public class GameControl : MonoBehaviour
{
    // Control variables 
    private bool started = true;
    private bool done = false;

    // Scene objects
    [SerializeField] GameObject redDot;
    [SerializeField] GameObject confetti;
    [SerializeField] Transform canvas;

    // Oculus parameters
    [SerializeField] Hand hand;
    [SerializeField] Transform OVRCamera;
    [SerializeField] Transform OVRCamera_Ghost;
    private Vector3 handPosition;

    // Scripts
    [SerializeField] CanvasHandler _canvas;
    [SerializeField] WozniakServiceHandle.TriggerLLMService _triggerllm;

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
            _canvas.UpdatePlayerCoordinate(OVRCamera.transform.position);
            _canvas.UpdateHandCoordinate(handPosition);
            _canvas.UpdateObjectCoordinate(redDot.transform.position);
        }

        // You can also start the application using the keyboard
        if (Input.GetKeyDown(KeyCode.Space))
        {
            NextTask();
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

    public void SetGameStart(bool state)
    {
        started = state;
    }

    public bool GetGameStart()
    {
        return started;
    }

    public void SetGameDone(bool state)
    {
        done = state;
    }

    public bool GetGameDone()
    {
        return done;
    }

    public void NextTask()
    {
        Debug.Log("Next task chamada");

        CopyCenterEyePosition();

        if (!started)
        {
            Debug.Log("I am ready to start.");

            string message = "I am ready to start";

            _triggerllm.TriggerLLMSendRequest(message);

            started = true;
        }
        else
        {
            string message = "Okay";

            Debug.Log("Message sent:" + message);

            _triggerllm.TriggerLLMSendRequest(message);
        } 
        
    }
    
    private void CopyCenterEyePosition()
    {
        OVRCamera_Ghost.position = OVRCamera.position;
        OVRCamera_Ghost.rotation = OVRCamera.rotation;
    }

    public void FinishGame()
    {
        _canvas.UpdateHandCoordinate(null);
        _canvas.UpdatePlayerCoordinate(null);
        _canvas.UpdateObjectCoordinate(null);

        Instantiate(confetti, canvas.transform.position, Quaternion.identity);

        done = true;
    }

}
