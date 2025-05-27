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
    bool started = true;
    bool done = false;

    // Scene objects
    public GameObject redDot;
    public GameObject confetti;
    public Transform canvas;

    // Hand parameters
    public Hand hand;
    public Transform OVRCamera;
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

    public void ProcessScene(string obj, string instruction)
    {
        if (!started) return;
        
        if (obj == "done") FinishGame();
        else _canvas.UpdateInstruction(instruction);
    }

    public void NextTask()
    {
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

            _triggerllm.TriggerLLMSendRequest(message);
        } 
        
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
