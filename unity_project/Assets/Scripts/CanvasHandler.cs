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

public class CanvasHandler : MonoBehaviour
{
    // Control variables 
/*    bool started = false;
    bool done = false;*/

    // Scene objects
    //private string instruction;
    //public GameObject confetti;

    // Canvas text
    public TextMeshProUGUI instructionText;
    public TextMeshProUGUI curCoordinate;
    public TextMeshProUGUI handCoordinate;
    public TextMeshProUGUI objCoordinate;

    public void UpdateObjectCoordinate(Vector3? coordinate)
    {
        if(coordinate == null)
        {
            objCoordinate.text = "--";
        }
        else
        {
            objCoordinate.text = coordinate.ToString();
        }
    }

    public void UpdateHandCoordinate(Vector3? coordinate)
    {
        if (coordinate == null)
        {
            handCoordinate.text = "--";
        }
        else
        {
            handCoordinate.text = coordinate.ToString();
        }
    }

    public void UpdatePlayerCoordinate(Vector3? coordinate)
    {
        if (coordinate == null)
        {
            curCoordinate.text = "--";
        }
        else
        {
            curCoordinate.text = coordinate.ToString();
        }
    }

    public void UpdateInstruction(string newInstruction)
    {
        instructionText.text = newInstruction;
    }
}
