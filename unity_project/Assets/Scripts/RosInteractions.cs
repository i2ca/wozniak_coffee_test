using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Meta.XR.MRUtilityKit;
using TMPro;

public class RosInteractions : MonoBehaviour
{
    [SerializeField] string obj1 = "Xicara", obj2 = "Cafe", obj3 = "Cafeteira";
    int totalObjects = 3;

    private string[] sceneObjects;
    private GameObject[] sceneGameObjects;
    private bool sceneLoaded = false;


    public GameObject textPrefab;

    [SerializeField] Material selected;

    private Vector3 offset, scale;
    private Quaternion additionalRotation;

        // Ros components
    public GameObject subscriberHolder;
    private RosSharp.RosBridgeClient.StringSubscriber subscriber;
    private string stringAnterior;


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
    }

    void InitializeProgram()
    {
        // Initialize variables
        sceneObjects = new string[] { obj1, obj2, obj3 };

        sceneGameObjects = new GameObject[totalObjects];


        additionalRotation = Quaternion.Euler(90, 0, 0);
        offset = new Vector3(0, 0.1f, 0);
        scale = new Vector3(0.3f, 0.3f, 0.3f);

        subscriber = subscriberHolder.GetComponent<RosSharp.RosBridgeClient.StringSubscriber>();
        stringAnterior = "";
    }

    void Update()
    {
        if (sceneLoaded)
        {

            if(subscriber.receivedString != stringAnterior)
            {
                CubeRed(subscriber.receivedString);
                stringAnterior = subscriber.receivedString;
            }
            
            /*
            for (int i = 0; i < totalObjects; i++)
            {
                UpdateTextPositionAndRotation(sceneObjects[i], buttons[i].isOn, texts[i]);
            }
            */
        }
    }


public void CubeRed(string stringAux)
{

        Transform foundObject = GameObject.FindGameObjectWithTag(stringAux).transform.root;

        Debug.Log(foundObject);
        if(foundObject != null)
        {
            Material materialToApply = selected;
            foundObject.GetChild(1).GetChild(0).gameObject.GetComponent<MeshRenderer>().material = materialToApply;
            
            //create a text component
            GameObject text = Instantiate(textPrefab, foundObject.position, foundObject.rotation);
            text.transform.rotation = text.transform.rotation * additionalRotation;
            text.transform.position = text.transform.position + offset;
            text.transform.localScale = scale;

            text.gameObject.GetComponent<TextMeshPro>().text = stringAux;

            text.gameObject.transform.SetParent(foundObject,true);
        } 
        
                
        
}

/*
    void UpdateTextPositionAndRotation(string tag, bool isOn, GameObject textObject)
    {
        GameObject foundObject = GameObject.FindGameObjectWithTag(tag);

        if (foundObject != null && textObject != null)
        {
            if (isOn)
            {
                textObject.transform.rotation = foundObject.transform.rotation * additionalRotation;
                textObject.transform.position = foundObject.transform.position + offset;
                textObject.transform.localScale = scale;
            }
        }
        else
        {
            Debug.LogWarning("Game object nulo informado");
        }
    }

    void UpdateText()
    {
        for (int i = 0; i < totalObjects; i++)
        {
            TextMeshPro canvaText = texts[i].GetComponent<TextMeshPro>();
            TextMeshProUGUI buttonText = buttons[i].GetComponentInChildren<TextMeshProUGUI>();

            // Modify button text
            if (buttonText != null)
            {
                buttonText.text = sceneObjects[i];
            }

            // Modify canva text
            if (canvaText != null)
            {
                canvaText.text = sceneObjects[i];
            }
        }
    }
*/
    

}
