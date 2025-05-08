using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Meta.XR.MRUtilityKit;
using TMPro;

public class ButtonInteractions : MonoBehaviour
{
    [SerializeField] string obj1 = "Xicara", obj2 = "Cafe", obj3 = "Cafeteira";
    int totalObjects = 3;

    private string[] sceneObjects;
    private GameObject[] sceneGameObjects;
    private bool sceneLoaded = false;


    [SerializeField] Material selected, notselected;

    private GameObject[] texts;
    private GameObject[] toggleVector;
    private Toggle[] buttons;

    private Vector3 offset, scale;
    private Quaternion additionalRotation;

    void Start()
    {
        StartCoroutine(InitializeAfterSceneLoad());
    }

    IEnumerator InitializeAfterSceneLoad()
    {
        yield return new WaitForSeconds(1f);
        InitializeProgram();
        UpdateText();
        sceneLoaded = true;
    }

    void InitializeProgram()
    {
        // Initialize variables
        sceneObjects = new string[] { obj1, obj2, obj3 };

        sceneGameObjects = new GameObject[totalObjects];

        // Assign texts and buttons to their respective arrays
        texts = GameObject.FindGameObjectsWithTag("Text");
        toggleVector = GameObject.FindGameObjectsWithTag("Toggle");
        buttons = new Toggle[totalObjects];

        // Set the buttons
        for (int i = 0; i < totalObjects; i++)
        {
            buttons[i] = toggleVector[i].GetComponent<Toggle>();
        }

        // Funcoes de evento para os botoes
        buttons[0].onValueChanged.AddListener(Obj1ToggleValueChanged);
        buttons[1].onValueChanged.AddListener(Obj2ToggleValueChanged);
        buttons[2].onValueChanged.AddListener(Obj3ToggleValueChanged);

        additionalRotation = Quaternion.Euler(90, 0, 0);
        offset = new Vector3(0, 0.1f, 0);
        scale = new Vector3(0.3f, 0.3f, 0.3f);
    }

    void Update()
    {
        if (sceneLoaded)
        {
            for (int i = 0; i < totalObjects; i++)
            {
                UpdateTextPositionAndRotation(sceneObjects[i], buttons[i].isOn, texts[i]);
            }
        }
    }


    public void Obj1ToggleValueChanged(bool isOn)
    {
        foreach (var anchor in MRUK.Instance.GetCurrentRoom().Anchors)
        {
            if (anchor.gameObject.tag == sceneObjects[0])
            {
                Material materialToApply = isOn ? selected : notselected;
                anchor.gameObject.transform.GetChild(0).gameObject.GetComponent<MeshRenderer>().material = materialToApply;
                texts[0].GetComponent<MeshRenderer>().enabled = isOn;
            }
        }
    }

    public void Obj2ToggleValueChanged(bool isOn)
    {
        foreach (var anchor in MRUK.Instance.GetCurrentRoom().Anchors)
        {
            if (anchor.gameObject.tag == sceneObjects[1])
            {
                Material materialToApply = isOn ? selected : notselected;
                anchor.gameObject.transform.GetChild(0).gameObject.GetComponent<MeshRenderer>().material = materialToApply;

                texts[1].GetComponent<MeshRenderer>().enabled = isOn;
            }
        }
    }

    public void Obj3ToggleValueChanged(bool isOn)
    {
        foreach (var anchor in MRUK.Instance.GetCurrentRoom().Anchors)
        {
            if (anchor.gameObject.tag == sceneObjects[2])
            {
                Material materialToApply = isOn ? selected : notselected;
                anchor.gameObject.transform.GetChild(0).gameObject.GetComponent<MeshRenderer>().material = materialToApply;
                texts[2].GetComponent<MeshRenderer>().enabled = isOn;
            }
        }

    }

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

}
