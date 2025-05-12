using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Meta.XR.MRUtilityKit;

public class SpawnInteractables : MonoBehaviour
{
    public GameObject interactablePrefab;
    private static int totalObjects = 3;

    [SerializeField] string obj1 = "coffe maker", obj2 = "cup", obj3 = "sweetener";
    [SerializeField] string anchor_name = "OTHER";

    private string[] sceneObjects;


    // Start is called before the first frame update
    void Start()
    {
        sceneObjects = new string[] { obj1, obj2, obj3 };

        StartCoroutine(InitializeAfterSceneLoad());
    }

    IEnumerator InitializeAfterSceneLoad()
    {
        yield return new WaitForSeconds(1f);
        InitializeObjects();
    }

    void InitializeObjects()
    {
        var rooms = MRUK.Instance.GetCurrentRoom().Anchors;
        int i = 0;

        foreach (var anchor in rooms)
        {
            //anchor.gameObject.layer = LayerMask.NameToLayer("Hands");
            //anchor.gameObject.transform.GetChild(0).gameObject.layer = LayerMask.NameToLayer("Hands");
        }

        foreach (var anchor in rooms)
        {
            if (i == totalObjects) break;

            if (anchor.gameObject.name == anchor_name && anchor.gameObject.tag == "Untagged")
            {
                // Tag object
                anchor.gameObject.transform.GetChild(0).gameObject.tag = sceneObjects[i];
                anchor.gameObject.tag = sceneObjects[i];

                // Spawn interactive box
                Vector3 anchorCenter = anchor.GetAnchorCenter();
                Vector3 anchorSize = anchor.GetAnchorSize();
                Vector3 instancePosition = anchorCenter;

                GameObject instance = Instantiate(interactablePrefab, instancePosition, anchor.transform.rotation);
                instance.transform.localScale = anchorSize;
                instance.gameObject.tag = sceneObjects[i];
                instance.gameObject.name = sceneObjects[i];

                anchor.transform.SetParent(instance.transform, true);

                i++;
            }
        }
    }


}
