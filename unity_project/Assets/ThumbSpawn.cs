using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ThumbSpawn : MonoBehaviour
{
    public GameObject prefab;
    public Transform reference;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void SpawnSphere()
    {
        Instantiate(prefab, reference.position, Quaternion.identity);
    }
}
