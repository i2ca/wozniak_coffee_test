using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DestroyInteractable : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

        if(this.GetComponent<Rigidbody>().isKinematic)
        {
             Destroy(this.gameObject);
        }
        
    }


}
