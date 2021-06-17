using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class followZED : MonoBehaviour
{   
    public Transform ZED;
    public Transform left;
    public Transform right;

    int x;
    int y;

    // Start is called before the first frame update
    void Start()
    {   
        x=0;
        y=0;
    }

     protected void rotateTowards(Vector3 to) {
 
     Quaternion _lookRotation = 
         Quaternion.LookRotation((to - transform.position).normalized);
     
    //  //over time
    //  transform.rotation = 
    //      Quaternion.Slerp(transform.rotation, _lookRotation, Time.deltaTime * turn_speed);
     
     //instant
     transform.rotation = _lookRotation;
 }

    // Update is called once per frame
    void Update()
    {
        //Vector3 center = ((LeftFrame.position - RightFrame.position)/2.0f) + RightFrame.position;
        //transform.LookAt(center);
        // x++;
        // if(x%120==0 && y < 2) {
        //         y++;
        //         Vector3 center = (left.position + right.position)/2;
        //         Vector3 delta = center - right.position;
        //         right.position = right.position + delta/2;
        //         left.position = left.position - delta/2;
            
        // }
        transform.LookAt((left.position + right.position)/2);
        transform.position = (left.position + right.position)/2 - Quaternion.Inverse(Quaternion.Lerp(left.rotation, right.rotation, 0.5f)) * (Vector3.forward* 0.3f);
        //transform.position = LeftFrame.position - Quaternion.Inverse(transform.rotation) * (Vector3.forward* 0.2f);
        //transform.position = (left.position + right.position)/2;
        //transform.position = ZED.position - (Quaternion.Inverse(transform.rotation) * (Vector3.forward * 0.2f));
        //Vector3 direction = (center - transform.position).normalized;
        //transform.position = ZED.position;
        

    }
    
}


/*
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class followZED : MonoBehaviour
{   
    public Transform ZED;

    // Start is called before the first frame update
    void Start()
    {   
        Debug.Log("Following ZED");
    }

    // Update is called once per frame
    void Update()
    {
        //Vector3 center = ((LeftFrame.position - RightFrame.position)/2.0f) + RightFrame.position;
        //transform.LookAt(center);
        //transform.position = (LeftFrame.position + RightFrame.position)/2 - Quaternion.Inverse(transform.rotation) * (Vector3.forward* 0.2f);
        //transform.position = LeftFrame.position - Quaternion.Inverse(transform.rotation) * (Vector3.forward* 0.2f);
        transform.rotation = ZED.rotation;
        transform.position = ZED.position;
        //transform.position = ZED.position - (Quaternion.Inverse(transform.rotation) * (Vector3.forward * 0.2f));
        //Vector3 direction = (center - transform.position).normalized;
        //transform.position = ZED.position;
        

    }
    
}
*/