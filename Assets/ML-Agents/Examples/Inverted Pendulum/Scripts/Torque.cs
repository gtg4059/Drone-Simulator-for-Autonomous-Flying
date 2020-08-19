using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Torque : MonoBehaviour
{
    Rigidbody WheelEffect;
    int a;
    //public Rigidbody Wheel;
    // Start is called before the first frame update
    void Start()
    {
        Transform t = this.transform;
        WheelEffect = t.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    void FixedUpdate()
    {
        
        WheelEffect.AddTorque(transform.forward*20, ForceMode.Force);
    }
}
