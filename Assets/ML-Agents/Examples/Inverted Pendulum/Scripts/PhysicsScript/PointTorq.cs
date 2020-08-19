using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointTorq : MonoBehaviour
{
    public float torque;
    Rigidbody rb;
    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        rb.AddRelativeForce(new Vector3(torque, 0,0));
    }
}
