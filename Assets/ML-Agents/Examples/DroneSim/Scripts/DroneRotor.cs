using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneRotor : MonoBehaviour {
    Rigidbody rBody;
    public float power;
    SHOIntegrator theIntegrator;
    /// <summary>
    /// Specify the verse of the rotation
    /// <para> Set this in the editor
    /// </summary>
    public bool counterclockwise;
    

    // Use this for initialization
    void Start () {
        theIntegrator = new SHOIntegrator();
        Transform t = this.transform;
        while (t.parent != null && t.tag != "Player") t = t.parent;
        rBody = t.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update() { transform.Rotate(0, 0, power * (counterclockwise ? -1 : 1)); }

    /// <summary>
    /// Sets the rotating power of the rotor
    /// </summary>
    /// <param name="intensity"> The rotating power of the rotor </param>
    public void setPower(float intensity) { power = intensity; }

    void FixedUpdate()
    {
        /*rBody.AddForceAtPosition(transform.forward * theIntegrator.k * power*power, transform.position);
        if (counterclockwise) rBody.AddTorque(transform.forward * theIntegrator.b * power * power,ForceMode.Force);
        else rBody.AddTorque(-1*transform.forward * theIntegrator.b * power * power, ForceMode.Force);*/
        //rBody.AddTorque()
        //lr.SetPosition(1, new Vector3(0, 0, power / 3f));
    }
}
