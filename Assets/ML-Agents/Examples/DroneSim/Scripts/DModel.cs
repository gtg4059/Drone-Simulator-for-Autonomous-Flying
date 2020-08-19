using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Linq;
using System.Text;
using System.IO;

public class DModel : MonoBehaviour
{

    //public GameObject objectToMove;

    DIntegrator theIntegrator;
    public float[] Control = new float[4];
    public DroneRotor f1;
    public DroneRotor f2;
    public DroneRotor f3;
    public DroneRotor f4;
    float t = 0.0f;
    float h = 0.01f;
    List<float[]> data;
    Vector3 velocity;
    Vector3 previous;
    Vector3 pos, rot;
    Rigidbody rb;
    float[] Xsave;
    bool CollisionEnter = false, StartTrigger = false, ResetTrigger = false;
    Vector3 FixedVelocity;
    Quaternion rotq;
    public Transform ColliderTransform;
    float innertimer = 0;
    void Start()
    {
        theIntegrator = new DIntegrator();
        rb = gameObject.GetComponent<Rigidbody>();
        theIntegrator.m = 0.468f;
        theIntegrator.l = 0.225f;
        theIntegrator.Ax = 0.25f;
        theIntegrator.Ay = 0.25f;
        theIntegrator.Az = 0.25f;
        theIntegrator.Ixx = 0.004856f;
        theIntegrator.Iyy = 0.004856f;
        theIntegrator.Izz = 0.008801f;
        theIntegrator.k = 0.00000298f;
        theIntegrator.b = 0.000000114f;
        //gameObject.transform.localScale = new Vector3(theIntegrator.l / 0.225f, theIntegrator.l / 0.225f, theIntegrator.l / 0.225f);
        gameObject.transform.Translate(Vector3.up * theIntegrator.l / 0.45f);
        float[] x0 = new float[12] { 0, 0, 0
            , 0, 0, 0, 0, 0,
            0, 0, 0, 0 };
        //gameObject.transform.position = new Vector3(UIButton.posX, UIButton.posZ+0.5f, UIButton.posY);
        //gameObject.transform.rotation = Quaternion.Euler(0, UIButton.rotZ, 0);
        Xsave = x0;
        theIntegrator.SetIC(x0);
    }

    void FixedUpdate()
    {
        //play runge-kutta method
        t = theIntegrator.RK4Step(theIntegrator.X, t, h);
        pos = gameObject.transform.position;
        velocity = (pos - previous) / Time.deltaTime; //get velocity at Scene ifself
        previous = pos;
        //verify position, rotation
        pos.x = theIntegrator.X[0];
        pos.z = theIntegrator.X[1];
        pos.y = theIntegrator.X[2];
        rot = gameObject.transform.rotation.eulerAngles;
        rot.x = -theIntegrator.X[6] * 180 / Mathf.PI;
        rot.z = -theIntegrator.X[7] * 180 / Mathf.PI;
        rot.y = -theIntegrator.X[8] * 180 / Mathf.PI;
        rotq = Quaternion.Euler(rot.x, rot.y, rot.z);
        //verify Input
        theIntegrator.z_d += Control[0];
        theIntegrator.phi_d = Control[1];
        theIntegrator.theta_d = Control[2];
        theIntegrator.psi_d += Control[3];
        gameObject.transform.position = pos;
        gameObject.transform.rotation = rotq;

        //verify Rotor Angular Velocity(Power)
        /*f1.setPower(theIntegrator.w1);
        f2.setPower(theIntegrator.w2);
        f3.setPower(theIntegrator.w3);
        f4.setPower(theIntegrator.w4);    */

    }

}
