using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using System.Linq;
using System.Text;
using System.IO;

public class SHOModel : MonoBehaviour
{

    //public GameObject objectToMove;

    SHOIntegrator theIntegrator;
    /*public float M1 = 0.02f, M2 = 0.2f, L1 = 0.1f, L2 = 0.2f, g = 9.8f,
        J1 = 3.33f * Mathf.Pow(10, -5), J2= 1.00f * Mathf.Pow(10, -3);*/

    float t = 0.00f;
    float h = 0.02f;
    public GameObject Wheel;
    float[] x0 = { 0, 0, 0 };
    Vector3 Wrot, Hrot;
    float wheelangle;
    public float MotorInput, Angle, AngularVel, WheelSpeed;

    public void Reset()
    {
        //theIntegrator = new SHOIntegrator();
        x0[0] = Random.Range(-1 * Mathf.PI, Mathf.PI);//Mathf.PI;
        x0[1] = Random.Range(-2f, 2f);
        x0[2] = 0;
        theIntegrator = new SHOIntegrator();
        theIntegrator.SetIC(x0);
        Angle = x0[0]; AngularVel = 0;
    }
    void Start()
    {
        x0[0] = Random.Range(-1 * Mathf.PI, Mathf.PI);//Mathf.PI;
        x0[1] = Random.Range(-2f, 2f);
        x0[2] = 0;
        theIntegrator = new SHOIntegrator();
        theIntegrator.SetIC(x0);
    }

    void FixedUpdate()
    {
        //X0=q1 X1=q1dot X2=q2dot
        //theIntegrator.X[0] + wheelangle
        //play runge-kutta method
        theIntegrator.torque = MotorInput; //import torque
        t = theIntegrator.RK4Step(theIntegrator.X, t, h);
        this.gameObject.transform.rotation = Quaternion.Euler(0, 180, theIntegrator.X[0] * 180 / Mathf.PI);
        //Debug.Log((theIntegrator.X[0] + wheelangle) * 180 / Mathf.PI);
        Wheel.transform.Rotate(new Vector3(0, theIntegrator.X[2], 0));
        float turnangle = (theIntegrator.X[0]) % (Mathf.PI * 2);
        if (turnangle < -1 * Mathf.PI) turnangle = Mathf.Abs(turnangle + Mathf.PI * 2);
        else if (turnangle > Mathf.PI) turnangle = Mathf.Abs(turnangle - Mathf.PI * 2);
        Angle = Mathf.Abs(turnangle); //Hinge Angle
        AngularVel = theIntegrator.X[1]; //Hinge angular velocity
        WheelSpeed = theIntegrator.X[2] * Time.deltaTime; //Hinge angular velocity(rad/s)
    }
    // Update is called once per frame
    void Update()
    {

    }

}
