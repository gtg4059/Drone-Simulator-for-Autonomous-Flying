using UnityEngine;
using MLAgents;
using System.Linq;
using System.Text;
using System.Collections;
using System.Collections.Generic;

public class DroneAgent : Agent
{
    DIntegrator theIntegrator;
    public Transform goal, Axis;
    public List<Transform> obs;
    public Vector3 Drone;
    public float[] Control = new float[2];
    public DroneRotor f1;
    public DroneRotor f2;
    public DroneRotor f3;
    public DroneRotor f4;
    float t = 0.0f;
    float h = 0.01f;
    Vector3 velocity;
    Vector3 previous;
    Vector3 pos, rot, alterpos;
    Rigidbody rb;
    float[] Xsave;
    Vector3 FixedVelocity;
    Quaternion rotq;
    public Transform ColliderTransform;
    float LidarRange = 12f, DownRange = 5, ProxPanalty = 0.45f, predist, curdist;
    /*public float DRayInform;
    public float[] RayInformContain;
    private int RotCount = 1;*/
    public float DRayInform, cmlReward, time = 0;
    List<float> RayInformContain;
    private int RotCount = 1, AngleDistrib = 12;
    bool collisiontrigger = false, randomspwn = true;
    public MazeSpawner Maze = null;
    protected float targetDistance;
    protected Vector3 targetDirection;
    protected float targetPolarAngle;
    protected float DSpeedMag;
    private int decisionInterval;
    protected int crntStep;
    private bool isOutOfBounds => pos.x < 0 || pos.x > 3.23 || pos.z < 0 || pos.z > 7.2;

    public override void InitializeAgent()
    {

        base.InitializeAgent();
        theIntegrator = new DIntegrator();
        rb = gameObject.GetComponent<Rigidbody>();
        Drone = gameObject.transform.position;
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
        //gameObject.transform.Translate(Vector3.up * theIntegrator.l / 0.45f);
        float[] x0 = new float[12] { 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        //gameObject.transform.position = new Vector3(UIButton.posX, UIButton.posZ+0.5f, UIButton.posY);
        //gameObject.transform.rotation = Quaternion.Euler(0, UIButton.rotZ, 0);
        Xsave = x0;
        theIntegrator.SetIC(x0);
        /*foreach (GameObject obs in Obslist) obs.transform.position = new Vector3(Drone.x + obs.transform.position.x
                , Drone.y + obs.transform.position.y, Drone.z + obs.transform.position.z);*/
        RayInformContain = new List<float>();
    }


    public override void CollectObservations()
    {
        base.CollectObservations();
        RayInformContain = new List<float>();
        RaycastHit hit, Dhit;
        for (int i = 0; i < AngleDistrib; i++)
        {
            Quaternion rotation = Quaternion.AngleAxis(i * (360 / AngleDistrib), transform.up);
            //Ray Ray = new Ray(transform.position + Vector3.up * 0.3f, rotation * this.transform.right * 2);
            Ray Ray = new Ray(transform.position + Vector3.up * 0.4f, rotation * this.transform.right);
            if (Physics.Raycast(Ray, out hit, LidarRange))
            {
                if (hit.distance < ProxPanalty) Debug.DrawRay(transform.position + Vector3.up * 0.4f,
                       rotation * this.transform.right * hit.distance, Color.gray);
                else Debug.DrawRay(transform.position + Vector3.up * 0.4f,
                    rotation * this.transform.right * hit.distance, Color.green);
                RayInformContain.Add(1f - hit.distance / LidarRange);
            }
            else RayInformContain.Add(-1f);
        }
        Quaternion rot = Quaternion.AngleAxis(0, transform.up);
        //Ray DRay = new Ray(transform.position + Vector3.down * 0.4f, this.transform.up * -1);
        //if (Physics.Raycast(DRay, out Dhit, DownRange)) DRayInform = 1f - Dhit.distance / DownRange;
        //else DRayInform = -1;
        Debug.DrawRay(transform.position + Vector3.down * 0.4f, this.transform.up * -1, Color.blue);
        RotCount++;
        Vector2 CrntDir = new Vector2(theIntegrator.X[3], theIntegrator.X[4]).normalized; //vel new Vector2(1, 0);//
        Vector2 CrntAngle = new Vector2(theIntegrator.X[6]/Mathf.PI*18, theIntegrator.X[7] / Mathf.PI*18); //Roll Pitch
        //* Mathf.Rad2Deg/180
        //Vector3 perp = Vector3.Cross(CrntDir, Vector3.up);

        targetDistance = new Vector2(goal.transform.position.x - gameObject.transform.position.x,
            goal.transform.position.z - gameObject.transform.position.z).magnitude;
        targetDirection = new Vector2(goal.transform.position.x - gameObject.transform.position.x,
            goal.transform.position.z - gameObject.transform.position.z).normalized;

        targetPolarAngle = Vector2.SignedAngle(CrntDir, targetDirection);//SignedAnglePlane(CrntDir, targetDirection, perp);
        //targetPolarAngle.y = SignedAnglePlane(CrntDir, targetDirection, Vector3.up);
        DSpeedMag = Mathf.Sqrt(theIntegrator.X[3] * theIntegrator.X[3] + theIntegrator.X[4] * theIntegrator.X[4] +
                theIntegrator.X[5] * theIntegrator.X[5]);

        Debug.DrawRay(transform.position, new Vector3(CrntDir.x, 0, CrntDir.y), Color.red);
        Debug.DrawRay(transform.position, new Vector3(targetDirection.x, 0, targetDirection.y), Color.magenta);

        //Raycast
        AddVectorObs(RayInformContain);
        //object direction
        AddVectorObs(targetPolarAngle / 180f);//1
        //object distance
        AddVectorObs(Util.Sigmoid(targetDistance) * 2f - 1f);//1
        //vel
        AddVectorObs(DSpeedMag/10);//1
        //Vector
        AddVectorObs(CrntDir);//2
        //Angle
        AddVectorObs(CrntAngle);//2 CrntAngle
        //whole - RayInformContain+11
    }

    private void FixedUpdate()
    {
        t = theIntegrator.RK4Step(theIntegrator.X, t, h);
        pos = gameObject.transform.position;
        velocity = (pos - previous) / Time.deltaTime; //get velocity at Scene ifself
        previous = pos;
        pos.x = theIntegrator.X[0]; //1.7f;//
        pos.z = theIntegrator.X[1]; //5.7f;//
        pos.y = theIntegrator.X[2];
        rot = gameObject.transform.rotation.eulerAngles;
        rot.x = -theIntegrator.X[6] * 180 / Mathf.PI;
        rot.z = -theIntegrator.X[7] * 180 / Mathf.PI;
        rot.y = -theIntegrator.X[8] * 180 / Mathf.PI;
        theIntegrator.z_d = Mathf.Clamp(theIntegrator.z_d, goal.transform.position.y - 10, goal.transform.position.y + 10);
        theIntegrator.phi_d = Mathf.Clamp(Control[0], -0.2f, 0.2f);
        theIntegrator.theta_d = Mathf.Clamp(Control[1], -0.2f, 0.2f);
        //Debug.Log(theIntegrator.phi_d* Mathf.Rad2Deg + "," + theIntegrator.theta_d* Mathf.Rad2Deg);
        gameObject.transform.position = pos;
        gameObject.transform.localEulerAngles = rot;
        //((goal.transform.position - gameObject.transform.position).magnitude < 1f)

        float Sum = 0; float count = 0;
        // Reward move direction & speed.           
        crntStep++;
        //cmlReward = 0;
        float x = Util.PowInt(1f - Mathf.Abs(targetPolarAngle / 180f), 16);
        //float y = Util.PowInt(1f - Mathf.Abs(targetPolarAngle.y / 180f), 16);
        cmlReward += (x * DSpeedMag/500);
        // Proportional penalty for proximity to obstacle (average of all rays with length <= 2)  
        List<float> list = RayInformContain.Where(d => d > 1 - ProxPanalty / LidarRange).ToList();
        cmlReward -= list.Count > 0 ? (list.Sum() / list.Count)*0.08f : 0;
        //cmlReward += (Util.Sigmoid(1f / (goal.transform.position - gameObject.transform.position).magnitude)-0.3f)*0.0001f;
        //0.3f / (goal.transform.position - gameObject.transform.position).magnitude - 0.3f;
        //(goal.transform.position - gameObject.transform.position).magnitude
        if (new Vector2(goal.transform.position.x - gameObject.transform.position.x,
            goal.transform.position.z - gameObject.transform.position.z).magnitude < 0.2f)
        {
            if (Maze.reachedtrig > 250)
            {
                goal.position = new Vector3((Random.Range(1f, 2.2f)), 1.5f, (Random.Range(1f, 6.2f)));
                
                    while (Vector3.Distance(obs[0].position, goal.transform.position) < 1
                     || Vector3.Distance(obs[1].position, goal.transform.position) < 1
                     || Vector3.Distance(obs[2].position, goal.transform.position) < 1
                     || Vector3.Distance(obs[3].position, goal.transform.position) < 1
                        || Vector3.Distance(Axis.position, goal.transform.position) < 1)
                        goal.position = new Vector3((Random.Range(1f, 2.2f)), 1.5f, (Random.Range(1f, 6.2f)));                
                Maze.reachedtrig = 0;
            }
            AddReward(5f);
            Maze.reachedtrig++;
            time = 0;
            RequestDecision();
            Done();

        }
        else if (isOutOfBounds)
        {
            AddReward(-5f);
            Done();
        }
        /*else if (collisiontrigger)
        {
            AddReward(-5f);
            Done();
        }*/
        else if (crntStep == decisionInterval)
        {
            AddReward(cmlReward / (float)decisionInterval);
            RequestDecision();
        }

    }

    public override void AgentAction(float[] vectorAction)
    {
        base.AgentAction(vectorAction);
        Control[0] = vectorAction[0] * 0.2f;// * Time.fixedDeltaTime*10 ;
        Control[1] = vectorAction[1] * 0.2f;// * Time.fixedDeltaTime*10 ;
        decisionInterval = Mathf.RoundToInt((vectorAction[2] + 1f) * 5f) + 1;
        crntStep = 0;
        cmlReward = 0;
    }

    /*private void OnCollisionEnter(Collision collision)
    {
        collisiontrigger = true;
    }*/



    public override void AgentReset()
    {
        base.AgentReset();
        t = 0.0f;
        h = 0.02f;
        theIntegrator.X = new float[12];
        /*if (reachedtrigger)
        {
            
            alterpos = new Vector3(theIntegrator.X[0], theIntegrator.X[2], theIntegrator.X[1]);

            reachedtrigger = false;
        }
        else if (alterpos != Vector3.zero)
        {
            theIntegrator.X[0] = alterpos.x;
            theIntegrator.X[2] = alterpos.y;
            theIntegrator.X[1] = alterpos.z;
        }
        else
        {
            theIntegrator.X[0] = Drone.x;
            theIntegrator.X[2] = Drone.y;
            theIntegrator.X[1] = Drone.z;
        }*/
        //if (reachedtrigger)
        /*foreach (GameObject obs in Obslist) obs.transform.position = new Vector3(Drone.x + 
            Random.Range(-2.5f, 2.5f), Drone.y, Drone.z + Random.Range(2f, 15f));*/
        theIntegrator.X[0] = Drone.x + Random.Range(0.5f, 2.7f);
        theIntegrator.X[1] = Drone.z + Random.Range(0.5f, 6.3f);
        theIntegrator.X[2] = Drone.y;
        Vector3 Dronepos = new Vector3(theIntegrator.X[0], theIntegrator.X[2], theIntegrator.X[1]);
        while (Vector3.Distance(obs[0].position, Dronepos) < 1
                     || Vector3.Distance(obs[1].position, Dronepos) < 1
                     || Vector3.Distance(obs[2].position, Dronepos) < 1
                     || Vector3.Distance(obs[3].position, Dronepos) < 1
                        || Vector3.Distance(Axis.position, Dronepos) < 1)
        {
            theIntegrator.X[0] = Drone.x + Random.Range(0.5f, 2.7f);
            theIntegrator.X[1] = Drone.z + Random.Range(0.5f, 6.3f);
            theIntegrator.X[2] = Drone.y;
            Dronepos = new Vector3(theIntegrator.X[0], theIntegrator.X[2], theIntegrator.X[1]);
        }                    
        /*theIntegrator.X[0] = Drone.x + Random.Range(-2.5f, 2.5f);
        //if (randomspwn)theIntegrator.X[1] = Drone.z;
        //else
        theIntegrator.X[1] = Drone.z + Random.Range(0f, 2f);
        theIntegrator.X[2] = Drone.y;*/

        /*theIntegrator.X[0] = Drone.x+ Random.Range(-2.5f, 2.5f);
        theIntegrator.X[1] = Drone.z + Random.Range(0f, 18f);
        theIntegrator.X[2] = Drone.y;*/
        theIntegrator.z_d = theIntegrator.X[2];
        theIntegrator.phi_d = 0;
        theIntegrator.theta_d = 0;
        theIntegrator.psi_d = 0;
        for (int i = 0; i < 3; i++) Control[i] = 0;
        gameObject.transform.position = new Vector3(theIntegrator.X[0], theIntegrator.X[2], theIntegrator.X[1]);
        gameObject.transform.rotation = Quaternion.Euler(Vector3.zero);
        collisiontrigger = false;
        randomspwn = !randomspwn;
        RequestDecision();
    }

    public override void AgentOnDone()
    {
    }

    public override float[] Heuristic()
    {
        var action = new float[3];

        //action[0] = Input.GetAxis("Throttle");
        action[0] = Input.GetAxis("Roll")*10;
        action[1] = Input.GetAxis("Pitch")*10;
        //action[1] = Input.GetAxis("Pitch");
        //action[3] = Input.GetAxis("Yaw");
        return action;
    }
    //private static float SignedAnglePlane(Vector3 v1, Vector3 v2, Vector3 plane)
    //{
    //    return Vector3.SignedAngle(
    //        Vector3.ProjectOnPlane(v1, plane).normalized,
    //        Vector3.ProjectOnPlane(v2, plane).normalized,
    //        plane
    //    );
    //}

}
