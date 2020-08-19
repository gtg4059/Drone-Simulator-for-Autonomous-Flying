using UnityEngine;
using MLAgents;

public class PendulumAgent : Agent
{
    public SHOModel Hinge;
    public Rigidbody Wheel;
    public float action;
    bool condition;
    float MotorInput;
    private float time = 0, velocity;
    public override void InitializeAgent()
    {
        base.InitializeAgent();
    }

    public override void CollectObservations()
    {
        base.CollectObservations();
        AddVectorObs(Hinge.Angle);
        AddVectorObs(Hinge.AngularVel);
    }

    public override void AgentAction(float[] vectorAction)
    {
        base.AgentAction(vectorAction);
        action = vectorAction[0];
        //action = Mathf.Clamp(action, -1f, 1f);
        /*if (!System.Single.IsNaN(action))*/ MotorInput = action * 0.21f;
        Hinge.MotorInput = MotorInput;
        time++;
        float r = -1f * ((Hinge.Angle * Hinge.Angle + 0.0001f * Hinge.AngularVel * Hinge.AngularVel));
        //if (Hinge.Angle < Mathf.PI / 60 && Mathf.Abs(Hinge.AngularVel) < 0.5f) r = 0.1f;
        SetReward(r);
        /*if (Hinge.Angle < Mathf.PI / 60 && Mathf.Abs(Hinge.AngularVel) < 0.4f)
        {
            Done();
            r = 50f;
            SetReward(r);
        }*/
        if (time>=1000)
        {
            time = 0;
            Done();
        }

    }

    public override void AgentReset()
    {
        base.AgentReset();
        
            MotorInput = 0;
            Hinge.Reset();
    }

    public override void AgentOnDone()
    {
    }

    public override float[] Heuristic()
    {
        var action = new float[1];

        action[0] = Input.GetAxis("Horizontal");
        return action;
    }
}
