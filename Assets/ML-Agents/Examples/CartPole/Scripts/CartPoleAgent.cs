using UnityEngine;
using MLAgents;

public class CartPoleAgent : Agent
{
    public Rigidbody cartRigidbody;
    
    public Rigidbody poleRigidbody;

    
    public float action;
    
    private float speed = 10f;
    
    /*private void FixedUpdate()
    {
        action = Input.GetAxis("Horizontal");
        action = Mathf.Clamp(action, -1f, 1f);

        var rigidbody = cartRigidbody;
        var position = rigidbody.position + rigidbody.transform.TransformDirection
            (Vector3.right * action * Time.fixedDeltaTime * speed);
        rigidbody.MovePosition(position);

    }*/


    public override void InitializeAgent()
    {
        base.InitializeAgent();
    }
    
    public override void CollectObservations()
    {
        base.CollectObservations();


        AddVectorObs(cartRigidbody.transform.localPosition.x);
        
        AddVectorObs(poleRigidbody.transform.localRotation.z);
        
        AddVectorObs(poleRigidbody.angularVelocity.z);

    }
    
    public override void AgentAction(float[] vectorAction)
    {
        base.AgentAction(vectorAction);
        
        action = vectorAction[0];
        action = Mathf.Clamp(action, -1f, 1f);
        
        var position = cartRigidbody.position + 
            cartRigidbody.transform.TransformDirection(Vector3.right * action * Time.fixedDeltaTime * speed);
        cartRigidbody.MovePosition(position);
        
        SetReward(0.01f);
        
        bool condition = false;
        condition |= (60f < Vector3.Angle(poleRigidbody.transform.up, Vector3.up));
        condition |= Mathf.Abs(cartRigidbody.transform.localPosition.x) > 4f;
        if (condition)
        {
            Done();
            SetReward(-1f);
        }
    }
    public override void AgentReset()
    {
        base.AgentReset();
        
        {
            var transform = cartRigidbody.transform;
            var rigidbody = cartRigidbody;
            transform.localPosition = Vector3.zero;
            rigidbody.velocity = Vector3.zero;
            rigidbody.angularVelocity = Vector3.zero;
        }
        
        {
            var transform = poleRigidbody.transform;
            var rigidbody = poleRigidbody;
            transform.localPosition = new Vector3(0f, 2f, 0f);
            transform.localRotation = Quaternion.Euler(0f, 0f, Random.Range(-20f, 20f));
            rigidbody.velocity = Vector3.zero;
            rigidbody.angularVelocity = Vector3.zero;
        }
    }

    public override float[] Heuristic()
    {
        var action = new float[1];

        action[0] = -Input.GetAxis("Horizontal");
        return action;
    }
}
