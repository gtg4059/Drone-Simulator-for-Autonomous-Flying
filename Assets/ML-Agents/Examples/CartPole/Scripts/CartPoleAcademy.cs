using UnityEngine;
using MLAgents;

public class CartPoleAcademy : Academy
{
    public override void InitializeAcademy()
    {
        FloatProperties.RegisterCallback("gravity", f => { Physics.gravity = new Vector3(0, -f, 0); });
    }
}
