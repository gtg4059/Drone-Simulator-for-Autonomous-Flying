using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR;

namespace PA_DronePack
{
    public class PAVR_DroneRemote : MonoBehaviour
    {
        Vector3 rightHandPos;
        Quaternion rightHandRot;

        void Update()
        {
            transform.localPosition = UnityEngine.XR.InputTracking.GetLocalPosition(UnityEngine.XR.XRNode.RightHand);
            transform.localRotation = UnityEngine.XR.InputTracking.GetLocalRotation(UnityEngine.XR.XRNode.RightHand);
        }
    }
}