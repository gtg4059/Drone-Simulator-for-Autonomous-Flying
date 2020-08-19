using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lidar2D : MonoBehaviour
{
    public float DRayInform;
    public float[] RayInformContain;
    private int RotCount=1;
    void Start()
    {
        RayInformContain = new float[360];
    }
    void FixedUpdate()
    {
        RaycastHit hit;      
        for (int i = (int)(7.2f * 5.5f * (RotCount-1)); i < 7.2f*5.5f*RotCount; i++)
        {
            Quaternion rotation = Quaternion.AngleAxis(i%360, transform.up);
            Ray Ray = new Ray(transform.position+Vector3.up*0.3f, rotation * this.transform.right * 2);
            Debug.DrawRay(transform.position + Vector3.up * 0.3f, rotation * this.transform.right * 2, Color.red);
            if (Physics.Raycast(Ray, out hit) && hit.distance < 12)
            {
                RayInformContain[i % 360] = hit.distance;
            }
        }
        Quaternion rot = Quaternion.AngleAxis(0, transform.up);
        Ray DRay = new Ray(transform.position + Vector3.down * 0.3f, rot * this.transform.up * -2);
        Debug.DrawRay(transform.position + Vector3.down * 0.3f, rot * this.transform.up * -2, Color.blue);
        RotCount++;
    }
}