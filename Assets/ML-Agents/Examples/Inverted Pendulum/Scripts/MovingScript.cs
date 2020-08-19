using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovingScript : MonoBehaviour
{
    //Input
    float phi_d, theta_d, psi_d, phi_d_dot, theta_d_dot, psi_d_dot, z_d,
        z_d_dot, K_phiD, K_thetaD, K_psiD, K_phiP, K_thetaP, K_psiP, K_zD, K_zP;
    float[] X;
    public static Matrix4x4 identity;
    float g = 9.81f; // gravitational acceleration
    float k = 2.98f * Mathf.Pow(10, -6); // thrust coefficient over angular velocity = Thrust force = k * angular velocity ^ 2
    float l = 0.225f; // drone length
    float Ixx = 4.856f * Mathf.Pow(10, -3); // x axis rotational inertia
    float Iyy = 4.856f * Mathf.Pow(10, -3);   // y axis rotational inertia
    float Izz = 8.801f * Mathf.Pow(10, -3);  // z axis rotational inertia
    float m = 0.468f;  // drone mass
    float b = 1.14f * Mathf.Pow(10, -7); // damping coefficient
    float Ax = 0.25f;  // x axis drag force coefficient
    float Ay = 0.25f;  // y axis drag force coefficient
    float Az = 0.25f;  // z axis drag force coefficient
    //ArrayList Xdot = new ArrayList();
    List<float[]> Xdot = new List<float[]>();
    // Use this for initialization
    void Start()
    {
        //whole input
        float[] x0 = new float[12] { 0.1f, 0, 1, 0, 0, 0, 10 * Mathf.PI / 180, 10 * Mathf.PI / 180, 10 * Mathf.PI / 180, 0, 0, 0 };
        phi_d = 30 * Mathf.PI / 180; //roll target angle(deg)
        theta_d = 30 * Mathf.PI / 180; //pitch target angle
        psi_d = 30 * Mathf.PI / 180; //yaw target angle
        phi_d_dot = 0 * Mathf.PI / 180;  //roll target angluar velocity(deg/sec)
        theta_d_dot = 0 * Mathf.PI / 180;//pitch target angluar velocity
        psi_d_dot = 0 * Mathf.PI / 180;//yaw target angluar velocity
        z_d = 1000; //Attitude target(m)
        z_d_dot = 0; // Target Attitude velocity
        K_phiD = 1.75f; // roll D gain
        K_thetaD = 1.75f; // pitch D gain
        K_psiD = 1.75f; // Yaw D gain
        K_phiP = 6; // roll P gain
        K_thetaP = 6;// pitch P gain
        K_psiP = 6;// Yaw P gain
        K_zD = 2.5f; // Attitude D gain
        K_zP = 1.5f; // Attitude P gain
        X = x0;
    }

    // Update is called once per frame
    void Update()
    {
        //[t,X]=ode23(@(t,X) DronePIDControl_one(t,X),tspan,x0);
        float x = X[0];  // x displacement
        float y = X[1];  // y displacement
        float z = X[2]; // z displacement
        float x_dot = X[3]; // x velocity
        float y_dot = X[4]; // y velocity
        float z_dot = X[5]; // z velocity
        float phi = X[6];  // roll angle
        float theta = X[7]; // pitch angle
        float psi = X[8]; // yaw angle
        float phi_dot = X[9];  //roll angular velocity
        float theta_dot = X[10]; // pitch angular velocity
        float psi_dot = X[11]; // Yaw angular velocity
        
        float T = (g + K_zD * (z_d_dot - z_dot) + K_zP * (z_d - z)) * m / (Mathf.Cos(phi) * Mathf.Cos(theta)); // PID Attitude control input
        float tau_phi = (K_phiD * (phi_d_dot - phi_dot) + K_phiP * (phi_d - phi)) * Ixx; // Roll PID control input
        float tau_theta = (K_thetaD * (theta_d_dot - theta_dot) + K_thetaP * (theta_d - theta)) * Iyy;// Pitch PID control input
        float tau_psi = (K_psiD * (psi_d_dot - psi_dot) + K_psiP * (psi_d - psi)) * Izz; // Yaw PID control input

        float w1 = Mathf.Sqrt(T / (4 * k) - tau_theta / (2 * k * l) - tau_psi / (4 * b)); // 1st motor angular velocity(rad/ sec)
        float w2 = Mathf.Sqrt(T / (4 * k) - tau_phi / (2 * k * l) + tau_psi / (4 * b)); // 2nd motor angular velocity(rad/ sec)
        float w3 = Mathf.Sqrt(T / (4 * k) + tau_theta / (2 * k * l) - tau_psi / (4 * b));// 3rd motor angular velocity(rad/ sec)
        float w4 = Mathf.Sqrt(T / (4 * k) + tau_phi / (2 * k * l) + tau_psi / (4 * b));// 4th motor angular velocity(rad/ sec)

        float x_ddot = T / m * (Mathf.Cos(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) + Mathf.Sin(psi) * Mathf.Sin(phi)) - 1 / m * Ax * x_dot; // dx ^ 2 / dt ^ 2; x acceleration
        float y_ddot = T / m * (Mathf.Sin(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) - Mathf.Cos(psi) * Mathf.Sin(phi)) - 1 / m * Ay * y_dot; // dy ^ 2 / dt ^ 2; y acceleration
        float z_ddot = -g + T / m * (Mathf.Cos(theta) * Mathf.Cos(phi)) - 1 / m * Az * z_dot; // dz ^ 2 / dt ^ 2; z acceleration

        Matrix4x4 J = Matrix4x4.identity;
        J[0, 0] = Ixx;
        J[0, 1] = 0;
        J[0, 2] = -Ixx * Mathf.Sin(theta);
        J[1, 0] = 0;
        J[1, 1] = Iyy * Mathf.Pow(Mathf.Cos(phi), 2) + Izz * Mathf.Pow(Mathf.Sin(phi), 2);
        J[1, 2] = (Iyy - Izz) * Mathf.Cos(phi) * Mathf.Sin(phi) * Mathf.Cos(theta);
        J[2, 0] = -Ixx * Mathf.Sin(theta);
        J[2, 1] = (Iyy - Izz) * Mathf.Cos(phi) * Mathf.Sin(phi) * Mathf.Cos(theta);
        J[2, 2] = Ixx * Mathf.Pow(Mathf.Sin(theta), 2) + Iyy * Mathf.Pow(Mathf.Sin(phi), 2) * Mathf.Pow(Mathf.Cos(theta), 2)
    + Izz * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Pow(Mathf.Cos(theta), 2);
        Vector4 tau_B = new Vector4( tau_phi, tau_theta, tau_psi,1 );  // roll, pitch, yaw torque

        Vector4 eta_dot = new Vector4(phi_dot, theta_dot, psi_dot,1); // roll angular velocity, pitch angular velocity, Yaw angular velocity
                                                                        //float[,] c = new float[3, 3];

        //Matrix4x4 c = new Matrix4x4;
        Matrix4x4 c = Matrix4x4.identity;
        c[0, 0] = 0;
        c[0, 1] = (Iyy - Izz) * (theta_dot * Mathf.Sin(phi) * Mathf.Sin(phi) + psi_dot * Mathf.Pow(Mathf.Sin(phi), 2) * Mathf.Cos(theta)) + (Izz - Iyy) * psi_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Cos(theta) - Ixx * psi_dot * Mathf.Cos(theta);
        c[0, 2] = (Izz - Iyy) * psi_dot * Mathf.Sin(phi) * Mathf.Sin(phi) * Mathf.Pow(Mathf.Cos(theta), 2);
        c[1, 0] = (Iyy - Izz) * (theta_dot * Mathf.Cos(phi) * Mathf.Sin(phi) + psi_dot * Mathf.Pow(Mathf.Sin(phi), 2) * Mathf.Cos(theta)) + (Iyy - Izz) * psi_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Cos(theta) + Ixx * psi_dot * Mathf.Cos(theta);
        c[1, 1] = (Izz - Iyy) * phi_dot * Mathf.Sin(phi) * Mathf.Sin(phi);
        c[1, 2] = -Ixx * psi_dot * Mathf.Sin(theta) * Mathf.Cos(theta) + Iyy * psi_dot * Mathf.Pow(Mathf.Sin(phi), 2) * Mathf.Sin(theta) * Mathf.Cos(theta) + Izz * psi_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Sin(theta) * Mathf.Cos(theta);
        c[2, 0] = (Iyy - Izz) * psi_dot * Mathf.Pow(Mathf.Cos(theta), 2) * Mathf.Sin(phi) * Mathf.Cos(phi) - Ixx * theta_dot * Mathf.Cos(theta);
        c[2, 1] = (Izz - Iyy) * (theta_dot * Mathf.Cos(phi) * Mathf.Sin(phi) * Mathf.Sin(theta) + phi_dot * Mathf.Pow(Mathf.Sin(phi), 2) * Mathf.Cos(theta)) + (Iyy - Izz) * phi_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Cos(theta) + Ixx * psi_dot * Mathf.Sin(theta) * Mathf.Cos(theta) - Iyy * psi_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Cos(theta) * Mathf.Cos(theta) - Izz * psi_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Cos(theta) * Mathf.Cos(theta);
        c[2, 2] = (Iyy - Izz) * phi_dot * Mathf.Cos(phi) * Mathf.Sin(phi) * Mathf.Pow(Mathf.Cos(theta), 2) - Iyy * theta_dot * Mathf.Pow(Mathf.Sin(phi), 2) * Mathf.Cos(theta) * Mathf.Sin(theta) - Izz * theta_dot * Mathf.Pow(Mathf.Cos(phi), 2) * Mathf.Cos(theta) * Mathf.Sin(theta) + Ixx * theta_dot * Mathf.Cos(theta) * Mathf.Cos(theta);


        Vector4 eta_ddot = J.inverse * (tau_B - c *eta_dot); // roll, pitch, yaw angular acceleration
        float phi_ddot = eta_ddot[0];  // roll angular acceleration
        float theta_ddot = eta_ddot[1]; // pitch angular acceleration
        float psi_ddot = eta_ddot[2];// Yaw angular acceleration

        float[] xd = new float[12];
        xd[0] = X[3];
        xd[1] = X[4];
        xd[2] = X[5];
        xd[3] = x_ddot;
        xd[4] = y_ddot;
        xd[5] = z_ddot;
        xd[6] = X[9];
        xd[7] = X[10];
        xd[8] = X[11];
        xd[9] = phi_ddot;
        xd[10] = theta_ddot;
        xd[11] = psi_ddot;
    }

}

