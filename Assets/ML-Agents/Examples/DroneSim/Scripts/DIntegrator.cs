using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DIntegrator : Integrator
{
    float phi_d_dot, theta_d_dot, psi_d_dot,
        z_d_dot, K_phiD, K_thetaD, K_psiD, K_phiP, K_thetaP, K_psiP, K_zD, K_zP;

    public float z_d, phi_d, theta_d, psi_d;

    public float[] X;
    public static Matrix4x4 identity;
    public float g = 9.81f; // gravitational acceleration
    public float k, l, Ixx, Iyy, Izz, m, b,Ax,Ay,Az;
    //ArrayList Xdot = new ArrayList();
    List<float[]> Xdot = new List<float[]>();
    public float w1, w2, w3, w4;
    float a1, a2, a3, a4;
    public float[] f = { 0.1f, 0.3f, 0.1f, 0f, 0f, 0f };

    public void SetIC(float[] x)
    {
        phi_d = 0 * Mathf.PI / 180; //roll target angle(deg)
        theta_d = 0 * Mathf.PI / 180; //pitch target angle
        psi_d = 0;// -1*UIButton.rotZ * Mathf.PI / 180; //yaw target angle
        phi_d_dot = 0 * Mathf.PI / 180;  //roll target angluar velocity(deg/sec)
        theta_d_dot = 0 * Mathf.PI / 180;//pitch target angluar velocity
        psi_d_dot = 0 * Mathf.PI / 180;//yaw target angluar velocity
        z_d = 0;// UIButton.posZ; //Attitude target(m)
        z_d_dot = 0; // Target Attitude velocity
        K_phiD = 1.75f; // roll D gain
        K_thetaD = 1.75f; // pitch D gain
        K_psiD = 1.75f; // Yaw D gain
        K_phiP = 6; // roll P gain
        K_thetaP = 6;// pitch P gain
        K_psiP = 6;// Yaw P gain
        K_zD = 2.5f; // Attitude D gain
        K_zP = 1.5f; // Attitude P gain
        X = x;
        Init(12);
    }

    public override void RatesOfChange(float[] X, float[] F, float t)
    {
        float x = X[0];  // x displacement
        float y = X[1];  // y displacement
        float z = X[2]; // z displacement
        float x_dot = X[3]; // x velocity
        float y_dot = X[4]; // y velocity
        float z_dot = X[5]; // z velocity        
        float phi = X[6];   // roll angle     
        float theta = X[7];  // pitch angle
        float psi = X[8];  // yaw angle
        float phi_dot = X[9];  //roll angular velocity
        float theta_dot = X[10]; // pitch angular velocity
        float psi_dot = X[11]; // Yaw angular velocity
        for (int i = 0; i < 12; i++)
        {
            if (System.Single.IsNaN(X[i]))
                Debug.Log("a");
        }
        float T = (g + K_zD * (z_d_dot - z_dot) + K_zP * (z_d - z)) * m / (Mathf.Cos(phi) * Mathf.Cos(theta)); // PID Attitude control input
        float tau_phi = (K_phiD * (phi_d_dot - phi_dot) + K_phiP * (phi_d - phi)) * Ixx; // Roll PID control input
        float tau_theta = (K_thetaD * (theta_d_dot - theta_dot) + K_thetaP * (theta_d - theta)) * Iyy;// Pitch PID control input
        float tau_psi = (K_psiD * (psi_d_dot - psi_dot) + K_psiP * (psi_d - psi)) * Izz; // Yaw PID control input

         a1 = Mathf.Sqrt(T / (4 * k) - tau_theta / (2 * k * l) - tau_psi / (4 * b)); // 1st motor angular velocity(rad/ sec)
         a2 = Mathf.Sqrt(T / (4 * k) - tau_phi / (2 * k * l) + tau_psi / (4 * b)); // 2nd motor angular velocity(rad/ sec)
         a3 = Mathf.Sqrt(T / (4 * k) + tau_theta / (2 * k * l) - tau_psi / (4 * b));// 3rd motor angular velocity(rad/ sec)
         a4 = Mathf.Sqrt(T / (4 * k) + tau_phi / (2 * k * l) + tau_psi / (4 * b));// 4th motor angular velocity(rad/ sec)
        if (!System.Single.IsNaN(a1)&& !System.Single.IsNaN(a2)&& !System.Single.IsNaN(a3)&& !System.Single.IsNaN(a4))
        {
            w1 = a1; w2 = a2; w3 = a3; w4 = a4;
        }
        float x_ddot = T / m * (Mathf.Cos(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) + Mathf.Sin(psi) * Mathf.Sin(phi)) - 1 / m * Ax * x_dot;// +f[0]/m; // dx ^ 2 / dt ^ 2; x acceleration
        float y_ddot = T / m * (Mathf.Sin(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) - Mathf.Cos(psi) * Mathf.Sin(phi)) - 1 / m * Ay * y_dot;// + f[1] / m; // dy ^ 2 / dt ^ 2; y acceleration
        float z_ddot = -g + T / m * (Mathf.Cos(theta) * Mathf.Cos(phi)) - 1 / m * Az * z_dot;// + f[2] / m; // dz ^ 2 / dt ^ 2; z acceleration

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
        Vector4 tau_B = new Vector4(tau_phi, tau_theta, tau_psi, 1);  // roll, pitch, yaw torque

        Vector4 eta_dot = new Vector4(phi_dot, theta_dot, psi_dot, 1); // roll angular velocity, pitch angular velocity, Yaw angular velocity

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


        Vector4 eta_ddot = J.inverse * (tau_B - c * eta_dot); // roll, pitch, yaw angular acceleration
        float phi_ddot = eta_ddot[0]+f[3]/Ixx;  // roll angular acceleration
        float theta_ddot = eta_ddot[1]+f[4] / Iyy; // pitch angular acceleration
        float psi_ddot = eta_ddot[2]+f[5] / Izz;// Yaw angular acceleration

        for(int i = 0; i<12; i++)
        {
            if (System.Single.IsNaN(X[i]))
                Debug.Log("a");
        }
         F[0] = X[3];
        F[1] = X[4];
        F[2] = X[5];
        F[3] = x_ddot;
        F[4] = y_ddot;
        F[5] = z_ddot;
        F[6] = X[9];
        F[7] = X[10];
        F[8] = X[11];
        F[9] = phi_ddot;
        F[10] = theta_ddot;
        F[11] = psi_ddot; 
    }
}
