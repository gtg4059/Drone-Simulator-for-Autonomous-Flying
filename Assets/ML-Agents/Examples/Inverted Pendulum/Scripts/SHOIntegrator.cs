using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SHOIntegrator : Integrator
{
    float phi_d_dot, theta_d_dot, psi_d_dot,
        z_d_dot, K_phiD, K_thetaD, K_psiD, K_phiP, K_thetaP, K_psiP, K_zD, K_zP;

    public float z_d, phi_d, theta_d, psi_d;

    public float M1 = 0.02f, M2 = 0.3f, L1 = 0.125f, Lc1 = 0.063f, g = 9.81f,
        I1 = 47 * Mathf.Pow(10, -6), I2 = 32 * Mathf.Pow(10, -6), torque;
    float d11, d12, d21, d22, detD;
    public float[] X;

    public void SetIC(float[] x)
    {
        /*phi_d = 0 * Mathf.PI / 180; //roll target angle(deg)
        theta_d = 0 * Mathf.PI / 180; //pitch target angle
        psi_d = -1 * 0 * Mathf.PI / 180; //yaw target angle
        phi_d_dot = 0 * Mathf.PI / 180;  //roll target angluar velocity(deg/sec)
        theta_d_dot = 0 * Mathf.PI / 180;//pitch target angluar velocity
        psi_d_dot = 0 * Mathf.PI / 180;//yaw target angluar velocity
        z_d = 0; //Attitude target(m)
        z_d_dot = 0; // Target Attitude velocity
        K_phiD = 1.75f; // roll D gain
        K_thetaD = 1.75f; // pitch D gain
        K_psiD = 1.75f; // Yaw D gain
        K_phiP = 6; // roll P gain
        K_thetaP = 6;// pitch P gain
        K_psiP = 6;// Yaw P gain
        K_zD = 2.5f; // Attitude D gain
        K_zP = 1.5f; // Attitude P gain*/
        d11 = M1 * Mathf.Pow(Lc1, 2) + M2 * Mathf.Pow(L1, 2) + I1 + I2;
        d12 = I2; d21 = I2; d22 = I2; detD = d11 * d22 - d12 * d21;
        X = x;
        Init(3);
    }

    public override void RatesOfChange(float[] X, float[] F, float t)
    {
        float theta1 = X[0]; // theta1 angle
        float theta1_dot = X[1]; // theta1 angular velocity(rad/sec)
        float theta2_dot = X[2];  //theta2 angular velocity
        if (X[2] <= -481.7f) torque = Mathf.Clamp(torque, 0, 0.021f);
        else if (X[2] >= 481.7f) torque = Mathf.Clamp(torque, -0.021f, 0);
        float psiX0 = -1 * (M1 * Lc1 + M2 * L1) * g * Mathf.Sin(X[0]);
        F[0] = X[1]; // differential data
        F[1] = -1 * (d22 * psiX0 + d12 * torque) / detD; // differential data
        F[2] = (d21 * psiX0 + d11 * torque) / detD; // differential data
    }
}
