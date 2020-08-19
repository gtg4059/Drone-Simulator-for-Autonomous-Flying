// Integrator.cs
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Integrator is an abstract class for integrating a system of ODEs
/// </summary>
abstract public class Integrator
{

    int nEquations;
    float[] store;
    float[] k1;
    float[] k2;
    float[] k3;
    float[] k4;
    float[] ym1;
    float[] ym2;
    float[] ym3;
    float[] P;
    float[] dm1;
    float[] dm2;
    float[] dm3;
    float[] dp1;
    int abmSteps = 0;
    float abmRms2;

    public Integrator()
    {
        Init(1);
    }

    public float[] getK3()
    {
        return k3;
    }

    /// <summary>
    /// Allocate memory for all storage arrays and set number of equations
    /// </summary>
    /// <param name="nEquations">N equations.</param>
    public void Init(int nEquations)
    {
        // set up temp arrays
        this.nEquations = nEquations;
        store = new float[nEquations];
        k1 = new float[nEquations];
        k2 = new float[nEquations];
        k3 = new float[nEquations];
        k4 = new float[nEquations];
        ym1 = new float[nEquations];
        ym2 = new float[nEquations];
        ym3 = new float[nEquations];
        P = new float[nEquations];
        dm1 = new float[nEquations];
        dm2 = new float[nEquations];
        dm3 = new float[nEquations];
        dp1 = new float[nEquations];
        abmSteps = 0;
    }

    /// <summary>
    /// Abstract void, override this method to set the ODEs to be
    /// integrated.
    /// </summary>
    /// <param name="x">The values being integrated.</param>
    /// <param name="xdot">The derivatives being calculated.</param>
    abstract public void RatesOfChange(float[] x, float[] xdot, float t);

    /// <summary>
    /// Step forward using Euler's method
    /// </summary>
    /// <param name="x">The values being integrated.</param>
    /// <param name="h">The time step.</param>
    public void EulerStep(float[] x, float t, float h)
    {
        RatesOfChange(x, k1, t);
        for (int i = 0; i < nEquations; i++)
        {
            x[i] += k1[i] * h;
        }
    }

    /// <summary>
    /// Step forward using 4th order Runge Kutta method
    /// </summary>
    /// <param name="x">The values being integrated.</param>
    /// <param name="h">The time step.</param>
    public float RK4Step(float[] x, float t, float h)
    {
        RatesOfChange(x, k1, t);
        for (int i = 0; i < nEquations; i++)
        {
            store[i] = x[i] + k1[i] * h / 2.0f;
        }
        RatesOfChange(store, k2, t);
        for (int i = 0; i < nEquations; i++)
        {
            store[i] = x[i] + k2[i] * h / 2.0f;
        }
        RatesOfChange(store, k3, t);
        for (int i = 0; i < nEquations; i++)
        {
            store[i] = x[i] + k3[i] * h;
        }
        RatesOfChange(store, k4, t);
        for (int i = 0; i < nEquations; i++)
        {
            x[i] = x[i] + (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]) * h / 6.0f;
        }
        return t + h;
    }


    /**
	 * Calculates a single step using Adams Bashforth Moulton,
	 * 
	 * @param x Array of values being integrated.
	 * @param t Time at which step begins
	 * @param h Duration of step
	 * @return Error prediction at end of step
	 */
    public float abmStep(float[] x, float t, float h)
    {
        abmRms2 = 0.0f;
        if (abmSteps == 0)
        {
            for (int i = 0; i < x.Length; i++)
            {
                ym3[i] = x[i];
                ym2[i] = x[i];
            }
            RatesOfChange(dm3, ym3, t);
            t = RK4Step(ym2, t, h);
            RatesOfChange(dm2, ym2, t);
            for (int i = 0; i < x.Length; i++)
            {
                x[i] = ym2[i];
            }
            abmSteps += 1;
            return 1.0f;
        }
        else if (abmSteps == 1)
        {
            for (int i = 0; i < x.Length; i++)
            {
                ym1[i] = ym2[i];
            }
            t = RK4Step(ym1, t, h);
            RatesOfChange(dm1, ym1, t);
            for (int i = 0; i < x.Length; i++)
            {
                x[i] = ym1[i];
            }
            abmSteps += 1;
            return 1.0f;
        }
        else
        {
            RatesOfChange(k1, x, t);
            for (int i = 0; i < x.Length; i++)
            {
                P[i] = x[i] + (h / 24.0f) *
                    (55.0f * k1[i] - 59.0f * dm1[i] + 37.0f * dm2[i] - 9.0f * dm3[i]);
            }
            RatesOfChange(dp1, P, t + h);
            abmRms2 = 0.0f;
            for (int i = 0; i < x.Length; i++)
            {
                store[i] = x[i];
                x[i] += (h / 24.0f) * (9 * dp1[i] + 19.0f * k1[i] - 5.0f * dm1[i] + dm2[i]);
                dm3[i] = dm2[i];
                dm2[i] = dm1[i];
                dm1[i] = k1[i];
                ym3[i] = ym2[i];
                ym2[i] = ym1[i];
                ym1[i] = store[i];
                abmRms2 += (x[i] - P[i]) * (x[i] - P[i]) / (x[i] + P[i]) / (x[i] + P[i]);
            }
            abmRms2 /= x.Length;
            if (abmSteps < 5) abmSteps += 1;
            return t + h;
        }
    }

    public float abmError()
    {
        return abmRms2;
    }

}