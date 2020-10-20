using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//// dv / dt = -(v*∇)v-(1/D)∇p+s∇²v+F
//// ∇*v = 0   // v[x+1,y] - v[x-1,y] + s[x+1,y] - s[x-1,y] =0
//// ∇s = [(s[x+1,y].x-s[x-1,y].x), (s[x,y+1].y-s[x,y-1].y)]
//// ∇²s = ∇.dot(w) //Poisson-pressure exuation.
//// Step(v,p) = Projection(    Force(  Diffusion(    Advection(    v,p))))
//v = velocity
//p = Preasure
//s = viscosity
//D = dencityConstant


//Advection Term:   -(v*∇)v     //"self-advection of the velocity field"
//Pressure Term:    -(1/P)∇p    //acceleration from pressure
//Diffusion Term:   +s(∇^2)v    //"Viscosity is a measure of how resistive a fluid is to flow. This resistance results in diffusion of the momentum"


// u + ∇p = w
public class NavierFluid
{
    int size;
    Vector2[][,] velocity;
    float[][,] pressure;
    float[,] density;
    float[,] divergence;
    float viscosity;
    float deltaTime;
    int READ = 0;
    int WRITE = 1;
    const int velocityW = 2;

    float time;

    public float[,] Density => density;
    public Vector2[,] Velocities => velocity[READ];
    public NavierFluid(int size, float visc)
    {
        this.size = size;
        this.viscosity = visc;
        velocity = new Vector2[3][,]
        {
            new Vector2[size,size],
            new Vector2[size,size],
            new Vector2[size,size]
        };
        pressure = new float[2][,] {
            new float[size, size],
            new float[size, size]
        };
        density = new float[size, size];


        divergence = new float[size, size];
        for (int x = 0; x < size; x++)
        {
            for (int y = 0; y < size; y++)
            {
                density[x, y] = ((x + y) / 4) % 2;
            }
        }
    }
    void SwapReadWrite(object[] array)
    {
        object temp = array[0];
        array[0] = array[1];
        array[1] = temp;
    }
    public void AddDencity(int x, int y, float amount)
    {
        if (x > 0 && x < size - 1 && y > 0 && y < size - 1)
        {
            density[x, y] += amount;
        }
    }
    public void AddVelocity(int x, int y, Vector2 amount)
    {
        if (x > 0 && x < size - 1 && y > 0 && y < size - 1)
            velocity[READ][x, y] += amount;
    }




    public void StepSimulation(float timeStep)
    {
        deltaTime = timeStep;
        time += timeStep;
        float dx = 1.0f / size;
        float diffAlpha = (dx * dx) / (viscosity * deltaTime);
        float jacobiInverseBeta = 1 / (4 + diffAlpha);




        //////ADVECT
        System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
        {
            //for (int x = 1; x < size - 1; x++)
            //{
            for (int y = 1; y < size - 1; y++)
            {
                Vector2Int pos = new Vector2Int(x, y);
                velocity[velocityW][x, y] = AdvectVec(pos, velocity[READ], velocity[READ]);
                velocity[WRITE][x, y] = velocity[velocityW][x, y];
                density[x, y] = AdvectF(pos, velocity[READ], density);
            }
        });
        SwapReadWrite(velocity);




        //////DIFFUSION

        for (int i = 0; i < 40; i++)
        {
            System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
            {
                //for (int x = 1; x < size - 1; x++)
                //{
                for (int y = 1; y < size - 1; y++)
                {
                    velocity[WRITE][x, y] = Jacobi(new Vector2Int(x, y), diffAlpha, jacobiInverseBeta, velocity[READ], velocity[velocityW]);
                }
            });
            SwapReadWrite(velocity);
        }

        //////////EXTERNAL FORCES
        Vector2 forceDirection = new Vector2(Mathf.Sin(time), Mathf.Cos(time));
        float mid = size / 2;
        Vector2 midPoint = new Vector2(mid, mid);
        //for (int x = 1; x < size - 1; x++)
        //{
        System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
        {
            for (int y = 1; y < size - 1; y++)
            {
                float force = Mathf.Exp(-200 * Vector2.Distance(midPoint, new Vector2(x, y)));
                velocity[WRITE][x, y] += forceDirection * deltaTime * force * 30000;
            }
        });
        SwapReadWrite(velocity);



        //////PROJECTION

        divergence = new float[size, size];
        for (int x = 1; x < size - 1; x++)
        {
            for (int y = 1; y < size - 1; y++)
            {
                divergence[x, y] = ComputeDivergence(new Vector2Int(x, y), velocity[velocityW], size * 0.5f);
                pressure[WRITE][x, y] = 0;
            }
        }
        SwapReadWrite(velocity);
        SwapReadWrite(pressure);
        for (int i = 0; i < 50; i++)
        {
            for (int x = 1; x < size - 1; x++)
            {
                for (int y = 1; y < size - 1; y++)
                {
                    pressure[WRITE][x, y] = Jacobi(new Vector2Int(x, y), -(dx * dx), 0.25f, pressure[READ], divergence);
                }
            }
            SwapReadWrite(pressure);
        }
        for (int x = 1; x < size - 1; x++)
        {
            for (int y = 1; y < size - 1; y++)
            {
                velocity[WRITE][x, y] -= ComputeGradient(new Vector2Int(x, y), pressure[READ]);
                if (x == 1) velocity[WRITE][0, y] = -velocity[WRITE][x, y];
                if (y == 1) velocity[WRITE][x, 0] = -velocity[WRITE][x, y];
                if (x == size - 2) velocity[WRITE][size - 1, y] = -velocity[WRITE][x, y];
                if (y == size - 2) velocity[WRITE][x, size - 1] = -velocity[WRITE][x, y];
            }
        }

        SwapReadWrite(velocity);
    }




    Vector2 AdvectVec(Vector2Int coords,
        Vector2[,] velocity,
        Vector2[,] quantity,   // quantity to advect from
        float dissipation = 1)
    {
        // follow the velocity field "back in time"
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int swCorner = Vector2Int.FloorToInt(pos);
        swCorner.x = Mathf.Clamp(swCorner.x, 1, size - 1);
        swCorner.y = Mathf.Clamp(swCorner.y, 1, size - 1);
        //Get adjacent velocities
        Vector2 ne = quantity[swCorner.x + 1, swCorner.y + 1];
        Vector2 nw = quantity[swCorner.x, swCorner.y + 1];
        Vector2 se = quantity[swCorner.x + 1, swCorner.y];
        Vector2 sw = quantity[swCorner.x, swCorner.y];

        // interpolate and write to the output fragment
        Vector2 northLerp = Vector2.Lerp(nw, ne, pos.x - swCorner.x);
        Vector2 southLerp = Vector2.Lerp(sw, se, pos.x - swCorner.x);
        return Vector2.Lerp(southLerp, northLerp, pos.y - swCorner.y) * dissipation;
    }
    float AdvectF(Vector2Int coords,
        Vector2[,] velocity,
        float[,] quantity,      // quantity to advect from
        float dissipation = 1)
    {
        // follow the velocity field "back in time"
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int swCorner = Vector2Int.FloorToInt(pos);
        swCorner.x = Mathf.Clamp(swCorner.x, 1, size - 1);
        swCorner.y = Mathf.Clamp(swCorner.y, 1, size - 1);
        //Get adjacent velocities
        float ne = quantity[swCorner.x + 1, swCorner.y + 1];
        float nw = quantity[swCorner.x, swCorner.y + 1];
        float se = quantity[swCorner.x + 1, swCorner.y];
        float sw = quantity[swCorner.x, swCorner.y];

        // interpolate and write to the output fragment
        float northLerp = Mathf.Lerp(nw, ne, pos.x - swCorner.x);
        float southLerp = Mathf.Lerp(sw, se, pos.x - swCorner.x);
        float returnVal = Mathf.Lerp(southLerp, northLerp, pos.y - swCorner.y) * dissipation;

        return returnVal;
    }


    ///(x[i-1,j]+x[i+1,j]+x[i,j-1]+x[i,j+1]+ab[i,j]) / beta
    ///-(1/dencityConstant)∇Preasure
    //v = p
    //specialP = D
    float Jacobi(Vector2Int coords,
        float alpha,
        float inverseBeta,
        float[,] x,   // x vector (Ax = b)
        float[,] b)   // b vector (Ax = b)
    {
        // left, right, bottom, and top x samples
        float xL = x[coords.x - 1, coords.y];
        float xR = x[coords.x + 1, coords.y];
        float xB = x[coords.x, coords.y - 1];
        float xT = x[coords.x, coords.y + 1];

        // b sample, from center
        float bC = b[coords.x, coords.y];

        // evaluate Jacobi iteration
        return (xL + xR + xB + xT + alpha * bC) * inverseBeta;
    }
    Vector2 Jacobi(Vector2Int coords,
    float alpha,
    float inverseBeta,
    Vector2[,] x,   // x vector (Ax = b)
    Vector2[,] b)   // b vector (Ax = b)
    {
        // left, right, bottom, and top x samples
        Vector2 xL = x[coords.x - 1, coords.y];
        Vector2 xR = x[coords.x + 1, coords.y];
        Vector2 xB = x[coords.x, coords.y - 1];
        Vector2 xT = x[coords.x, coords.y + 1];

        // b sample, from center
        Vector2 bC = b[coords.x, coords.y];

        // evaluate Jacobi iteration
        return (xL + xR + xB + xT + alpha * bC) * inverseBeta;
    }


    float ComputeDivergence(Vector2Int coords,
        Vector2[,] w,  // vector field
        float halfInverseCellSize = 1)
    {
        float wL = w[coords.x - 1, coords.y].x;
        float wR = w[coords.x + 1, coords.y].x;
        float wB = w[coords.x, coords.y - 1].y;
        float wT = w[coords.x, coords.y + 1].y;

        return halfInverseCellSize * (wR - wL + wT - wB);
    }
    Vector2 ComputeGradient(Vector2Int coords,   // grid coordinates
        float[,] p)
    {
        float pL = p[coords.x - 1, coords.y];
        float pR = p[coords.x + 1, coords.y];
        float pB = p[coords.x, coords.y - 1];
        float pT = p[coords.x, coords.y + 1];

        return new Vector2(pR - pL, pT - pB) * size * 0.5f;
    }
    Vector2 Boundary(Vector2Int coords,    // grid coordinates
       int size)  // state field
    {
        if (coords.x <= 0 || coords.x >= size - 1 || coords.y <= 0 || coords.y >= size - 1)
        {
            return Vector2.zero;
        }
        return Vector2.one;
    }

    int Index(int x, int y) => Mathf.Clamp(x + y * size, 0, size * size - 1);
}
