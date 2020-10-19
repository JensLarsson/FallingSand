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
    float[,] pressure;
    float[,] density;
    float[,] divergence;
    float viscosity;
    float deltaTime;
    int READ = 0;
    int WRITE = 1;
    const int velocityW = 2;

    public float[,] Density => density;
    public Vector2[,] Velocities => velocity[READ];
    public NavierFluid(int size, float dens, float visc)
    {
        this.size = size;
        this.viscosity = visc;
        velocity = new Vector2[3][,]
        {
            new Vector2[size,size],
            new Vector2[size,size],
            new Vector2[size,size]
        };
        pressure =
            new float[size, size];

        density =
            new float[size, size];
        divergence = new float[size, size];
        for (int x = 0; x < size; x++)
        {
            for (int y = 0; y < size; y++)
            {
                density[x, y] = 0.0f;
            }
        }
    }
    void SwapReadWrite(/*object[] array*/)
    {
        READ = READ == 0 ? 1 : 0;
        WRITE = WRITE == 0 ? 1 : 0;
        //object temp = array[0];
        //array[0] = array[1];
        //array[1] = temp;
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
        //if (velocity[READ][128, 128].y > 50) Debug.LogError(velocity[READ][128, 128]);
        deltaTime = timeStep;
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
        SwapReadWrite();




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
            SwapReadWrite();
        }

        ////////EXTERNAL FORCES
        Vector2 forceDirection = new Vector2(Mathf.Sin(Time.time), Mathf.Cos(Time.time));
        float mid = size / 2;
        Vector2 midPoint = new Vector2(mid, 2);
        //for (int x = 1; x < size - 1; x++)
        //{
        System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
        {
            for (int y = 1; y < size - 1; y++)
            {
                float force = Mathf.Exp(-200 * Vector2.Distance(midPoint, new Vector2(x, y)));
                velocity[WRITE][x, y] += Vector2.up * forceDirection*300;
            }
        });
        SwapReadWrite();



        //////PROJECTION

        float[,] divergence = new float[size, size];
        for (int x = 1; x < size - 1; x++)
        {
            for (int y = 1; y < size - 1; y++)
            {
                divergence[x, y] = ComputeDivergence(new Vector2Int(x, y), velocity[READ]);
                pressure[x, y] = 0;
            }
        }
        SwapReadWrite();
        for (int i = 0; i < 50; i++)
        {
            for (int x = 1; x < size - 1; x++)
            {
                for (int y = 1; y < size - 1; y++)
                {
                    pressure[x, y] = Jacobi(new Vector2Int(x, y), -diffAlpha, 0.25f, pressure, divergence);
                }
            }
        }
        for (int x = 1; x < size - 1; x++)
        {
            for (int y = 1; y < size - 1; y++)
            {
                velocity[WRITE][x, y] -= ComputeGradient(new Vector2Int(x, y), pressure);
                if (x == 1) velocity[WRITE][0, y] = -velocity[WRITE][x, y];
                if (y == 1) velocity[WRITE][x, 0] = -velocity[WRITE][x, y];
                if (x == size - 2) velocity[WRITE][size - 1, y] = -velocity[WRITE][x, y];
                if (y == size - 2) velocity[WRITE][size - 1, 0] = -velocity[WRITE][x, y];
            }
        }

        SwapReadWrite();
    }




    Vector2 AdvectVec(Vector2Int coords,
        Vector2[,] velocity,
        Vector2[,] quantity,   // quantity to advect from
        float dissipation = 1)
    {
        // follow the velocity field "back in time"
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int blCorner = Vector2Int.FloorToInt(pos);
        blCorner.x = Mathf.Clamp(blCorner.x, 1, size - 1);
        blCorner.y = Mathf.Clamp(blCorner.y, 1, size - 1);
        //Get adjacent velocities
        Vector2 ne = quantity[blCorner.x, blCorner.y];
        Vector2 nw = quantity[blCorner.x - 1, blCorner.y];
        Vector2 se = quantity[blCorner.x, blCorner.y - 1];
        Vector2 sw = quantity[blCorner.x - 1, blCorner.y - 1];

        // interpolate and write to the output fragment
        Vector2 lerpA = Vector2.Lerp(ne, nw, pos.x - blCorner.x);
        Vector2 lerpB = Vector2.Lerp(se, sw, pos.x - blCorner.x);
        return Vector2.Lerp(lerpA, lerpB, pos.y - blCorner.y) * dissipation;
    }
    float AdvectF(Vector2Int coords,
        Vector2[,] velocity,
        float[,] quantity,      // quantity to advect from
        float dissipation = 1)
    {
        // follow the velocity field "back in time"
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int blCorner = Vector2Int.FloorToInt(pos);
        blCorner.x = Mathf.Clamp(blCorner.x, 1, size - 1);
        blCorner.y = Mathf.Clamp(blCorner.y, 1, size - 1);
        //Get adjacent velocities
        float ne = quantity[blCorner.x, blCorner.y];
        float nw = quantity[blCorner.x - 1, blCorner.y];
        float se = quantity[blCorner.x, blCorner.y - 1];
        float sw = quantity[blCorner.x - 1, blCorner.y - 1];

        // interpolate and write to the output fragment
        float lerpA = Mathf.Lerp(ne, nw, pos.x - blCorner.x);
        float lerpB = Mathf.Lerp(se, sw, pos.x - blCorner.x);
        float returnVal = Mathf.Lerp(lerpA, lerpB, pos.y - blCorner.y) * dissipation;

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

        return halfInverseCellSize * (wR - wL + wT - wB) * 0.5f;
    }
    Vector2 ComputeGradient(Vector2Int coords,   // grid coordinates
        float[,] p)
    {
        float pL = p[coords.x - 1, coords.y];
        float pR = p[coords.x + 1, coords.y];
        float pB = p[coords.x, coords.y - 1];
        float pT = p[coords.x, coords.y + 1];

        return new Vector2(pR - pL, pT - pB) *1f;
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
