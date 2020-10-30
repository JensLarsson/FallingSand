using SFB;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;

//// dv / dt = -(v*∇)v-(1/D)∇p+s∇²v+F
//// ∇*v = 0   // v[x+1,y] - v[x-1,y] + s[x+1,y] - s[x-1,y] =0
//// ∇s = [(s[x+1,y].x-s[x-1,y].x), (s[x,y+1].y-s[x,y-1].y)]
//// ∇²s = ∇.dot(w) //Poisson-pressure exuation.



//// Step(v,t) = Projection(    Force(  Diffusion(    Advection(    v,t))))






//v = velocity
//p = Preasure
//s = viscosity
//D = dencityConstant


//Advection Term:   -(v*∇)v     //"self-advection of the velocity field"
//Pressure Term:    -(1/P)∇p    //acceleration from pressure
//Diffusion Term:   +s(∇^2)v    //"Viscosity is a measure of how resistive a fluid is to flow. This resistance results in diffusion of the momentum"


// u + ∇p = w
public class NavierFluid : MonoBehaviour
{
    [SerializeField] int size;
    [SerializeField] float forceMultiplier = 300;
    [SerializeField] float viscosity;
    [SerializeField] float spread = 200;
    [SerializeField] float deltaTime;
    [SerializeField] Texture2D image;
    [SerializeField] Slider forceSlider;
    [SerializeField] Slider forceSpreadSlider;
    [SerializeField] Slider viscositySlider;


    Vector2[][,] velocity;
    float[][,] pressure;
    Vector3[,] quantity;
    float[,] divergence;
    int READ = 0;
    int WRITE = 1;
    const int velocityW = 2;

    Camera camera;
    float time;

    public int Size => size;
    public Vector3[,] Quantity => quantity;
    public Vector2[,] VelocityField => velocity[READ];


    public static NavierFluid Instance;
    void Awake()
    {
        Instance = this;
        SetImage(image);

    }

    void SetImage(Texture2D texture)
    {
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
        quantity = new Vector3[size, size];

        divergence = new float[size, size];




        if (texture != null)
        {
            float imageScale = (float)(texture.width < texture.height ? texture.width : texture.height) / size;
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    var colour = texture.GetPixel(Mathf.FloorToInt((float)x * imageScale), Mathf.FloorToInt((float)y * imageScale));
                    quantity[x, y] = new Vector3(colour.r, colour.g, colour.b);
                }
            }
        }
        else
        {
            for (int x = 0; x < size / 2; x++)
            {
                for (int y = 0; y < size / 2; y++)
                {
                    quantity[x, y].x = 1;
                    quantity[x + size / 2, y].y = 1;
                    quantity[x, y + size / 2].z = 1;
                }
            }
        }
        Blipper.Instance.SettupShader(size);
    }

    //private void Start()
    //{
    //    var path = StandaloneFileBrowser.OpenFilePanel("Open File", "", "", false);
    //    if (path.Length != 0)
    //    {
    //        var fileContent = File.ReadAllBytes(path[0]);
    //        Texture2D texture = new Texture2D(2, 2);
    //        texture.LoadImage(fileContent);
    //        SetImage(texture);
    //    }
    //}


    void SwapReadWrite(object[] array)
    {
        object temp = array[0];
        array[0] = array[1];
        array[1] = temp;
    }

    Vector2 previousMousePos = new Vector2();
    Vector2 gravity = new Vector2(0, -9.8f);
    public void Update()
    {

        forceMultiplier = Mathf.Lerp(100, 2000, forceSlider.value);
        spread = Mathf.Lerp(2, 0.4f, forceSpreadSlider.value);
        viscosity = Mathf.Lerp(1e-18f, 1e-03f, viscositySlider.value);

        if (!PauseScreen.Instance.IsPaused)
        {

            time += deltaTime;
            float dx = 1.0f / size;
            float diffAlpha = (dx * dx) / (viscosity * deltaTime);
            float jacobiInverseBeta = 1 / (4 + diffAlpha);


            //////ADVECT
            System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
            {
                for (int y = 1; y < size - 1; y++)
                {
                    Vector2Int pos = new Vector2Int(x, y);
                    velocity[velocityW][x, y] = AdvectVec2(pos, velocity[READ], velocity[READ]);
                    velocity[WRITE][x, y] = velocity[velocityW][x, y];
                    quantity[x, y] = AdvectVec3(pos, velocity[READ], quantity);
                }
            });
            SwapReadWrite(velocity);




            //////DIFFUSION

            for (int i = 0; i < 20; i++)
            {
                System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
                {
                    for (int y = 1; y < size - 1; y++)
                    {
                        velocity[WRITE][x, y] = Jacobi(x, y, diffAlpha, jacobiInverseBeta, velocity[READ], velocity[velocityW]);
                    }
                });
                SwapReadWrite(velocity);
            }

            ////////EXTERNAL FORCES
            ///
            float scale = (float)Screen.height / size;

            Vector2 mousePos = Input.mousePosition / scale;
            Vector2 forceDirection = mousePos - previousMousePos;
            System.Threading.Tasks.Parallel.For(2, size - 2, (x) =>
            {
                for (int y = 2; y < size - 2; y++)
                {
                    float force = Mathf.Exp(-spread * Vector2.Distance(mousePos, new Vector2(x, y)));
                    velocity[WRITE][x, y] += forceDirection * deltaTime * force * forceMultiplier;

                    //Bounds
                    if (x == 2) velocity[WRITE][1, y] = -velocity[WRITE][x, y] * Vector2.right;
                    if (y == 2) velocity[WRITE][x, 1] = -velocity[WRITE][x, y] * Vector2.up;
                    if (y == size - 3) velocity[WRITE][x, size - 2] = -velocity[WRITE][x, y] * Vector2.up;
                    if (x == size - 3) velocity[WRITE][size - 2, y] = -velocity[WRITE][x, y] * Vector2.right;
                }
            });
            previousMousePos = mousePos;



            //////PROJECTION

            divergence = new float[size, size];
            System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
            {
                for (int y = 1; y < size - 1; y++)
                {
                    divergence[x, y] = ComputeDivergence(new Vector2Int(x, y), velocity[velocityW], size * 0.5f);
                    pressure[WRITE][x, y] = 0;
                }
            });
            SwapReadWrite(pressure);
            for (int i = 0; i < 30; i++)
            {
                System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
                {
                    for (int y = 1; y < size - 1; y++)
                    {
                        pressure[WRITE][x, y] = Jacobi(x, y, -(dx * dx), 0.25f, pressure[READ], divergence);
                    }
                });
                SwapReadWrite(pressure);
            }
            System.Threading.Tasks.Parallel.For(1, size - 1, (x) =>
            {
                for (int y = 1; y < size - 1; y++)
                {
                    velocity[WRITE][x, y] -= ComputeGradient(new Vector2Int(x, y), pressure[READ]);
                }
            });

            if (Input.GetKeyDown(KeyCode.Space))
            {
                velocity = new Vector2[3][,]
                {
                    new Vector2[size,size],
                    new Vector2[size,size],
                    new Vector2[size,size]
                };
            }
            SwapReadWrite(velocity);

            Blipper.Instance.UpdateValues(VelocityField, Quantity);
        }
    }




    Vector2 AdvectVec2(Vector2Int coords,
        Vector2[,] velocity,
        Vector2[,] quantity,   // quantity to advect from
        float dissipation = 1)
    {
        // follow the velocity field "back in time"
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int swCorner = Vector2Int.FloorToInt(pos);
        swCorner.x = Mathf.Clamp(swCorner.x, 0, size - 2);
        swCorner.y = Mathf.Clamp(swCorner.y, 0, size - 2);


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


    Vector3 AdvectVec3(Vector2Int coords,
        Vector2[,] velocity,
        Vector3[,] quantity,   // quantity to advect from
        float dissipation = 1)
    {
        // Reverse velocity
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int swCorner = Vector2Int.FloorToInt(pos);
        swCorner.x = Mathf.Clamp(swCorner.x, 0, size - 2);
        swCorner.y = Mathf.Clamp(swCorner.y, 0, size - 2);

        //Get adjacent quantities
        Vector3 ne = quantity[swCorner.x + 1, swCorner.y + 1];
        Vector3 nw = quantity[swCorner.x, swCorner.y + 1];
        Vector3 se = quantity[swCorner.x + 1, swCorner.y];
        Vector3 sw = quantity[swCorner.x, swCorner.y];

        // interpolate and output
        Vector3 northLerp = Vector3.Lerp(nw, ne, pos.x - swCorner.x);
        Vector3 southLerp = Vector3.Lerp(sw, se, pos.x - swCorner.x);
        return Vector3.Lerp(southLerp, northLerp, pos.y - swCorner.y) * dissipation;
    }
    float AdvectF(Vector2Int coords,
        Vector2[,] velocity,
        float[,] quantity,      // quantity to advect from
        float dissipation = 1)
    {
        // Reverse velocity
        Vector2 pos = coords - deltaTime * velocity[coords.x, coords.y];
        Vector2Int swCorner = Vector2Int.FloorToInt(pos);
        swCorner.x = Mathf.Clamp(swCorner.x, 0, size - 2);
        swCorner.y = Mathf.Clamp(swCorner.y, 0, size - 2);
        //Get adjacent quantities
        float ne = quantity[swCorner.x + 1, swCorner.y + 1];
        float nw = quantity[swCorner.x, swCorner.y + 1];
        float se = quantity[swCorner.x + 1, swCorner.y];
        float sw = quantity[swCorner.x, swCorner.y];

        // interpolate and output
        float northLerp = Mathf.Lerp(nw, ne, pos.x - swCorner.x);
        float southLerp = Mathf.Lerp(sw, se, pos.x - swCorner.x);
        float returnVal = Mathf.Lerp(southLerp, northLerp, pos.y - swCorner.y) * dissipation;

        return returnVal;
    }


    ///(x[i-1,j]+x[i+1,j]+x[i,j-1]+x[i,j+1]+ab[i,j]) / beta
    ///-(1/dencityConstant)∇Preasure
    //v = p
    //specialP = D
    float Jacobi(int xPos, int yPos,
        float alpha,
        float inverseBeta,
        float[,] x,   // x vector (Ax = b)
        float[,] b)   // b vector (Ax = b)
    {
        //// left, right, bottom, and top x samples
        //float xL = x[coords.x - 1, coords.y];
        //float xR = x[coords.x + 1, coords.y];
        //float xB = x[coords.x, coords.y - 1];
        //float xT = x[coords.x, coords.y + 1];

        //// b sample, from center
        //float bC = b[coords.x, coords.y];

        // evaluate Jacobi iteration
        return (x[xPos - 1, yPos] + x[xPos + 1, yPos] + x[xPos, yPos - 1] + x[xPos, yPos + 1] + alpha * b[xPos, yPos]) * inverseBeta;
    }
    Vector2 Jacobi(int xPos, int yPos,
    float alpha,
    float inverseBeta,
    Vector2[,] x,   // x vector (Ax = b)
    Vector2[,] b)   // b vector (Ax = b)
    {
        //// left, right, bottom, and top x samples
        //Vector2 xL = x[coords.x - 1, coords.y];
        //Vector2 xR = x[coords.x + 1, coords.y];
        //Vector2 xB = x[coords.x, coords.y - 1];
        //Vector2 xT = x[coords.x, coords.y + 1];

        //// b sample, from center
        //Vector2 bC = b[coords.x, coords.y];

        // evaluate Jacobi iteration
        return (x[xPos - 1, yPos] + x[xPos + 1, yPos] + x[xPos, yPos - 1] + x[xPos, yPos + 1] + alpha * b[xPos, yPos]) * inverseBeta;
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
