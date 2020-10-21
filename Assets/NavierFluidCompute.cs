using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NavierFluidCompute : MonoBehaviour
{
    struct Kernels
    {
        public static int Advect = 0;
        public static int AdvectV = 1;
        public static int DiffusionJacobi = 2;
        public static int ProjectionJacobi = 3;
        public static int AddForce = 4;
        public static int Divergence = 5;
        public static int Gradient = 6;
    };
    struct ReadWriteIndexes
    {
        public int READ;
        public int WRITE;
        public ReadWriteIndexes(int read = 0, int write = 1)
        {
            READ = read; WRITE = write;
        }
    }

    [SerializeField] ComputeShader computeShader;
    [SerializeField] int size = 500;
    [SerializeField] float viscosity;
    [SerializeField] float diffusionJacobiItterations = 40;
    [SerializeField] float projectionJacobiItterations = 50;
    [SerializeField] float deltaTime;

    ComputeBuffer VelocityFieldBuffer;
    ComputeBuffer WvelocityBuffer;
    ComputeBuffer QuantityBuffer_IN;
    ComputeBuffer QuantityBuffer_OUT;
    ComputeBuffer DiffusionBuffer_X;
    ComputeBuffer DiffusionBuffer_OUT;
    ComputeBuffer DivergenceBuffer;
    ComputeBuffer PressureBuffer_IN;
    ComputeBuffer PressureBuffer_OUT;

    public Vector2[,] VelocityField => velocityField[RWvel.READ];
    public Vector3[,] Quantity => quantity;


    Vector2[][,] velocityField;
    ReadWriteIndexes RWvel = new ReadWriteIndexes();
    const int W = 2;

    float[][,] pressure;
    ReadWriteIndexes RWpressure = new ReadWriteIndexes();

    Vector3[,] quantity;
    float[,] divergence;



    float dx;
    float dxNegativeSquare;
    Vector2 previousMousePosition = new Vector2();

    public static NavierFluidCompute Instance;
    private void Awake()
    {
        Instance = this;


        velocityField = new Vector2[3][,]
       {
            new Vector2[size,size],
            new Vector2[size,size],
            new Vector2[size,size]
       };
        pressure = new float[2][,]
       {
            new float[size,size],
            new float[size,size]
       };
        quantity = new Vector3  [size, size];

        for (int x = 0; x < size; x++)
        {
            for (int y = 0; y < size; y++)
            {
                quantity[x, y] = new Vector2(((x + y) / 4) % 2, 0);
            }
        }

        divergence = new float[size, size];
        dx = 1.0f / size;
        dxNegativeSquare = -dx * dx;
        computeShader.SetInt("size", size);
        computeShader.SetFloat("projectionAlpha", dxNegativeSquare);
        computeShader.SetFloat("halfInverseCellSize", size * 0.5f);
        computeShader.SetVector("position", new Vector2((float)size * 0.5f, (float)size * 0.5f));

        SettupBuffers();
    }
    private void Start()
    {
        Blipper.Instance.SettupShader(size);
    }

    void FlipReadWrite(ReadWriteIndexes rwIndex)
    {
        int temp = rwIndex.READ;
        rwIndex.READ = rwIndex.WRITE;
        rwIndex.WRITE = temp;
    }

    void SettupBuffers()
    {
        VelocityFieldBuffer = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Advect, "Velocity", VelocityFieldBuffer);
        computeShader.SetBuffer(Kernels.AdvectV, "Velocity", VelocityFieldBuffer);
        computeShader.SetBuffer(Kernels.AddForce, "Velocity", VelocityFieldBuffer);
        computeShader.SetBuffer(Kernels.Gradient, "Velocity", VelocityFieldBuffer);

        WvelocityBuffer = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.AdvectV, "Wvelocity", WvelocityBuffer);
        computeShader.SetBuffer(Kernels.DiffusionJacobi, "Wvelocity", WvelocityBuffer);
        computeShader.SetBuffer(Kernels.ProjectionJacobi, "Wvelocity", WvelocityBuffer);
        computeShader.SetBuffer(Kernels.Divergence, "Wvelocity", WvelocityBuffer);
        computeShader.SetBuffer(Kernels.Gradient, "Wvelocity", WvelocityBuffer);

        QuantityBuffer_IN = new ComputeBuffer(size * size, sizeof(float) * 3);
        computeShader.SetBuffer(Kernels.Advect, "AdvectQuantityIn", QuantityBuffer_IN);
        computeShader.SetBuffer(Kernels.AdvectV, "AdvectQuantityIn", QuantityBuffer_IN);

        QuantityBuffer_OUT = new ComputeBuffer(size * size, sizeof(float) * 3);
        computeShader.SetBuffer(Kernels.Advect, "AdvectQuantityOut", QuantityBuffer_OUT);
        computeShader.SetBuffer(Kernels.AdvectV, "AdvectQuantityOut", QuantityBuffer_OUT);

        DiffusionBuffer_X = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.DiffusionJacobi, "diffusionX", DiffusionBuffer_X);

        DiffusionBuffer_OUT = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.DiffusionJacobi, "diffusionOut", DiffusionBuffer_OUT);

        DivergenceBuffer = new ComputeBuffer(size * size, sizeof(float));
        computeShader.SetBuffer(Kernels.Divergence, "divergence", DivergenceBuffer);

        PressureBuffer_OUT = new ComputeBuffer(size * size, sizeof(float));
        computeShader.SetBuffer(Kernels.ProjectionJacobi, "PressureOUT", PressureBuffer_OUT);
        computeShader.SetBuffer(Kernels.Divergence, "PressureOUT", PressureBuffer_OUT);

        PressureBuffer_IN = new ComputeBuffer(size * size, sizeof(float));
        computeShader.SetBuffer(Kernels.ProjectionJacobi, "PressureIN", PressureBuffer_IN);


        PressureBuffer_OUT.SetData(pressure[RWpressure.WRITE]);
        DivergenceBuffer.SetData(divergence);
        WvelocityBuffer.SetData(velocityField[W]);
    }
    Vector2 forceDirection = new Vector2();
    float time;
    private void Update()
    {
        float diffAlpha = (dx * dx) / (viscosity * deltaTime);
        float jacobiInverseBeta = 1 / (4 + diffAlpha);
        computeShader.SetFloat("deltaTime", deltaTime);


        //Advect
        VelocityFieldBuffer.SetData(velocityField[RWvel.READ]);


        QuantityBuffer_IN.SetData(quantity);
        QuantityBuffer_OUT.SetData(quantity);
        computeShader.Dispatch(Kernels.Advect, size, size, 1);
        QuantityBuffer_OUT.GetData(quantity);

        QuantityBuffer_IN.SetData(velocityField[RWvel.READ]);
        QuantityBuffer_OUT.SetData(velocityField[RWvel.WRITE]);
        computeShader.Dispatch(Kernels.AdvectV, size, size, 1);
        QuantityBuffer_OUT.GetData(velocityField[RWvel.WRITE]);
        FlipReadWrite(RWvel);


        //Diffusion
        computeShader.SetFloat("diffusionAlpha", diffAlpha);
        computeShader.SetFloat("diffusionInverseBeta", jacobiInverseBeta);
        for (int i = 0; i < diffusionJacobiItterations; i++)
        {
            DiffusionBuffer_X.SetData(velocityField[RWvel.READ]);
            DiffusionBuffer_OUT.SetData(velocityField[RWvel.WRITE]);
            computeShader.Dispatch(Kernels.DiffusionJacobi, size, size, 1);
            DiffusionBuffer_OUT.GetData(velocityField[RWvel.WRITE]);
            FlipReadWrite(RWvel);
        }

        //Force Addition
        //float scale = (float)Screen.width / (float)size;
        //Vector2 mousePos = new Vector2((Input.mousePosition.x / scale), (Input.mousePosition.y / scale)); ;
        time += deltaTime * 10;
        forceDirection.x = Mathf.Sin(time);
        forceDirection.y = Mathf.Cos(time);
        computeShader.SetVector("direction", forceDirection);
        computeShader.Dispatch(Kernels.AddForce, size, size, 1);
        //previousMousePosition = mousePos;

        //Projection Divergence
        PressureBuffer_OUT.SetData(pressure[RWpressure.WRITE]);
        computeShader.Dispatch(Kernels.Divergence, size, size, 1);
        PressureBuffer_OUT.GetData(pressure[RWpressure.WRITE]);
        FlipReadWrite(RWpressure);

        //Projection Jacobi
        for (int i = 0; i < projectionJacobiItterations; i++)
        {
            PressureBuffer_IN.SetData(pressure[RWpressure.READ]);
            PressureBuffer_OUT.SetData(pressure[RWpressure.WRITE]);
            computeShader.Dispatch(Kernels.ProjectionJacobi, size, size, 1);
            PressureBuffer_OUT.GetData(pressure[RWpressure.WRITE]);
            FlipReadWrite(RWpressure);
        }


        //Projection Gradient
        computeShader.Dispatch(Kernels.Gradient, size, size, 1);
        VelocityFieldBuffer.GetData(velocityField[RWvel.WRITE]);
        FlipReadWrite(RWvel);


        Blipper.Instance.UpdateValues(VelocityField, quantity);
    }
}
