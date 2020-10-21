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
        public static int Diffusion = 2;
        public static int AddForce = 3;
        public static int Divergence = 4;
        public static int Gradient = 5;
    };
    struct ReadWriteIndexes
    {
        public int READ;
        public int Write;
    }

    [SerializeField] ComputeShader computeShader;
    [SerializeField] int size = 500;
    [SerializeField] float viscosity;
    [SerializeField] float diffusionJacobiItterations = 40;
    [SerializeField] float projectionJacobiItterations = 50;

    ComputeBuffer VelocityFieldBuffer;
    ComputeBuffer QuantityBuffer_IN;
    ComputeBuffer QuantityBuffer_OUT;
    ComputeBuffer DiffusionBuffer_IN;
    ComputeBuffer DiffusionBuffer_OUT;



    Vector2[][,] velocityField;
    ReadWriteIndexes velRW;
    const int W = 2;
    Vector2[,] quantity;
    float dx;
    float dxNegativeSquare;
    Vector2 previousMousePosition = new Vector2();
    private void Awake()
    {
        velocityField = new Vector2[3][,]
       {
            new Vector2[size,size],
            new Vector2[size,size],
            new Vector2[size,size]
       };
        velRW = new ReadWriteIndexes { READ = 0, Write = 1 };
        quantity = new Vector2[size, size];
        computeShader.SetInt("size", size);
        dx = 1.0f / size;
        dxNegativeSquare = -dx * dx;
        computeShader.SetFloat("projectionAlpha", dxNegativeSquare);
        computeShader.SetFloat("halfInverseCellSize", size * 0.5f);
    }

    void FlipReadWrite(ReadWriteIndexes rwIndex)
    {
        int temp = rwIndex.READ;
        rwIndex.READ = rwIndex.Write;
        rwIndex.Write = temp;
    }

    void SettupBuffers()
    {
        VelocityFieldBuffer = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Advect, "Velocity", VelocityFieldBuffer);

        VelocityFieldBuffer = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Advect, "Velocity", VelocityFieldBuffer);

        QuantityBuffer_IN = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Advect, "AdvectQuantityIn", QuantityBuffer_IN);

        QuantityBuffer_IN = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Advect, "AdvectQuantityOut", QuantityBuffer_OUT);

        DiffusionBuffer_IN = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Diffusion, "diffusionOut", DiffusionBuffer_IN);

        DiffusionBuffer_OUT = new ComputeBuffer(size * size, sizeof(float) * 2);
        computeShader.SetBuffer(Kernels.Diffusion, "diffusionOut", DiffusionBuffer_OUT);
    }

    private void FixedUpdate()
    {
        float diffAlpha = (dx * dx) / (viscosity * Time.deltaTime);
        float jacobiInverseBeta = 1 / (4 + diffAlpha);
        computeShader.SetFloat("deltaTime", Time.fixedDeltaTime);





        //Advect
        VelocityFieldBuffer.SetData(velocityField[velRW.READ]);
        QuantityBuffer_IN.SetData(velocityField[velRW.READ]);
        QuantityBuffer_OUT.SetData(velocityField[velRW.Write]);
        computeShader.Dispatch(0, size, size, 0);

        QuantityBuffer_IN.SetData(quantity);
        QuantityBuffer_OUT.SetData(quantity);
        computeShader.Dispatch(Kernels.Advect, size, size, 0);

        FlipReadWrite(velRW);

        //Diffusion
        computeShader.SetFloat("diffusionAlpha", diffAlpha);
        computeShader.SetFloat("diffusionInverseBeta", jacobiInverseBeta);
        for (int i = 0; i < diffusionJacobiItterations; i++)
        {
            DiffusionBuffer_IN.SetData(velocityField[velRW.READ]);
            DiffusionBuffer_OUT.SetData(velocityField[velRW.Write]);
            computeShader.Dispatch(Kernels.Diffusion, size, size, 0);
        }

        //Force Addition
        float scale = (float)Screen.width / (float)size;
        Vector2 mousePos = new Vector2((Input.mousePosition.x / scale), (Input.mousePosition.y / scale));
        computeShader.SetVector("position", mousePos);
        computeShader.SetVector("direction", mousePos - previousMousePosition);
        computeShader.Dispatch(Kernels.AddForce, size, size, 0);
        previousMousePosition = mousePos;

        //Projection Divergence




        //Projection Jacobi




        //Projection Gradient


    }
}
