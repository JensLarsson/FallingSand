using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public enum TYPE : byte { A = 0, B, C, D, ASDASDAD }

public class Blipper : MonoBehaviour
{
    public ComputeShader renderShader;
    private RenderTexture _target;

    ComputeBuffer intBuffer;
    ComputeBuffer densityBuffer;
    ComputeBuffer velocityBuffer;

    SandSimulation simulation;

    [SerializeField] int width = 512;
    [SerializeField] int height = 512;
    [SerializeField] float viscocity = 1e-06f;
    NavierFluid fluid;
    private void Awake()
    {
        //simulation = new SandSimulation(width, height);
        fluid = new NavierFluid(width, viscocity);
        Debug.Log(width);
        densityBuffer = new ComputeBuffer(width * width, sizeof(float));
        velocityBuffer = new ComputeBuffer(width * width, sizeof(float) * 2);
        densityBuffer.SetData(fluid.Density);
        velocityBuffer.SetData(fluid.Velocities);
        renderShader.SetBuffer(0, "Density", densityBuffer);
        renderShader.SetBuffer(0, "Velocity", velocityBuffer);

        //intBuffer = new ComputeBuffer(width * height, sizeof(int));
        //SetPixelArray(simulation.PixelArray);
        //renderShader.SetBuffer(0, "Test", intBuffer);
    }
    Vector2Int previousMousePos;
    Vector2 EPSILON2 = new Vector2(0.000000001f, 000000001f);
    private void Update()
    {
        float scale = (float)Screen.width / (float)width;
        Vector2Int mousePos = new Vector2Int((int)(Input.mousePosition.x / scale), (int)(Input.mousePosition.y / scale));
        if (Input.GetMouseButton(0))
        {
            for (int x = -2; x < 3; x++)
            {
                for (int y = -2; y < 3; y++)
                {
                    Vector2 direction = (new Vector2(mousePos.x - previousMousePos.x, mousePos.y - previousMousePos.y) + EPSILON2).normalized * 1;
                    fluid.AddDencity(Mathf.Clamp(mousePos.x + x, 0, width - 1), Mathf.Clamp(mousePos.y + y, 0, height - 1), 100);
                    fluid.AddVelocity(Mathf.Clamp(mousePos.x, 0, width - 1), Mathf.Clamp(mousePos.y, 0, height - 1), direction);
                }
            }
        }
        previousMousePos = mousePos;
        //if (Input.GetKeyDown(KeyCode.Space))
        fluid.StepSimulation(0.0033f);
        densityBuffer.SetData(fluid.Density);
        velocityBuffer.SetData(fluid.Velocities);


        //if (Input.GetMouseButton(0))
        //{
        //    simulation.AddPixelToArray(Input.mousePosition, 1);
        //}
        //else if (Input.GetMouseButton(1))
        //{
        //    simulation.AddPixelToArray(Input.mousePosition, 2);
        //}


        //simulation.ItterateSimulation();
        //SetPixelArray(simulation.PixelArray);
    }
    public void SetPixelArray(PARTICLES[,] pixels)
    {
        intBuffer.SetData(pixels);
    }


    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Render(destination);
    }

    private void Render(RenderTexture destination)
    {
        // Make sure we have a current render target
        InitRenderTexture();
        // Set the target and dispatch the compute shader
        renderShader.SetTexture(0, "Result", _target);
        renderShader.SetInt("Width", width);
        renderShader.SetInt("Height", height);


        int threadGroupsX = Mathf.CeilToInt(width);
        int threadGroupsY = Mathf.CeilToInt(width);
        renderShader.Dispatch(0, threadGroupsX, threadGroupsY, 1);
        // Blit the result texture to the screen
        Graphics.Blit(_target, destination);
    }

    private void InitRenderTexture()
    {
        if (_target == null || _target.width != Screen.width || _target.height != Screen.height)
        {
            // Release render texture if we already have one
            if (_target != null)
                _target.Release();
            // Get a render target for Ray Tracing
            _target = new RenderTexture(Screen.width, Screen.height, 0,
                RenderTextureFormat.ARGBFloat, RenderTextureReadWrite.Linear);
            _target.enableRandomWrite = true;
            _target.Create();
        }
    }
}
