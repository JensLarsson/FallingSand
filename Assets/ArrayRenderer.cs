using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArrayRenderer : MonoBehaviour
{
    public ComputeShader renderShader;
    private RenderTexture _target;
    int height, width;

    ComputeBuffer arrayBuffer;

    public void SetRenderableArray(Vector2[,] vectorArray/*, float[,] floatArray*/)
    {
        width = vectorArray.GetLength(0);
        height = vectorArray.GetLength(1);
        renderShader.SetInt("Width", width);
        renderShader.SetInt("Height", height);
        arrayBuffer = new ComputeBuffer(width * height, sizeof(float) * 2);
    }

    // Start is called before the first frame update
    void Awake()
    {

    }

    // Update is called once per frame
    void Update()
    {

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


        int threadGroupsX = Mathf.CeilToInt(width);
        int threadGroupsY = Mathf.CeilToInt(height);
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
