using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ballSpawner : MonoBehaviour
{
    [SerializeField] GameObject gObject;

    Camera camera;
    private void Start()
    {
        camera = Camera.main;
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Vector2 pos = camera.ScreenToWorldPoint(Input.mousePosition);
            Instantiate(gObject, pos, Quaternion.identity);

        }
    }
}
