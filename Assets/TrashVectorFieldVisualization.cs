using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrashVectorFieldVisualization : MonoBehaviour
{
    public GameObject arrow;
    int size;
    GameObject[,] arrowArray;
    //Vector3[,] rotations;
    // Start is called before the first frame update
    void Start()
    {
        size = NavierFluid.Instance.Size;
        arrowArray = new GameObject[size, size];
        //rotations = new Vector3[size, size];
        float offset = 10f / size;
        for (int x = 0; x < size; x++)
        {
            for (int y = 0; y < size; y++)
            {
                arrowArray[x, y] = Instantiate(arrow, new Vector2(x, y) * offset - new Vector2(5 - offset * 0.5f, 5 - offset * 0.5f), Quaternion.identity);
                arrowArray[x, y].transform.localScale = new Vector3(1, 1, 1) * offset;
            }
        }
    }

    Vector3 rotation = new Vector3();
    Vector2 tempRotation = new Vector2();
    // Update is called once per frame
    void Update()
    {
        for (int x = 0; x < size; x++)
        {
            for (int y = 0; y < size; y++)
            {
                tempRotation = NavierFluid.Instance.VelocityField[x, y];
                rotation.z = Mathf.Atan2(tempRotation.y, tempRotation.x) * Mathf.Rad2Deg;
                arrowArray[x, y].transform.eulerAngles = rotation;
            }
        }
    }
}
