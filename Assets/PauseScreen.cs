using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PauseScreen : MonoBehaviour
{
    [SerializeField] GameObject panel;


    public bool IsPaused => panel.activeSelf;

    public static PauseScreen Instance;
    private void Awake()
    {
        Instance = this;
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            panel.SetActive(!panel.activeSelf);
        }
    }
}
