using UnityEngine;
using UnityEngine.UI;
using System;
using UnityEngine.InputSystem;

public struct FramerateCalculator
{
    int m_FramesCount;
    float m_FramesDeltaSum;
    float m_MinDeltaTimeForAvg;
    float m_MaxDeltaTimeForAvg;
    string[] m_FramerateStrings;

    public void Initialize()
    {
        m_MinDeltaTimeForAvg = Mathf.Infinity;
        m_MaxDeltaTimeForAvg = Mathf.NegativeInfinity;
        m_FramerateStrings = new string[1001];
        for (int i = 0; i < m_FramerateStrings.Length; i++)
        {
            if (i >= m_FramerateStrings.Length - 1)
            {
                m_FramerateStrings[i] = i + "+" + " (<" + (1000f / i).ToString("F") + "ms)";
            }
            else
            {
                m_FramerateStrings[i] = i + " (" + (1000f / i).ToString("F") + "ms)";
            }
        }
    }

    public void Update()
    {
        // Regular frames
        m_FramesCount++;
        m_FramesDeltaSum += Time.deltaTime;

        // Max and min
        if (Time.deltaTime < m_MinDeltaTimeForAvg)
        {
            m_MinDeltaTimeForAvg = Time.deltaTime;
        }

        if (Time.deltaTime > m_MaxDeltaTimeForAvg)
        {
            m_MaxDeltaTimeForAvg = Time.deltaTime;
        }
    }

    private string GetNumberString(int fps)
    {
        if (fps < m_FramerateStrings.Length - 1 && fps >= 0)
        {
            return m_FramerateStrings[fps];
        }
        else
        {
            return m_FramerateStrings[m_FramerateStrings.Length - 1];
        }
    }

    public void PollFramerate(out string avg, out string worst, out string best)
    {
        avg = GetNumberString(Mathf.RoundToInt(1f / (m_FramesDeltaSum / m_FramesCount)));
        worst = GetNumberString(Mathf.RoundToInt(1f / m_MaxDeltaTimeForAvg));
        best = GetNumberString(Mathf.RoundToInt(1f / m_MinDeltaTimeForAvg));

        m_FramesDeltaSum = 0f;
        m_FramesCount = 0;
        m_MinDeltaTimeForAvg = Mathf.Infinity;
        m_MaxDeltaTimeForAvg = Mathf.NegativeInfinity;
    }
}

public class FramerateMenuManager : MonoBehaviour
{
    [Header("Components")] public Canvas MainCanvas;
    public Text AvgFPS;
    public Text WorstFPS;
    public Text BestFPS;

    [Header("Misc")] public float FPSPollRate = 1f;

    FramerateCalculator m_FramerateCalculator = default;
    float m_LastTimePolledFPS = float.MinValue;
    bool m_HasVSync;

    void Start()
    {
        m_FramerateCalculator.Initialize();
        UpdateRenderSettings();
    }

    void Update()
    {
        // show hide
        if (Keyboard.current.f1Key.wasPressedThisFrame)
        {
            MainCanvas.gameObject.SetActive(!MainCanvas.gameObject.activeSelf);
        }

        if (Keyboard.current.f3Key.wasPressedThisFrame)
        {
            m_HasVSync = !m_HasVSync;
            UpdateRenderSettings();
        }

        // FPS
        m_FramerateCalculator.Update();
        if (Time.time >= m_LastTimePolledFPS + FPSPollRate)
        {
            m_FramerateCalculator.PollFramerate(out string avg, out string worst, out string best);
            AvgFPS.text = avg;
            WorstFPS.text = worst;
            BestFPS.text = best;

            m_LastTimePolledFPS = Time.time;
        }
    }

    void UpdateRenderSettings()
    {
        QualitySettings.vSyncCount = m_HasVSync ? 1 : 0;
    }
}