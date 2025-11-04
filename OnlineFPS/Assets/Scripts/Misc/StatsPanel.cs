using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.NetCode;
using UnityEngine;
using UnityEngine.UIElements;

namespace OnlineFPS
{
    public class StatsPanel : MonoBehaviour
    {
        public UIDocument UIDocument;

        public float StatsPollDuration = 1f;

        bool m_StatsEnabled;
        float m_AccumulatedDeltaTimes;
        int m_AccumulatedFPSFrames;
        float m_MaxFPSDeltaTime;
        float m_DeltaAvg;
        float m_DeltaWorst;

        VisualElement m_UIRoot;
        Label m_AvgFramerateLabel;
        Label m_WorstFramerateLabel;
        Label m_PingLabel;

        World m_ObservedWorld;

        void Awake()
        {
            m_StatsEnabled = true;

            m_UIRoot = UIDocument.rootVisualElement;
            m_AvgFramerateLabel = m_UIRoot.Query<Label>("FramerateAvg");
            m_WorstFramerateLabel = m_UIRoot.Query<Label>("FramerateMin");
            m_PingLabel = m_UIRoot.Query<Label>("Ping");
        }

        void Update()
        {
            if (m_StatsEnabled)
            {
                // Update observed world
                if (m_ObservedWorld == null || !m_ObservedWorld.IsCreated)
                {
                    m_ObservedWorld = null;
                    for (int i = 0; i < World.All.Count; i++)
                    {
                        World tmpWorld = World.All[i];
                        if (m_ObservedWorld == null)
                        {
                            m_ObservedWorld = tmpWorld;
                        }
                        else
                        {
                            if (tmpWorld == World.DefaultGameObjectInjectionWorld)
                            {
                                m_ObservedWorld = tmpWorld;
                            }

                            if (tmpWorld.IsClient())
                            {
                                m_ObservedWorld = tmpWorld;
                                break;
                            }
                        }
                    }
                }

                if (m_ObservedWorld != null && m_ObservedWorld.IsCreated)
                {
                    m_AccumulatedDeltaTimes += m_ObservedWorld.Time.DeltaTime;

                    // Framerate
                    {
                        m_MaxFPSDeltaTime = math.max(m_MaxFPSDeltaTime, m_ObservedWorld.Time.DeltaTime);
                        m_AccumulatedFPSFrames++;
                        m_DeltaAvg = m_AccumulatedDeltaTimes / m_AccumulatedFPSFrames;
                        m_DeltaWorst = m_MaxFPSDeltaTime;
                    }

                    // Update stats display
                    if (m_AccumulatedDeltaTimes >= StatsPollDuration)
                    {
                        // Framerate
                        {
                            m_AccumulatedFPSFrames = 0;
                            m_MaxFPSDeltaTime = 0f;

                            m_AvgFramerateLabel.text =
                                $"FPS avg: {1f / m_DeltaAvg:0} ({m_DeltaAvg * 1000f:0.0}ms)";
                            m_WorstFramerateLabel.text =
                                $"FPS min: {1f / m_DeltaWorst:0} ({m_DeltaWorst * 1000f:0.0}ms)";
                        }

                        // Ping
                        {
                            EntityQuery networkAckQuery = new EntityQueryBuilder(Allocator.Temp)
                                .WithAll<NetworkSnapshotAck>()
                                .Build(m_ObservedWorld.EntityManager);
                            if (networkAckQuery.HasSingleton<NetworkSnapshotAck>())
                            {
                                NetworkSnapshotAck networkAck = networkAckQuery.GetSingleton<NetworkSnapshotAck>();
                                m_PingLabel.text = $"Ping: {(int)networkAck.EstimatedRTT}";
                            }
                            else
                            {
                                m_PingLabel.text = $"Ping: ---";
                            }
                        }

                        m_AccumulatedDeltaTimes -= StatsPollDuration;
                    }
                }
            }
        }
    }
}
