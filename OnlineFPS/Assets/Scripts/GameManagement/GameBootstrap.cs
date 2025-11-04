using Unity.NetCode;
using UnityEngine;

namespace OnlineFPS
{
    [UnityEngine.Scripting.Preserve]
    public class GameBootstrap : ClientServerBootstrap
    {
        public static GameBootstrap Instance;

        public override bool Initialize(string defaultWorldName)
        {
            Instance = this;

            // Create the Game Manager and handle all initialization there
            GameManager gameManagerPrefab = Resources.Load<GameManager>("GameManager");
            GameManager gameManager = GameObject.Instantiate<GameManager>(gameManagerPrefab);
            gameManager.OnInitialize();

            return true;
        }
    }
}
