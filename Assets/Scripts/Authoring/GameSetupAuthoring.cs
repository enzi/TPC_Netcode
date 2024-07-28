using Unity.Entities;
using UnityEngine;

public class GameSetupAuthoring : MonoBehaviour
{   
    public GameObject CharacterPrefab;
    public GameObject PlayerPrefab;
    public GameObject CameraPrefab;

    class Baker : Baker<GameSetupAuthoring>
    {
        public override void Bake(GameSetupAuthoring authoring)
        {
            AddComponent(GetEntity(authoring, TransformUsageFlags.None), new GameSetup
            {
                CharacterPrefab = GetEntity(authoring.CharacterPrefab, TransformUsageFlags.None),
                PlayerPrefab = GetEntity(authoring.PlayerPrefab, TransformUsageFlags.None),
                CameraPrefab = GetEntity(authoring.CameraPrefab, TransformUsageFlags.None),
            });
        }
    }
}
