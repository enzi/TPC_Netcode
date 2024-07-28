using System;
using Unity.Entities;

[Serializable]
public struct GameSetup : IComponentData
{
    public Entity CharacterPrefab;
    public Entity PlayerPrefab;
    public Entity CameraPrefab;
}