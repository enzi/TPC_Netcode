using Unity.NetCode;

public class AutoConnectBootstrap : ClientServerBootstrap
{
    public override bool Initialize(string defaultWorldName)
    {      
        AutoConnectPort = 7979;
        CreateDefaultClientServerWorlds();
        return true;
    }
}