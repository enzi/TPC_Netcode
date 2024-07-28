using DOTS_TPC;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using Unity.CharacterController;
using Unity.NetCode;

[UpdateInGroup(typeof(GhostInputSystemGroup))]
[WorldSystemFilter(WorldSystemFilterFlags.ClientSimulation)]
public partial class ThirdPersonPlayerInputsSystem : SystemBase
{
    protected override void OnCreate()
    {
        RequireForUpdate<NetworkTime>();
        RequireForUpdate(SystemAPI.QueryBuilder().WithAll<ThirdPersonPlayer, ThirdPersonPlayerInputs>().Build());
    }

    protected override void OnUpdate()
    {
        foreach (var (playerInputs, player) in SystemAPI.Query<RefRW<ThirdPersonPlayerInputs>, ThirdPersonPlayer>().WithAll<GhostOwnerIsLocal>())
        {
            playerInputs.ValueRW.MoveInput = new float2
            {
                x = (Input.GetKey(KeyCode.D) ? 1f : 0f) + (Input.GetKey(KeyCode.A) ? -1f : 0f),
                y = (Input.GetKey(KeyCode.W) ? 1f : 0f) + (Input.GetKey(KeyCode.S) ? -1f : 0f),
            };
            
            NetworkInputUtilities.AddInputDelta(ref playerInputs.ValueRW.CameraLookInput.x, Input.GetAxis("Mouse X"));
            NetworkInputUtilities.AddInputDelta(ref playerInputs.ValueRW.CameraLookInput.y, Input.GetAxis("Mouse Y"));
            NetworkInputUtilities.AddInputDelta(ref playerInputs.ValueRW.CameraZoomInput, -Input.mouseScrollDelta.y);

            playerInputs.ValueRW.JumpPressed = default;
            if (Input.GetKeyDown(KeyCode.Space))
            {
                playerInputs.ValueRW.JumpPressed.Set();
            }
        }
    }
}

/// <summary>
/// Apply inputs that need to be read at a variable rate
/// </summary>
[UpdateInGroup(typeof(PredictedSimulationSystemGroup))]
[UpdateAfter(typeof(PredictedFixedStepSimulationSystemGroup))]
[BurstCompile]
public partial struct ThirdPersonPlayerVariableStepControlSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<NetworkTime>();
        state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<ThirdPersonPlayer, ThirdPersonPlayerInputs>().Build());
    }
    
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        NetworkInputUtilities.GetCurrentAndPreviousTick(SystemAPI.GetSingleton<NetworkTime>(), out NetworkTick currentTick, out NetworkTick previousTick);
        
        foreach (var (playerInputsBuffer, player) in SystemAPI.Query<DynamicBuffer<InputBufferData<ThirdPersonPlayerInputs>>, ThirdPersonPlayer>().WithAll<Simulate>())
        {
            NetworkInputUtilities.GetCurrentAndPreviousTickInputs(playerInputsBuffer, currentTick, previousTick, out ThirdPersonPlayerInputs currentTickInputs, out ThirdPersonPlayerInputs previousTickInputs);
            
            if (SystemAPI.HasComponent<TPC_CameraTarget>(player.ControlledCamera))
            {
                ref var target = ref SystemAPI.GetComponentRW<TPC_CameraTarget>(player.ControlledCamera).ValueRW;
                target.Target = player.ControlledCharacter;
            }

            if (SystemAPI.HasComponent<OrbitCameraControl>(player.ControlledCamera))
            {
                OrbitCameraControl cameraControl = SystemAPI.GetComponent<OrbitCameraControl>(player.ControlledCamera);
                
                cameraControl.FollowedCharacterEntity = player.ControlledCharacter;
                cameraControl.LookDegreesDelta.x = NetworkInputUtilities.GetInputDelta(currentTickInputs.CameraLookInput.x, previousTickInputs.CameraLookInput.x);
                cameraControl.LookDegreesDelta.y = NetworkInputUtilities.GetInputDelta(currentTickInputs.CameraLookInput.y, previousTickInputs.CameraLookInput.y);
                cameraControl.ZoomDelta = NetworkInputUtilities.GetInputDelta(currentTickInputs.CameraZoomInput, previousTickInputs.CameraZoomInput);
                
                SystemAPI.SetComponent(player.ControlledCamera, cameraControl);
            }
        }
    }
}

/// <summary>
/// Apply inputs that need to be read at a fixed rate.
/// It is necessary to handle this as part of the fixed step group, in case your framerate is lower than the fixed step rate.
/// </summary>
[UpdateInGroup(typeof(PredictedFixedStepSimulationSystemGroup), OrderFirst = true)]
[BurstCompile]
public partial struct ThirdPersonPlayerFixedStepControlSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate(SystemAPI.QueryBuilder().WithAll<ThirdPersonPlayer, ThirdPersonPlayerInputs>().Build());
    }
    
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        foreach (var (playerInputs, player) in SystemAPI.Query<ThirdPersonPlayerInputs, ThirdPersonPlayer>().WithAll<Simulate>())
        {
            if (SystemAPI.HasComponent<ThirdPersonCharacterControl>(player.ControlledCharacter))
            {
                ThirdPersonCharacterControl characterControl = SystemAPI.GetComponent<ThirdPersonCharacterControl>(player.ControlledCharacter);

                var transform = SystemAPI.GetComponent<LocalTransform>(player.ControlledCharacter);
                var characterComponent = SystemAPI.GetComponent<ThirdPersonCharacterComponent>(player.ControlledCharacter);
                float3 characterUp = MathUtilities.GetUpFromRotation(transform.Rotation);
                
                // Get camera rotation, since our movement is relative to it.
                quaternion cameraRotation = quaternion.identity;
                if (characterComponent.RelativeMovement)
                {
                    if (SystemAPI.HasComponent<OrbitCamera>(player.ControlledCamera))
                    {
                        // Camera rotation is calculated rather than gotten from transform, because this allows us to 
                        // reduce the size of the camera ghost state in a netcode prediction context.
                        // If not using netcode prediction, we could simply get rotation from transform here instead.
                        OrbitCamera orbitCamera = SystemAPI.GetComponent<OrbitCamera>(player.ControlledCamera);
                        cameraRotation = OrbitCameraUtilities.CalculateCameraRotation(characterUp, orbitCamera.PlanarForward, orbitCamera.PitchAngle);
                    }

                    if (SystemAPI.HasComponent<TPC_NetcodeInput>(player.ControlledCamera))
                    {
                        var tpcNetcodeInput = SystemAPI.GetComponent<TPC_NetcodeInput>(player.ControlledCamera);
                        var eulerAngles = new float3(tpcNetcodeInput.CameraAngles.x, tpcNetcodeInput.CameraAngles.y, 0);

                        cameraRotation = math.mul(
                            quaternion.LookRotationSafe(tpcNetcodeInput.PlanarForward, characterUp),
                            quaternion.Euler(eulerAngles));
                    }
                }
                else
                {
                    cameraRotation = transform.Rotation;
                }
                
                float3 cameraForwardOnUpPlane = math.normalizesafe(MathUtilities.ProjectOnPlane(MathUtilities.GetForwardFromRotation(cameraRotation), characterUp));
                float3 cameraRight = MathUtilities.GetRightFromRotation(cameraRotation);

                // Move
                characterControl.MoveVector = (playerInputs.MoveInput.y * cameraForwardOnUpPlane) + (playerInputs.MoveInput.x * cameraRight);
                characterControl.MoveVector = MathUtilities.ClampToMaxLength(characterControl.MoveVector, 1f);

                // Jump
                characterControl.Jump = playerInputs.JumpPressed.IsSet;

                SystemAPI.SetComponent(player.ControlledCharacter, characterControl);
            }
        }
    }
}