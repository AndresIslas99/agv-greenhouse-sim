"""PhysX scene configuration."""

from pxr import UsdPhysics, Sdf, Gf


def create_physics_scene(stage, cfg):
    """Configure PhysX scene with GPU acceleration."""
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    g = cfg["physics"]["gravity"]
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(abs(g[2]))

    prim = stage.GetPrimAtPath("/World/PhysicsScene")
    prim.CreateAttribute(
        "physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool
    ).Set(cfg["physics"]["gpu_dynamics"])
    prim.CreateAttribute(
        "physxScene:broadphaseType", Sdf.ValueTypeNames.Token
    ).Set("GPU" if cfg["physics"]["gpu_broadphase"] else "MBP")
    prim.CreateAttribute(
        "physxScene:gpuMaxRigidContactCount", Sdf.ValueTypeNames.Int
    ).Set(cfg["physics"]["gpu_max_contacts"])
    prim.CreateAttribute(
        "physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int
    ).Set(cfg["physics"]["timestep_hz"])
