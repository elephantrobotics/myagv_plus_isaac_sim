# Script Editor 片段: 场景级动力学参数
#   三个开关全 False 时纯只读, 不改 stage
#   一次只开一个, 每次改完 Stop -> Play 重测, 记下是哪个旋钮起的作用
#   tune_dynamics.py 的 base 惯量 / 轮 maxForce 两段新 URDF 已自带, 不要再跑

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import omni.usd

APPLY_RATE = True
APPLY_ITERS = False
APPLY_FRICTION = False

PHYSICS_HZ = 60
POS_ITERS = 32
VEL_ITERS = 4
STATIC_FRICTION = 0.9
DYNAMIC_FRICTION = 0.8

stage = omni.usd.get_context().get_stage()

print("APPLY rate/iters/friction =", APPLY_RATE, APPLY_ITERS, APPLY_FRICTION)
print("")

for prim in stage.Traverse():
    if not prim.IsA(UsdPhysics.Scene):
        continue
    print("scene", prim.GetPath())
    if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
        ps = PhysxSchema.PhysxSceneAPI(prim)
        print("  timeStepsPerSecond =", ps.GetTimeStepsPerSecondAttr().Get())
    else:
        print("  no PhysxSceneAPI, using engine default 60")
    if APPLY_RATE:
        ps = PhysxSchema.PhysxSceneAPI.Apply(prim)
        ps.CreateTimeStepsPerSecondAttr(PHYSICS_HZ)
        print("  -> set", PHYSICS_HZ)

print("")
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        continue
    print("articulation", prim.GetPath())
    if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        a = PhysxSchema.PhysxArticulationAPI(prim)
        print("  solver iters  pos =", a.GetSolverPositionIterationCountAttr().Get(),
              " vel =", a.GetSolverVelocityIterationCountAttr().Get())
    else:
        print("  no PhysxArticulationAPI, using engine default")
    if APPLY_ITERS:
        a = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        a.CreateSolverPositionIterationCountAttr(POS_ITERS)
        a.CreateSolverVelocityIterationCountAttr(VEL_ITERS)
        print("  -> set pos", POS_ITERS, "vel", VEL_ITERS)

print("")
mats = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.MaterialAPI)]
print("PhysicsMaterial count =", len(mats))
for p in mats:
    m = UsdPhysics.MaterialAPI(p)
    print(" ", p.GetPath())
    print("    static =", m.GetStaticFrictionAttr().Get(),
          " dynamic =", m.GetDynamicFrictionAttr().Get(),
          " restitution =", m.GetRestitutionAttr().Get())
if not mats:
    print("  no physics material, everything uses PhysX default friction (~0.5)")

if APPLY_FRICTION:
    path = "/PhysicsMaterial_rubber"
    prim = stage.GetPrimAtPath(path)
    if not prim:
        prim = UsdShade.Material.Define(stage, path).GetPrim()
    m = UsdPhysics.MaterialAPI.Apply(prim)
    m.CreateStaticFrictionAttr().Set(STATIC_FRICTION)
    m.CreateDynamicFrictionAttr().Set(DYNAMIC_FRICTION)
    m.CreateRestitutionAttr().Set(0.0)
    px = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    px.CreateFrictionCombineModeAttr().Set("max")
    print("")
    print("created", path, "static", STATIC_FRICTION, "dynamic", DYNAMIC_FRICTION, "combine max")

    mat = UsdShade.Material(prim)
    n = 0
    for target in stage.Traverse():
        p = target.GetPath().pathString
        name = target.GetName()
        hit = name.endswith("_link") and "_roller_" in name
        hit = hit or (name == "GroundPlane")
        if not hit:
            continue
        UsdShade.MaterialBindingAPI.Apply(target)
        UsdShade.MaterialBindingAPI(target).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        n += 1
    print("bound to", n, "prims (36 roller links + ground = 37)")

    checked = 0
    resolved = 0
    for target in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not target.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if "_roller_" not in target.GetPath().pathString:
            continue
        checked += 1
        bound = UsdShade.MaterialBindingAPI(target).ComputeBoundMaterial("physics")[0]
        if bound and bound.GetPrim().IsValid():
            resolved += 1
    print("roller colliders resolving material:", resolved, "/", checked)
    if resolved < checked:
        print("inheritance not applied; de-instancing needed")

print("")
print("after editing: Stop -> Play and re-measure")
