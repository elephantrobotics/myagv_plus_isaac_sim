# Script Editor 片段: 导入后体检
#   跑 tune_rollers.py / tune_dynamics.py 之前先跑这个, 任何一项 FAIL 就别往下走
#   遍历带 TraverseInstanceProxies, 否则实例化的碰撞体查不到

from pxr import Usd, UsdPhysics
import omni.usd

stage = omni.usd.get_context().get_stage()

roots = []
bodies = []
joints = []
roller_joints = []
colliders = []
roller_colliders = []
wheel_colliders = []
base_body = None

for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    name = prim.GetName()
    path = prim.GetPath().pathString
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        roots.append(path)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        bodies.append(path)
        if name == "base_link":
            base_body = prim
    if prim.IsA(UsdPhysics.Joint):
        joints.append(name)
        if "_roller_" in name:
            roller_joints.append(name)
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        colliders.append(path)
        if "_roller_" in path:
            roller_colliders.append(path)
        elif "_wheel_link" in path:
            wheel_colliders.append(path)


def report(label, ok, detail):
    tag = "PASS" if ok else "FAIL"
    print("[" + tag + "] " + label + ": " + str(detail))


report("articulation root", len(roots) == 1, roots)
report("rigid bodies", len(bodies) == 42, str(len(bodies)) + " expect 42")
report("joints", len(joints) == 41, str(len(joints)) + " expect 41")
report("roller joints", len(roller_joints) == 36, str(len(roller_joints)) + " expect 36")
report("roller colliders", len(roller_colliders) == 36, str(len(roller_colliders)) + " expect 36")
report("wheel hub no collider", len(wheel_colliders) == 0, str(len(wheel_colliders)) + " expect 0")

if base_body:
    print("     base rigid body = " + base_body.GetPath().pathString)
    inertia = UsdPhysics.MassAPI(base_body).GetDiagonalInertiaAttr().Get()
    mass = UsdPhysics.MassAPI(base_body).GetMassAttr().Get()
    ok = inertia is not None and abs(inertia[2] - 0.0628) < 0.005
    report("base_link inertia", ok, str(inertia) + " mass=" + str(mass))
else:
    report("base rigid body", False, "not found")

approx = {}
for path in colliders:
    prim = stage.GetPrimAtPath(path)
    a = None
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        a = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
    approx[a] = approx.get(a, 0) + 1
print("     collision approximation = " + str(approx))

print("")
print("--- fl_roller_0_link subtree ---")
for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    if prim.GetName() != "fl_roller_0_link":
        continue
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        apis = [s for s in child.GetAppliedSchemas() if "Physics" in s or "Collision" in s]
        print("  " + child.GetPath().pathString
              + "  type=" + child.GetTypeName()
              + "  inst=" + str(child.IsInstance())
              + "  " + str(apis))
    break
