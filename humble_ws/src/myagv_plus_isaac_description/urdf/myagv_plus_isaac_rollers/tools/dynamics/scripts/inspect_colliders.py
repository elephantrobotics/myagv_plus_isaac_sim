# Script Editor 片段: 统计当前 stage 里所有碰撞体的几何类型和尺寸
#   只读, 不改任何东西; 任何机器人都能用, 用来对照官方模型和我们自己的模型

from pxr import Usd, UsdPhysics, UsdGeom
import omni.usd

stage = omni.usd.get_context().get_stage()

rows = []
for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    t = str(prim.GetTypeName())
    info = ""
    if t == "Sphere":
        info = "r=" + str(round(float(UsdGeom.Sphere(prim).GetRadiusAttr().Get() or 0), 5))
    elif t == "Capsule":
        c = UsdGeom.Capsule(prim)
        info = ("r=" + str(round(float(c.GetRadiusAttr().Get() or 0), 5))
                + " h=" + str(round(float(c.GetHeightAttr().Get() or 0), 5)))
    elif t == "Cylinder":
        c = UsdGeom.Cylinder(prim)
        info = ("r=" + str(round(float(c.GetRadiusAttr().Get() or 0), 5))
                + " h=" + str(round(float(c.GetHeightAttr().Get() or 0), 5)))
    elif t == "Cube":
        info = "size=" + str(UsdGeom.Cube(prim).GetSizeAttr().Get())
    elif prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        info = "approx=" + str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get())
    rows.append((t, info, prim.GetPath().pathString))

kinds = {}
for t, info, path in rows:
    kinds[t] = kinds.get(t, 0) + 1
print("total colliders =", len(rows))
print("by type =", kinds)

seen = {}
for t, info, path in rows:
    key = t + "  " + info
    seen[key] = seen.get(key, 0) + 1
print("")
print("grouped by type+size:")
for key in sorted(seen, key=lambda k: -seen[k]):
    print("  ", seen[key], "x ", key)

print("")
print("one example path per group:")
shown = {}
for t, info, path in rows:
    if shown.get(t, 0) >= 2:
        continue
    shown[t] = shown.get(t, 0) + 1
    print("  ", t, info)
    print("     ", path)

bodies = [p for p in stage.Traverse(Usd.TraverseInstanceProxies())
          if p.HasAPI(UsdPhysics.RigidBodyAPI)]
rev = [p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
print("")
print("rigid bodies =", len(bodies), "  revolute joints =", len(rev))
print("first 15 revolute:", rev[:15])
