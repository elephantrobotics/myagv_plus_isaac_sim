# Script Editor 片段: 导入后一次性配置, 每次重新导入或 Isaac 崩溃重开都要跑
#   1) 滚子关节驱动清零 -> 滚子必须自由空转, 否则没有横移能力
#   2) 轮关节速度驱动   -> stiffness=0, damping 当增益; 导入值 0.001 等同没有驱动
#   3) 滚子碰撞体       -> 解实例化后设 contactOffset; 默认 0.02m 是滚子半径的 2.5 倍,
#                          接触判定过糊会把相邻滚子一起约束, 侧向自由度被锁死
#   4) 接触材质         -> 默认摩擦约 0.5, 横移时驱动力靠 45 度滚子接触传递, 摩擦不足会打滑
#
# Stop 状态下跑, 跑完 Play 验证, 满意后 File->Save 固化

from pxr import Usd, Gf, UsdPhysics, UsdShade, PhysxSchema
import omni.usd

VERIFY_ONLY = True

WHEEL_DAMPING = 0.8
WHEEL_MAX_FORCE = 1.35
ROLLER_DAMPING = 5e-5
ROLLER_SPIN_INERTIA_SCALE = 1.0    # 1.0 = 不放大; 球接触稳定后应该用不上
ROLLER_MAX_VEL_DEG = 5730.0   # = 100 rad/s; 单位是 deg/s, 实测确认
CONTACT_OFFSET = 0.002
REST_OFFSET = 0.0
STATIC_FRICTION = 0.9
DYNAMIC_FRICTION = 0.8
MATERIAL_PATH = "/PhysicsMaterial_rubber"
EXPECT_PHYSICS_HZ = 60  # 终审暂用 60; 实测 240 方向精度好 100 倍(±0.1° vs ±10°), 后期再优化

stage = omni.usd.get_context().get_stage()

if VERIFY_ONLY:
    ok = True
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            hz = None
            if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                hz = PhysxSchema.PhysxSceneAPI(prim).GetTimeStepsPerSecondAttr().Get()
            print("physics rate =", hz, "(expect", EXPECT_PHYSICS_HZ, ")")
            ok = ok and hz == EXPECT_PHYSICS_HZ
    for side in ("fl", "fr", "rl", "rr"):
        for prim in stage.Traverse():
            if prim.GetName() != side + "_wheel_joint":
                continue
            d = UsdPhysics.DriveAPI.Get(prim, "angular")
            dm = d.GetDampingAttr().Get() if d else None
            mf = d.GetMaxForceAttr().Get() if d else None
            print(side, "damping =", dm, " maxForce =", mf, "(expect 0.8 / 1.35)")
            ok = ok and dm is not None and abs(dm - WHEEL_DAMPING) < 1e-6
    n_cap = 0
    n_off = 0
    for prim in stage.Traverse():
        nm = prim.GetName()
        if "_roller_" in nm and nm.endswith("_joint"):
            if prim.HasAPI(PhysxSchema.PhysxJointAPI):
                v = PhysxSchema.PhysxJointAPI(prim).GetMaxJointVelocityAttr().Get()
                if v:
                    n_cap += 1
        if prim.HasAPI(PhysxSchema.PhysxCollisionAPI) and "_roller_" in prim.GetPath().pathString:
            if PhysxSchema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Get():
                n_off += 1
    print("roller maxJointVelocity set =", n_cap, "/36")
    print("roller contactOffset set =", n_off, "/36")
    n_sphere = sum(1 for p in stage.Traverse(Usd.TraverseInstanceProxies())
                   if p.HasAPI(UsdPhysics.CollisionAPI)
                   and "_roller_" in p.GetPath().pathString
                   and str(p.GetTypeName()) == "Sphere")
    print("roller colliders are spheres =", n_sphere, "/36")
    mats = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.MaterialAPI)]
    print("physics materials =", len(mats), "(expect 1)")
    ok = ok and n_cap == 36 and n_off == 36 and len(mats) == 1
    print("")
    print("all ready" if ok else "incomplete; set VERIFY_ONLY=False and rerun")
else:
    roller_joints = 0
    cleared = 0
    wheels = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name.endswith("_wheel_joint"):
            wheels[name] = prim
            continue
        if "_roller_" not in name or not name.endswith("_joint"):
            continue
        roller_joints += 1
        for token in ("angular", "linear"):
            d = UsdPhysics.DriveAPI.Get(prim, token)
            if d:
                d.CreateTargetVelocityAttr().Set(0.0)
                d.CreateStiffnessAttr().Set(0.0)
                d.CreateDampingAttr().Set(ROLLER_DAMPING)
                d.CreateMaxForceAttr().Set(0.0)
                cleared += 1
        japi = PhysxSchema.PhysxJointAPI.Apply(prim)
        japi.CreateJointFrictionAttr().Set(0.0)
        japi.CreateMaxJointVelocityAttr().Set(ROLLER_MAX_VEL_DEG)

    print("1) roller joints", roller_joints, "(expect 36), drives cleared", cleared)
    print("   damping =", ROLLER_DAMPING, " maxJointVelocity =", ROLLER_MAX_VEL_DEG,
          "deg/s =", round(ROLLER_MAX_VEL_DEG / 57.29578, 1), "rad/s (soft limit, transient overshoot allowed)")

    for side in ("fl", "rl", "fr", "rr"):
        key = side + "_wheel_joint"
        prim = wheels.get(key)
        if prim is None:
            print("   WARNING", key, "not found")
            continue
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(prim, "angular")
        d.CreateStiffnessAttr().Set(0.0)
        d.CreateDampingAttr().Set(WHEEL_DAMPING)
        d.CreateMaxForceAttr().Set(WHEEL_MAX_FORCE)
        d.CreateTargetVelocityAttr().Set(0.0)
    print("2) wheel drive stiffness=0 damping =", WHEEL_DAMPING, " maxForce =", WHEEL_MAX_FORCE,
          " targetVelocity=0")

    links = [p for p in stage.Traverse()
             if "_roller_" in p.GetName() and p.GetName().endswith("_link")]
    uninst = 0
    for p in links:
        if p.IsInstanceable():
            p.SetInstanceable(False)
            uninst += 1

    colliders = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if "_roller_" not in prim.GetPath().pathString:
            continue
        colliders.append(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr().Set(CONTACT_OFFSET)
        api.CreateRestOffsetAttr().Set(REST_OFFSET)
    scaled = 0
    for prim in (stage.Traverse() if ROLLER_SPIN_INERTIA_SCALE != 1.0 else []):
        nm = prim.GetName()
        if "_roller_" not in nm or not nm.endswith("_link"):
            continue
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
        mass_api = UsdPhysics.MassAPI(prim)
        inertia = mass_api.GetDiagonalInertiaAttr().Get()
        if inertia is None:
            continue
        vals = [float(v) for v in inertia]
        axial = vals.index(min(vals))
        if vals[axial] < 1e-6:
            vals[axial] = vals[axial] * ROLLER_SPIN_INERTIA_SCALE
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*vals))
            scaled += 1
    print("3b) roller spin inertia scaled", ROLLER_SPIN_INERTIA_SCALE, "x:", scaled, "(expect 36)")

    print("3) de-instanced", uninst, "links; colliders set", len(colliders), "(expect 36)")
    print("   contactOffset =", CONTACT_OFFSET, )

    mat_prim = stage.GetPrimAtPath(MATERIAL_PATH)
    if not mat_prim:
        mat_prim = UsdShade.Material.Define(stage, MATERIAL_PATH).GetPrim()
    m = UsdPhysics.MaterialAPI.Apply(mat_prim)
    m.CreateStaticFrictionAttr().Set(STATIC_FRICTION)
    m.CreateDynamicFrictionAttr().Set(DYNAMIC_FRICTION)
    m.CreateRestitutionAttr().Set(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(mat_prim).CreateFrictionCombineModeAttr().Set("max")

    mat = UsdShade.Material(mat_prim)
    bound = 0
    for target in stage.Traverse():
        name = target.GetName()
        hit = (name.endswith("_link") and "_roller_" in name) or name == "GroundPlane"
        if not hit:
            continue
        UsdShade.MaterialBindingAPI.Apply(target)
        UsdShade.MaterialBindingAPI(target).Bind(
            mat, UsdShade.Tokens.weakerThanDescendants, "physics")
        bound += 1
    print("4) friction material static =", STATIC_FRICTION, " dynamic =", DYNAMIC_FRICTION,
          "; bound", bound, "prims (expect 37)")

    print("")
    if roller_joints == 36 and len(colliders) == 36 and bound == 37:
        print("all good. verify at Play then File->Save")
    else:
        print("count mismatch; check the WARNING above")
