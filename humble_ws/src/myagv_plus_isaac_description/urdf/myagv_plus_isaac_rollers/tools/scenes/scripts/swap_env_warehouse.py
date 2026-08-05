# 给当前场景挂一个官方环境引用,把无环境场景变成可测试场景。
# 只管 /Environment 一个 prim,不删任何东西,其余 prim 都不碰。
# 前提:当前打开的是无环境母版 myagv_plus_isaac_rollers_noenv.usda。
# 用法:Isaac Script Editor 打开本文件、在当前场景运行,确认无误后 File -> Save As。

import omni.usd
from isaacsim.storage.native import get_assets_root_path

# 换场景改这一行即可:
#   Simple_Warehouse/warehouse.usd                 简仓
#   Simple_Warehouse/warehouse_multiple_shelves.usd 多货架布局
#   Simple_Warehouse/full_warehouse.usd            大仓,压测
#   Hospital/hospital.usd                          走廊 / 多房间
#   Office/office.usd                              开放办公区
ENV_REL = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
ENV_PRIM = "/Environment"

stage = omni.usd.get_context().get_stage()

root = get_assets_root_path()
if root is None:
    raise RuntimeError("assets root = None (Isaac asset root not set / offline)")

# 先清空 /Environment 的引用再挂,重复运行即换场景、不叠加。
env = stage.DefinePrim(ENV_PRIM, "Xform")
env.GetReferences().ClearReferences()
env.GetReferences().AddReference(root + ENV_REL)
print("environment attached to", ENV_PRIM, "|", root + ENV_REL)
print("check the robot rests on the floor and is not stuck in shelves/forklifts; move the robot or /Environment in the GUI if needed")
print("once OK, File -> Save As to a per-environment usda")
