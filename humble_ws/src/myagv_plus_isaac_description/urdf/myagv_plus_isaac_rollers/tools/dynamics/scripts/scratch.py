# 一次性测试用, 内容随时被覆盖
# 跑法: Isaac Script Editor 打开本文件运行
#
# 当前内容: 列出 /Graph 下每张图的节点, 查有没有重复节点

import omni.graph.core as og

for g in og.get_all_graphs():
    p = g.get_path_to_graph()
    if not p.startswith("/Graph"):
        continue
    names = sorted(n.get_prim_path().split("/")[-1] for n in g.get_nodes())
    print("%s  (%d nodes)" % (p, len(names)))
    for n in names:
        print("    " + n)
    print("")
