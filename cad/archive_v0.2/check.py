"""干涉检查：两两求交，列出交叠体积 > 1 mm³ 的零件对"""
import itertools
import build

items = [(n, s) for n, s, _ in build.items]
ELEC = ("树莓派", "MC02", "配电板", "DC-DC", "风扇", "电池", "预留")
hits = []
for (n1, s1), (n2, s2) in itertools.combinations(items, 2):
    b1, b2 = s1.BoundingBox(), s2.BoundingBox()
    if (b1.xmax < b2.xmin or b2.xmax < b1.xmin or b1.ymax < b2.ymin or b2.ymax < b1.ymin
            or b1.zmax < b2.zmin or b2.zmax < b1.zmin):
        continue
    if n1.startswith("预留") and n2.startswith("预留"):
        continue
    v = s1.intersect(s2).Volume()
    if v > 1.0:
        hits.append((v, n1, n2))
for v, n1, n2 in sorted(hits, reverse=True):
    print(f"{v:9.1f} mm3   {n1}  <->  {n2}")
print(f"{len(hits)} 处干涉")
