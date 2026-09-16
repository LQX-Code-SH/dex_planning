"""RobotProfile 加载与描述四件套路径重写（封装方案 §4.2 / N10，实施计划步 4）。

职责边界：本模块不 import tesseract——纯文本/路径层。Robot 装载本身即 FK
冒烟（tess.Robot.from_files 对重写产物做真实解析）。

重写器：上游交付的 URDF/SRDF 内嵌绝对 file:// 路径（换机即失效，N10）。
resolve_description 把网格引用重定向到 profile 声明的 mesh_root、SRDF 的
插件 yaml 引用重定向到实际绝对路径，产物落在缓存目录：
  - TIENKUNG_PLANNING_CACHE_DIR 设置时持久缓存，键 = 重写后内容 SHA-256
  - 未设置时一次性临时目录（进程生命周期，OS 负责回收）
注意：tesseract 的 resource locator 不做百分号解码——file:// URL 即原始
文件系统路径，中文文件名（腕部相机右.STL）保留原始 UTF-8 字节（N10 修正）。
"""
import hashlib
import os
import re
import tempfile

import yaml

_DESC_KEYS = ("urdf", "srdf", "kinematics_yaml", "contact_manager_yaml")
_URL_RE = re.compile(r'filename="file://([^"]+)"')


class ProfileError(Exception):
    """profile 加载或描述校验失败（fail-fast：调用方无需继续）。"""


def load_profile(name=None):
    """按名字（内置 robots/ 下的 yaml）或路径加载 RobotProfile。"""
    name = name or os.environ.get("TIENKUNG_PLANNING_PROFILE", "tienkung_dex")
    if name.endswith((".yaml", ".yml")):
        path = name
    else:
        path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "robots", name + ".yaml")
    if not os.path.isfile(path):
        raise ProfileError(f"profile 不存在: {path}")
    with open(path, encoding="utf-8") as f:
        profile = yaml.safe_load(f)
    missing = [k for k in ("schema", "robot", "description", "groups", "fingers")
               if k not in profile]
    if missing:
        raise ProfileError(f"profile {path} 缺字段 {missing}")
    return profile


def find_description_root():
    """从本包位置上溯定位含 tienkung_dex/ 描述树的目录（仓库内部署）。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        if os.path.isfile(os.path.join(
                d, "tienkung_dex", "urdf", "tienkung_dex.hand.urdf")):
            return d
        d = os.path.dirname(d)
    raise ProfileError(
        "定位不到描述树 tienkung_dex/（可设 TIENKUNG_PLANNING_DESCRIPTION_ROOT）")


def resolve_description(profile, description_root=None, cache_dir=None):
    """四件套路径解析 + file:// 重写，返回 (urdf, srdf) 可装载路径。

    fail-fast 校验：四件套齐全、无 package:// 引用、SRDF 组数 = 声明值、
    mesh_root 存在、插件 yaml 引用可对账。
    """
    d = profile["description"]
    root = (description_root or os.environ.get("TIENKUNG_PLANNING_DESCRIPTION_ROOT")
            or find_description_root())
    root = os.path.abspath(root)   # 重写产物必须是绝对路径（与进程 cwd 无关）
    paths = {k: os.path.join(root, d[k]) for k in _DESC_KEYS}
    missing = sorted(k for k, p in paths.items() if not os.path.isfile(p))
    if missing:
        raise ProfileError(f"描述四件套缺失 {missing}（root={root}）")

    with open(paths["urdf"], encoding="utf-8") as f:
        urdf_text = f.read()
    with open(paths["srdf"], encoding="utf-8") as f:
        srdf_text = f.read()
    for name, text in (("urdf", urdf_text), ("srdf", srdf_text)):
        if "package://" in text:
            raise ProfileError(f"{name} 含 package:// 引用（N10：封装前须消除）")

    # 上游 srdf 的注释里含 "--"（tinyxml2 容忍、xml.etree 不容），故用正则数组
    groups = re.findall(r'<group\s+name="([^"]+)"', srdf_text)
    exp = d.get("expected_groups")
    if exp is not None and len(groups) != exp:
        raise ProfileError(f"srdf 组数 {len(groups)} != 声明 {exp}（fail-fast）")

    mesh_root = os.path.join(root, d["mesh_root"])
    if not os.path.isdir(mesh_root):
        raise ProfileError(f"mesh_root 不存在: {mesh_root}")

    # 网格引用：保留 mesh_root 下的相对路径（含 revo2_left_hand/ 等子目录），
    # 仅重定向根。实证（步 4 冒烟）：tesseract 的 resource locator 不做百分号
    # 解码——file:// URL 就是原始文件系统路径，中文文件名（腕部相机右.STL）
    # 必须保留原始 UTF-8 字节，按 RFC 3986 编码反而装载失败。修正 N10 预设。
    mesh_root_abs = os.path.abspath(mesh_root)
    mesh_url = "file://" + mesh_root_abs + "/"

    def _mesh_sub(m):
        old = os.path.abspath(m.group(1))
        rel = os.path.relpath(old, mesh_root_abs)
        if rel.startswith(".."):
            raise ProfileError(
                f"网格引用在 mesh_root 之外: {m.group(1)}（mesh_root={mesh_root_abs}）")
        return 'filename="{}{}"'.format(mesh_url, rel)

    n_mesh = len(_URL_RE.findall(urdf_text))
    urdf_out = _URL_RE.sub(_mesh_sub, urdf_text)   # 无网格（图元机器人）也合法

    # SRDF 插件 yaml 引用：按 basename 对账到四件套实际路径
    yaml_map = {os.path.basename(paths[k]): paths[k]
                for k in ("kinematics_yaml", "contact_manager_yaml")}

    def _yaml_sub(m):
        target = yaml_map.get(os.path.basename(m.group(1)))
        if target is None:
            raise ProfileError(
                f"srdf 引用了未知插件 yaml: {m.group(1)}（四件套之外的路径引用）")
        return 'filename="file://{}"'.format(target)

    srdf_out, n_yaml = _URL_RE.subn(_yaml_sub, srdf_text)

    key = hashlib.sha256((urdf_out + srdf_out).encode("utf-8")).hexdigest()
    cache_dir = cache_dir or os.environ.get("TIENKUNG_PLANNING_CACHE_DIR")
    out = (os.path.join(cache_dir, key) if cache_dir
           else tempfile.mkdtemp(prefix="tienkung_desc_"))
    os.makedirs(out, exist_ok=True)
    urdf_p = os.path.join(out, os.path.basename(paths["urdf"]))
    srdf_p = os.path.join(out, os.path.basename(paths["srdf"]))
    if not (os.path.isfile(urdf_p) and os.path.isfile(srdf_p)):
        with open(urdf_p, "w", encoding="utf-8") as f:
            f.write(urdf_out)
        with open(srdf_p, "w", encoding="utf-8") as f:
            f.write(srdf_out)
    print(f"[profile] {profile['robot']}: 网格 {n_mesh} 处 → {mesh_root}，"
          f"插件 yaml {n_yaml} 处重写；缓存 {'持久 ' if cache_dir else '临时 '}{out}")
    return urdf_p, srdf_p
