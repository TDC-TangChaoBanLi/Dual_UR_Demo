#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import pathlib
import xml.etree.ElementTree as ET

import trimesh

# 可选：用于解析 package:// 路径
try:
    import rospkg
    _ROSPKG = rospkg.RosPack()
except ImportError:
    _ROSPKG = None


def resolve_mesh_path(urdf_path, filename):
    """
    将 URDF 中的 mesh 路径解析为绝对路径，同时保留路径风格信息。

    返回:
        abs_path: 绝对路径 (str)
        scheme: 'package' 或 'file'
        pkg: 若为 package:// 则为包名，否则 None
        rel_path: 对于 package://，为包内路径；否则为原始/相对路径
    """
    filename = filename.strip()
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

    if filename.startswith("package://"):
        scheme = "package"
        s = filename[len("package://"):]
        if "/" not in s:
            raise ValueError(f"无法解析 package:// 路径: {filename}")
        pkg, rel = s.split("/", 1)
        if _ROSPKG is None:
            raise RuntimeError(
                f"需要解析 package:// 路径 {filename}，但未安装 rospkg 库"
            )
        pkg_path = _ROSPKG.get_path(pkg)  # 若找不到包会抛异常
        abs_path = os.path.join(pkg_path, rel)
        return abs_path, scheme, pkg, rel

    # 普通文件路径或 file://
    scheme = "file"
    pkg = None
    rel = filename
    if filename.startswith("file://"):
        rel = filename[len("file://"):]
    # 绝对 / 相对
    if os.path.isabs(rel):
        abs_path = rel
    else:
        abs_path = os.path.join(urdf_dir, rel)

    return abs_path, scheme, pkg, rel


def build_urdf_mesh_filename(output_urdf_path,
                             dest_mesh_path,
                             scheme,
                             pkg,
                             rel_in_pkg,
                             mesh_root_dir_for_pkg):
    """
    根据转换后的 mesh 绝对路径，生成写回 URDF 的 filename 字符串。

    规则：
    - 若原本是 package:// 且 mesh 存回原包目录下（未改变根包路径），
      则继续使用 package:// 形式。
    - 否则，使用相对于输出 URDF 文件所在目录的相对路径。
    """
    output_urdf_dir = os.path.dirname(os.path.abspath(output_urdf_path))
    dest_mesh_path = os.path.abspath(dest_mesh_path)

    if scheme == "package" and mesh_root_dir_for_pkg is not None:
        dest_mesh_path_norm = os.path.normpath(dest_mesh_path)
        pkg_root_norm = os.path.normpath(mesh_root_dir_for_pkg)
        if os.path.commonpath([dest_mesh_path_norm, pkg_root_norm]) == pkg_root_norm:
            rel_inside_pkg = os.path.relpath(dest_mesh_path_norm, pkg_root_norm)
            rel_inside_pkg = rel_inside_pkg.replace(os.sep, "/")
            return f"package://{pkg}/{rel_inside_pkg}"

    rel_to_urdf = os.path.relpath(dest_mesh_path, output_urdf_dir)
    rel_to_urdf = rel_to_urdf.replace(os.sep, "/")
    return rel_to_urdf


def _export_with_scene_preserving_materials(source_path, dest_mesh_path):
    """
    针对 DAE 等多几何/多材质格式，尽可能保留材质信息导出 OBJ。
    """
    loaded = trimesh.load(source_path, force="scene", process=False)
    if isinstance(loaded, trimesh.Scene):
        scene = loaded
    else:
        # 万一 loader 没给 Scene，就包一层
        scene = trimesh.Scene(loaded)

    scene.export(dest_mesh_path)


def _fix_mtl_name(dest_mesh_path):
    """
    将 OBJ 同目录下的 'material.mtl' 改名为 '<stem>.mtl'，
    并修补 OBJ 文件里的 'mtllib material.mtl'。
    """
    obj_path = pathlib.Path(dest_mesh_path)
    obj_dir = obj_path.parent
    stem = obj_path.stem

    old_mtl = obj_dir / "material.mtl"
    if not old_mtl.exists():
        return

    new_mtl = obj_dir / f"{stem}.mtl"
    # 若已有同名 .mtl，说明可能之前修过，直接用
    if not new_mtl.exists():
        old_mtl.rename(new_mtl)

    # 修改 OBJ 中的 mtllib 行
    try:
        with obj_path.open("r", encoding="utf-8") as f:
            txt = f.read()
        txt = txt.replace("mtllib material.mtl", f"mtllib {stem}.mtl")
        with obj_path.open("w", encoding="utf-8") as f:
            f.write(txt)
    except Exception as e:
        print(f"[WARN] 修补 OBJ 中 mtllib 失败: {dest_mesh_path} -> {e}")


def convert_mesh(source_path, target_ext, mesh_dir):
    """
    使用 trimesh 将 source_path 转换为 target_ext（'obj' 或 'stl'）。
    返回转换后 mesh 的绝对路径。

    关键点：
    - 每个源 mesh 使用单独子目录 <mesh_dir>/<stem>/，防止 material.mtl/纹理互相覆盖。
    - DAE -> OBJ：保持 Scene，多材质尽可能保留。
    - OBJ 导出后，修补 mtl 文件名为 <stem>.mtl。
    """
    source_path = os.path.abspath(source_path)
    if not os.path.isfile(source_path):
        raise FileNotFoundError(f"找不到源 mesh 文件: {source_path}")

    stem = pathlib.Path(source_path).stem

    # 每个文件一个子目录，避免 material.mtl 互相覆盖
    base_mesh_dir = os.path.abspath(mesh_dir)
    dest_dir = os.path.join(base_mesh_dir, stem)
    os.makedirs(dest_dir, exist_ok=True)

    dest_mesh_path = os.path.join(dest_dir, f"{stem}.{target_ext.lower()}")

    if os.path.exists(dest_mesh_path):
        return dest_mesh_path

    print(f"[INFO] Convert {source_path} -> {dest_mesh_path}")

    src_ext = os.path.splitext(source_path)[1].lower()

    # 针对 DAE -> OBJ 特殊处理：用 Scene 导出
    if src_ext in [".dae", ".zae"] and target_ext.lower() == "obj":
        _export_with_scene_preserving_materials(source_path, dest_mesh_path)
        _fix_mtl_name(dest_mesh_path)
        return dest_mesh_path

    # 其它情况：通用处理
    loaded = trimesh.load(source_path, force=None, process=False)

    if isinstance(loaded, trimesh.Scene) and target_ext.lower() == "obj":
        loaded.export(dest_mesh_path)
        _fix_mtl_name(dest_mesh_path)
    else:
        if isinstance(loaded, trimesh.Scene):
            # STL 不支持材质，只能合并成一个 mesh
            mesh = trimesh.util.concatenate(tuple(
                g for g in loaded.geometry.values()
            ))
        else:
            mesh = loaded
        mesh.export(dest_mesh_path)

    return dest_mesh_path


def process_urdf(input_urdf, output_urdf=None, mesh_dir=None):
    input_urdf = os.path.abspath(input_urdf)
    if output_urdf is None:
        in_path = pathlib.Path(input_urdf)
        output_urdf = str(in_path.with_name(in_path.stem + "_converted.urdf"))
    else:
        output_urdf = os.path.abspath(output_urdf)

    print(f"[INFO] Input URDF : {input_urdf}")
    print(f"[INFO] Output URDF: {output_urdf}")

    tree = ET.parse(input_urdf)
    root = tree.getroot()

    # 缓存 (原filename字符串, 目标扩展名) -> 新 filename 字符串
    filename_cache = {}

    for tag_name, target_for in [("visual", "visual"), ("collision", "collision")]:
        for elem in root.iter(tag_name):
            geom = elem.find("geometry")
            if geom is None:
                continue
            mesh_elem = geom.find("mesh")
            if mesh_elem is None:
                continue

            filename = mesh_elem.get("filename")
            if not filename:
                continue

            try:
                abs_path, scheme, pkg, rel = resolve_mesh_path(input_urdf, filename)
            except Exception as e:
                print(f"[WARN] 无法解析 mesh 路径 '{filename}': {e}")
                continue

            ext = os.path.splitext(abs_path)[1].lower()

            # visual：不是 stl/obj -> 转 obj
            if target_for == "visual":
                if ext in [".stl", ".obj"]:
                    continue
                target_ext = "obj"
            # collision：不是 stl -> 转 stl
            else:
                if ext == ".stl":
                    continue
                target_ext = "stl"

            cache_key = (filename, target_ext)
            if cache_key in filename_cache:
                new_filename_str = filename_cache[cache_key]
                mesh_elem.set("filename", new_filename_str)
                continue

            # 确定 mesh 输出目录根
            if mesh_dir is not None:
                dest_root_dir = os.path.abspath(mesh_dir)
                mesh_root_dir_for_pkg = None
                if scheme == "package" and pkg is not None and _ROSPKG is not None:
                    mesh_root_dir_for_pkg = _ROSPKG.get_path(pkg)
            else:
                dest_root_dir = os.path.dirname(abs_path)
                mesh_root_dir_for_pkg = os.path.dirname(abs_path) if scheme == "package" else None

            try:
                dest_mesh_path = convert_mesh(abs_path, target_ext, dest_root_dir)
            except Exception as e:
                print(f"[ERROR] 转换 mesh 文件失败 '{filename}': {e}")
                continue

            new_filename_str = build_urdf_mesh_filename(
                output_urdf_path=output_urdf,
                dest_mesh_path=dest_mesh_path,
                scheme=scheme,
                pkg=pkg,
                rel_in_pkg=rel,
                mesh_root_dir_for_pkg=mesh_root_dir_for_pkg
            )

            filename_cache[cache_key] = new_filename_str
            mesh_elem.set("filename", new_filename_str)

    os.makedirs(os.path.dirname(output_urdf), exist_ok=True)
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)
    print("[INFO] URDF 处理完成.")


def main():
    parser = argparse.ArgumentParser(
        description="使用 trimesh 将 URDF 中的 mesh 批量转换为 OBJ / STL。"
    )
    parser.add_argument(
        "urdf",
        help="输入 URDF 文件路径"
    )
    parser.add_argument(
        "-o", "--output",
        help="输出 URDF 文件路径，默认为原 URDF 目录下新文件（xxx_converted.urdf）",
        default=None
    )
    parser.add_argument(
        "-m", "--mesh-dir",
        help="转换后的 mesh 文件存放路径，默认为原 mesh 所在文件夹（每个 mesh 一个子目录）",
        default=None
    )

    args = parser.parse_args()
    process_urdf(args.urdf, args.output, args.mesh_dir)


if __name__ == "__main__":
    main()
