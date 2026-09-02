# xacro_to_urdf.py
#!/usr/bin/env python3
"""
将xacro文件转换为urdf文件的工具脚本
支持路径转换和资源文件复制功能
"""

import argparse
import subprocess
import xml.etree.ElementTree as ET
import os
import shutil
import sys
from urllib.parse import urlparse

def find_package_path(package_name, custom_package_paths=None):
    """
    查找ROS包路径
    
    Args:
        package_name (str): 包名
        custom_package_paths (dict): 自定义包路径映射
    
    Returns:
        str or None: 包路径，如果未找到则返回None
    """
    # 首先检查自定义路径
    if custom_package_paths and package_name in custom_package_paths:
        return custom_package_paths[package_name]
    
    # 常见的ROS包搜索路径
    search_paths = [
        '/opt/ros/humble/share',  # ROS 2 Humble
        '/opt/ros/foxy/share',    # ROS 2 Foxy
        '/opt/ros/galactic/share', # ROS 2 Galactic
        '/opt/ros/noetic/share',  # ROS 1 Noetic
        '/opt/ros/melodic/share', # ROS 1 Melodic
        os.path.expanduser('~/ros2_ws/install/share'),  # 用户工作空间
        os.path.expanduser('~/catkin_ws/src'),          # ROS 1 工作空间
    ]
    
    # 添加环境变量中的路径
    if 'AMENT_PREFIX_PATH' in os.environ:
        for path in os.environ['AMENT_PREFIX_PATH'].split(':'):
            search_paths.append(os.path.join(path, 'share'))
    
    if 'CMAKE_PREFIX_PATH' in os.environ:
        for path in os.environ['CMAKE_PREFIX_PATH'].split(':'):
            search_paths.append(os.path.join(path, 'share'))
    
    # 搜索包路径
    for base_path in search_paths:
        package_path = os.path.join(base_path, package_name)
        if os.path.exists(package_path):
            return package_path
            
    return None

def convert_paths_in_urdf(urdf_path, custom_package_paths=None):
    """
    转换URDF中的文件路径：
    - /opt/ros/ 下的包路径转换为绝对路径（添加file://前缀）
    - 其他路径下的包路径转换为相对路径（添加file://前缀）
    
    Args:
        urdf_path (str): URDF文件路径
        custom_package_paths (dict): 自定义包路径映射
    
    Returns:
        bool: 转换是否成功
    """
    try:
        # 解析URDF文件
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # 获取URDF文件所在目录
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        
        # 修改的路径数量
        modified_paths = 0
        
        # 查找所有mesh元素
        for mesh in root.iter('mesh'):
            if 'filename' in mesh.attrib:
                original_filename = mesh.attrib['filename']
                
                # 处理package://路径
                if original_filename.startswith('package://'):
                    # 提取包名
                    url_parsed = urlparse(original_filename)
                    path_parts = url_parsed.netloc + url_parsed.path
                    if path_parts.startswith('/'):
                        path_parts = path_parts[1:]
                    
                    path_components = path_parts.split('/', 1)
                    if len(path_components) >= 1:
                        package_name = path_components[0]
                        relative_path = path_components[1] if len(path_components) > 1 else ""
                        
                        # 查找包路径
                        package_path = find_package_path(package_name, custom_package_paths)
                        if package_path:
                            absolute_path = os.path.join(package_path, relative_path)
                            
                            # 判断是否在/opt/ros/目录下
                            if absolute_path.startswith('/opt/ros/'):
                                # 转换为绝对路径，添加file://前缀
                                new_path = f'file://{absolute_path}'
                                mesh.attrib['filename'] = new_path
                                print(f'Converted to absolute path: {original_filename} -> {new_path}')
                            else:
                                # 转换为相对路径，添加file://前缀
                                try:
                                    rel_path = os.path.relpath(absolute_path, urdf_dir)
                                    new_path = f'file://{rel_path}'
                                    mesh.attrib['filename'] = new_path
                                    print(f'Converted to relative path: {original_filename} -> {new_path}')
                                except ValueError:
                                    # 如果在不同的驱动器上（Windows），保持绝对路径
                                    new_path = f'file://{absolute_path}'
                                    mesh.attrib['filename'] = new_path
                                    print(f'Converted to absolute path: {original_filename} -> {new_path}')
                            
                            modified_paths += 1
                        else:
                            print(f'Warning: Package {package_name} not found for {original_filename}')
        
        # 保存修改后的URDF
        if modified_paths > 0:
            tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
            print(f'Successfully converted {modified_paths} paths in URDF')
        
        return True
        
    except Exception as e:
        print(f'Error converting paths: {str(e)}', file=sys.stderr)
        return False

def convert_xacro_to_urdf(xacro_path, urdf_path, custom_package_paths=None, auto_convert_paths=True):
    """
    使用xacro命令将xacro文件转换为urdf文件，可选择自动转换路径
    
    Args:
        xacro_path (str): 输入xacro文件路径
        urdf_path (str): 输出urdf文件路径
        custom_package_paths (dict): 自定义包路径映射
        auto_convert_paths (bool): 是否自动转换路径
    
    Returns:
        bool: 转换是否成功
    """
    try:
        # 执行xacro命令
        result = subprocess.run(['xacro', xacro_path], 
                              capture_output=True, text=True, check=True)
        
        # 写入URDF文件
        with open(urdf_path, 'w') as f:
            f.write(result.stdout)
            
        print(f"Successfully converted {xacro_path} to {urdf_path}")
        
        # 如果启用了自动路径转换，则进行转换
        if auto_convert_paths:
            if not convert_paths_in_urdf(urdf_path, custom_package_paths):
                return False
        
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"Error running xacro: {e.stderr}", file=sys.stderr)
        return False
    except Exception as e:
        print(f"Error writing URDF file: {str(e)}", file=sys.stderr)
        return False

def resolve_resource_path(filename, urdf_dir, custom_package_paths=None):
    """
    解析资源文件的绝对路径
    
    Args:
        filename (str): 文件名（可能是相对路径、绝对路径或package://路径）
        urdf_dir (str): URDF文件所在目录
        custom_package_paths (dict): 自定义包路径映射
    
    Returns:
        str or None: 解析后的绝对路径，如果无法解析则返回None
    """
    # 处理package://路径
    if filename.startswith('package://'):
        # 提取包名和路径
        url_parsed = urlparse(filename)
        path_parts = url_parsed.netloc + url_parsed.path
        if path_parts.startswith('/'):
            path_parts = path_parts[1:]
        
        path_components = path_parts.split('/', 1)
        if len(path_components) >= 1:
            package_name = path_components[0]
            relative_path = path_components[1] if len(path_components) > 1 else ""
            
            # 查找包路径
            package_path = find_package_path(package_name, custom_package_paths)
            if package_path:
                absolute_path = os.path.join(package_path, relative_path)
                if os.path.exists(absolute_path):
                    return absolute_path
                else:
                    print(f'Warning: File not found: {absolute_path}')
            else:
                print(f'Warning: Package {package_name} not found for {filename}')
    
    # 处理file://路径
    elif filename.startswith('file://'):
        absolute_path = filename[7:]  # 移除 'file://'
        if os.path.exists(absolute_path):
            return absolute_path
        else:
            print(f'Warning: File not found: {absolute_path}')
    
    # 处理绝对路径
    elif os.path.isabs(filename):
        if os.path.exists(filename):
            return filename
        else:
            print(f'Warning: File not found: {filename}')
    
    # 处理相对路径
    else:
        absolute_path = os.path.join(urdf_dir, filename)
        if os.path.exists(absolute_path):
            return absolute_path
        else:
            print(f'Warning: File not found: {absolute_path}')
    
    return None

def copy_referenced_resources(urdf_path, resource_output_dir, symlink_copy=False, custom_package_paths=None):
    """
    复制URDF引用的资源文件到指定目录并更新路径
    为每个mesh文件添加前缀：[link-name]_[visual/collision]_[num]_[source-file-name]
    
    Args:
        urdf_path (str): URDF文件路径
        resource_output_dir (str): 资源文件输出目录
        symlink_copy (bool): 是否创建符号链接而不是复制
        custom_package_paths (dict): 自定义包路径映射
    
    Returns:
        tuple: (是否成功, 复制的文件数量)
    """
    try:
        # 创建输出目录（如果不存在）
        os.makedirs(resource_output_dir, exist_ok=True)
        
        # 解析URDF文件
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # 获取URDF文件所在目录
        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        
        # 收集所有引用的资源文件及其上下文信息
        resource_files = {}  # 键为 (link_name, element_type, index, original_ref)，值为 src_path
        
        # 遍历所有link
        for link in root.iter('link'):
            link_name = link.attrib.get('name', 'unknown')
            
            # 处理visual元素中的mesh
            visual_index = 0
            for visual in link.iter('visual'):
                for mesh in visual.iter('mesh'):
                    if 'filename' in mesh.attrib:
                        filename = mesh.attrib['filename']
                        # 解析资源文件的绝对路径
                        absolute_path = resolve_resource_path(filename, urdf_dir, custom_package_paths)
                        if absolute_path:
                            key = (link_name, 'visual', visual_index, filename)
                            resource_files[key] = absolute_path
                visual_index += 1
            
            # 处理collision元素中的mesh
            collision_index = 0
            for collision in link.iter('collision'):
                for mesh in collision.iter('mesh'):
                    if 'filename' in mesh.attrib:
                        filename = mesh.attrib['filename']
                        # 解析资源文件的绝对路径
                        absolute_path = resolve_resource_path(filename, urdf_dir, custom_package_paths)
                        if absolute_path:
                            key = (link_name, 'collision', collision_index, filename)
                            resource_files[key] = absolute_path
                collision_index += 1
        
        # 复制或链接资源文件，并跟踪新文件名
        copied_files = {}  # 键为 (link_name, element_type, index, original_ref)，值为新文件名
        
        for (link_name, element_type, index, original_ref), src_path in resource_files.items():
            # 获取源文件的扩展名和基本名称
            src_basename = os.path.basename(src_path)
            src_name, src_ext = os.path.splitext(src_basename)
            
            # 生成新文件名：[link-name]_[visual/collision]_[num]_[source-file-name]
            new_filename = f'{link_name}_{element_type}_{index}_{src_basename}'
            dst_path = os.path.join(resource_output_dir, new_filename)
            
            # 检查目标文件是否已存在
            if os.path.exists(dst_path):
                print(f"Skipping {new_filename}: already exists in output directory")
                copied_files[(link_name, element_type, index, original_ref)] = new_filename
                continue
            
            if symlink_copy:
                # 创建符号链接
                os.symlink(src_path, dst_path)
                print(f'Created symlink: {dst_path} -> {src_path}')
            else:
                # 复制文件
                shutil.copy2(src_path, dst_path)
                print(f'Copied: {src_path} to {dst_path}')
            
            copied_files[(link_name, element_type, index, original_ref)] = new_filename
        
        # 更新URDF中的路径为新位置的相对路径
        if copied_files:
            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            updated_paths = 0
            
            # 重新遍历link并更新mesh路径
            for link in root.iter('link'):
                link_name = link.attrib.get('name', 'unknown')
                
                # 更新visual中的mesh
                visual_index = 0
                for visual in link.iter('visual'):
                    for mesh in visual.iter('mesh'):
                        if 'filename' in mesh.attrib:
                            original_ref = mesh.attrib['filename']
                            key = (link_name, 'visual', visual_index, original_ref)
                            if key in copied_files:
                                new_filename = copied_files[key]
                                # 更新为复制后文件的相对路径
                                new_rel_path = os.path.relpath(
                                    os.path.join(resource_output_dir, new_filename), 
                                    urdf_dir
                                )
                                mesh.attrib['filename'] = new_rel_path
                                print(f'Updated path: {original_ref} -> {new_rel_path}')
                                updated_paths += 1
                    visual_index += 1
                
                # 更新collision中的mesh
                collision_index = 0
                for collision in link.iter('collision'):
                    for mesh in collision.iter('mesh'):
                        if 'filename' in mesh.attrib:
                            original_ref = mesh.attrib['filename']
                            key = (link_name, 'collision', collision_index, original_ref)
                            if key in copied_files:
                                new_filename = copied_files[key]
                                # 更新为复制后文件的相对路径
                                new_rel_path = os.path.relpath(
                                    os.path.join(resource_output_dir, new_filename), 
                                    urdf_dir
                                )
                                mesh.attrib['filename'] = new_rel_path
                                print(f'Updated path: {original_ref} -> {new_rel_path}')
                                updated_paths += 1
                    collision_index += 1
            
            # 保存更新后的URDF
            tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
            print(f'Successfully updated {updated_paths} paths in URDF')
        
        print(f'Successfully processed {len(copied_files)} resource files')
        return True, len(copied_files)
        
    except Exception as e:
        print(f'Error copying resources: {str(e)}', file=sys.stderr)
        return False, 0

def strip_protocols(urdf_path):
    """
    去掉URDF中文件路径的file://前缀
    
    Args:
        urdf_path (str): URDF文件路径
    
    Returns:
        bool: 转换是否成功
    """
    try:
        # 解析URDF文件
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # 查找所有mesh元素
        modified_paths = 0
        for mesh in root.iter('mesh'):
            if 'filename' in mesh.attrib:
                original_filename = mesh.attrib['filename']
                
                # 去掉file://前缀
                if original_filename.startswith('file://'):
                    new_filename = original_filename[7:]  # 移除 'file://'
                    mesh.attrib['filename'] = new_filename
                    print(f'Stripped protocol from {original_filename} to {new_filename}')
                    modified_paths += 1
        
        # 保存修改后的URDF
        tree.write(urdf_path, encoding='utf-8', xml_declaration=True)
        print(f'Successfully stripped protocols from {modified_paths} paths')
        return True
        
    except Exception as e:
        print(f'Error stripping protocols: {str(e)}', file=sys.stderr)
        return False

def parse_package_mapping(mapping_str):
    """
    解析包映射字符串
    
    Args:
        mapping_str (str): 包映射字符串，格式为 package_name:=path
    
    Returns:
        tuple: (package_name, path)
    """
    if ':=' in mapping_str:
        parts = mapping_str.split(':=')
        if len(parts) == 2:
            return parts[0], parts[1]
    raise argparse.ArgumentTypeError(f"Invalid package mapping format: {mapping_str}. Use package_name:=path")

def main():
    parser = argparse.ArgumentParser(
        description='Convert xacro to urdf with additional options',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s robot.xacro
  %(prog)s robot.xacro -c ./meshes
  %(prog)s robot.xacro -p ur_description:=/opt/ros/humble/share/ur_description -c ./assets
  %(prog)s robot.xacro -s
        """
    )
    parser.add_argument('xacro_file', help='Input xacro file path')
    parser.add_argument('-p', '--package-path', action='append', type=parse_package_mapping,
                       help='Custom package path mapping (package_name:=path). Can be specified multiple times.')
    parser.add_argument('-s', '--strip-protocols', action='store_true',
                       help='Strip file:// protocol from paths')
    parser.add_argument('-c', '--copy-assets-to', type=str,
                       help='Copy referenced assets to directory and update paths')
    parser.add_argument('--symlink-copy', action='store_true',
                       help='Create symbolic links instead of copying assets')
    parser.add_argument('-o', '--output', type=str,
                       help='Output URDF file path (default: same directory as xacro file)')
    
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not os.path.exists(args.xacro_file):
        print(f"Error: Input file {args.xacro_file} does not exist", file=sys.stderr)
        return 1
    
    # 确定输出文件路径
    if args.output:
        urdf_file = args.output
    else:
        # 默认输出到xacro文件目录下
        xacro_dir = os.path.dirname(os.path.abspath(args.xacro_file))
        xacro_basename = os.path.splitext(os.path.basename(args.xacro_file))[0]
        urdf_file = os.path.join(xacro_dir, xacro_basename + '.urdf')
    
    # 创建输出目录（如果不存在）
    urdf_dir = os.path.dirname(os.path.abspath(urdf_file))
    if urdf_dir:
        os.makedirs(urdf_dir, exist_ok=True)
    
    # 解析自定义包路径
    custom_package_paths = {}
    if args.package_path:
        for package_name, path in args.package_path:
            custom_package_paths[package_name] = path
            print(f"Registered custom path for package {package_name}: {path}")
    
    # 执行转换（默认启用自动路径转换）
    success = convert_xacro_to_urdf(args.xacro_file, urdf_file, custom_package_paths, auto_convert_paths=True)
    if not success:
        return 1
    
    # 去掉协议前缀（只在指定-s参数时才执行）
    if args.strip_protocols:
        success = strip_protocols(urdf_file)
        if not success:
            return 1
    
    # 复制资源文件（如果指定了相关选项）
    if args.copy_assets_to:
        # 创建资源输出目录
        resource_output_dir = args.copy_assets_to
        os.makedirs(resource_output_dir, exist_ok=True)
        
        success, count = copy_referenced_resources(
            urdf_file, 
            resource_output_dir, 
            args.symlink_copy,
            custom_package_paths
        )
        if not success:
            return 1
    
    print("Conversion completed successfully")
    return 0

if __name__ == '__main__':
    sys.exit(main())