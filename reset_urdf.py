#!/usr/bin/env python3
"""
URDF路径转换脚本 - 使用ROS2包查找方法
"""

import os
import re
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
import subprocess
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

class URDFPathConverter:
    def __init__(self, urdf_file, output_file=None):
        self.urdf_file = Path(urdf_file)
        self.output_file = Path(output_file) if output_file else self.urdf_file.with_suffix('.converted.urdf')
        
        # 包名到路径的映射缓存
        self.package_cache = {}
        
        # 用户自定义的包路径设置
        self.package_settings = {}
        
        # 默认设置：/opt/ros/下的包使用绝对路径，其他使用相对路径
        self.default_ros_path = "/opt/ros/"
        
    def set_package_setting(self, package_name, use_absolute=False):
        """设置特定包使用绝对路径还是相对路径"""
        self.package_settings[package_name] = use_absolute
        
    def find_package_path(self, package_name):
        """使用ROS2方法查找包路径（类似于启动文件中的方法）"""
        if package_name in self.package_cache:
            return self.package_cache[package_name]
            
        try:
            # 方法1: 使用ament_index_python（推荐，与启动文件相同的方法）
            package_path = get_package_share_directory(package_name)
            self.package_cache[package_name] = package_path
            return package_path
        except PackageNotFoundError:
            pass
            
        # try:
        #     # 方法2: 使用ros2 pkg prefix命令
        #     result = subprocess.run(
        #         ['ros2', 'pkg', 'prefix', package_name], 
        #         capture_output=True, text=True, check=True
        #     )
        #     package_path = result.stdout.strip()
        #     self.package_cache[package_name] = package_path
        #     return package_path
        # except (subprocess.CalledProcessError, FileNotFoundError):
        #     pass
            
        # 方法3: 在常见位置查找
        common_paths = [
            # ROS2 标准位置
            f"/opt/ros/{os.getenv('ROS_DISTRO', 'humble')}/share/{package_name}",
            # 开发工作空间位置
            f"{os.path.expanduser('~')}/ros2_ws/install/{package_name}",
            f"{os.path.expanduser('~')}/ros2_ws/src/{package_name}",
            # 当前目录相关位置
            f"./src/{package_name}",
            f"./install/{package_name}",
            # COLCON工作空间
            f"{os.getenv('COLCON_PREFIX_PATH', '').split(':')[0]}/{package_name}" if os.getenv('COLCON_PREFIX_PATH') else None,
        ]
        
        for path in common_paths:
            if path and Path(path).exists():
                self.package_cache[package_name] = path
                return path
                
        print(f"警告: 无法找到包 '{package_name}' 的路径")
        return None
    
    def should_use_absolute_path(self, package_name, full_path):
        """判断是否应该使用绝对路径"""
        # 1. 检查用户设置
        if package_name in self.package_settings:
            return self.package_settings[package_name]
            
        # 2. 检查是否在/opt/ros/下
        if str(full_path).startswith(self.default_ros_path):
            return True
            
        # 3. 检查是否在系统目录下
        system_dirs = ['/usr/', '/opt/', '/lib/', '/include/']
        if any(str(full_path).startswith(dir_path) for dir_path in system_dirs):
            return True
            
        # 4. 默认使用相对路径
        return False
    
    def resolve_path(self, filename, current_package=None):
        """解析文件路径，根据设置返回相对路径或绝对路径"""
        # 匹配 package:// 格式的路径
        package_pattern = r'package://([^/]+)/(.+)'
        package_match = re.match(package_pattern, filename)
        
        if package_match:
            package_name, file_path = package_match.groups()
            
            # 如果包名与当前包相同，使用相对路径
            if current_package and package_name == current_package:
                return file_path
            
            # 获取包路径
            package_path = self.find_package_path(package_name)
            if not package_path:
                print(f"警告: 无法解析包路径: {package_name}")
                return filename  # 无法解析，返回原路径
            
            full_path = Path(package_path) / file_path
            
            if not full_path.exists():
                print(f"警告: 文件不存在: {full_path}")
                # 但仍然继续处理路径转换
            
            if self.should_use_absolute_path(package_name, full_path):
                return str(full_path.resolve())
            else:
                # 计算相对于输出文件的路径
                try:
                    relative_path = os.path.relpath(full_path, self.output_file.parent)
                    return str(relative_path)
                except ValueError:
                    # 如果在不同驱动器上，使用绝对路径
                    return str(full_path.resolve())
        
        # 匹配 file:// 格式的路径
        file_pattern = r'file://(.+)'
        file_match = re.match(file_pattern, filename)
        
        if file_match:
            file_path = file_match.group(1)
            abs_path = Path(file_path)
            
            if abs_path.exists() or abs_path.is_absolute():
                # 检查是否应该使用绝对路径
                if self.should_use_absolute_path('', abs_path):
                    return str(abs_path.resolve())
                else:
                    # 计算相对于输出文件的路径
                    try:
                        relative_path = os.path.relpath(abs_path, self.output_file.parent)
                        return str(relative_path)
                    except ValueError:
                        return str(abs_path.resolve())
            else:
                print(f"警告: 文件不存在: {file_path}")
                return filename
        
        # 如果不是ROS格式的路径，直接返回
        return filename
    
    def detect_current_package(self):
        """尝试检测当前URDF文件所属的包"""
        # 通过查找package.xml文件来检测当前包
        current_dir = self.urdf_file.parent
        for parent_dir in [current_dir] + list(current_dir.parents):
            package_xml = parent_dir / 'package.xml'
            if package_xml.exists():
                try:
                    # 简单解析package.xml获取包名
                    with open(package_xml, 'r', encoding='utf-8') as f:
                        content = f.read()
                    name_match = re.search(r'<name>([^<]+)</name>', content)
                    if name_match:
                        return name_match.group(1)
                except:
                    pass
        return None
    
    def convert_urdf(self):
        """转换URDF文件中的路径"""
        print(f"正在转换URDF文件: {self.urdf_file}")
        
        # 检测当前包
        current_package = self.detect_current_package()
        if current_package:
            print(f"检测到包: {current_package}")
        
        # 解析URDF文件
        try:
            tree = ET.parse(self.urdf_file)
        except ET.ParseError as e:
            print(f"解析URDF文件失败: {e}")
            return
        
        root = tree.getroot()
        
        # 查找所有需要转换路径的元素
        converted_count = 0
        
        # 查找并转换mesh元素的filename属性
        for mesh in root.findall('.//mesh'):
            if 'filename' in mesh.attrib:
                original_path = mesh.attrib['filename']
                new_path = self.resolve_path(original_path, current_package)
                if new_path != original_path:
                    print(f"转换mesh路径: {original_path} -> {new_path}")
                    mesh.attrib['filename'] = new_path
                    converted_count += 1
        
        # 查找并转换其他可能包含路径的元素
        for element in root.iter():
            for attr_name in ['filename', 'uri', 'path']:
                if attr_name in element.attrib:
                    original_path = element.attrib[attr_name]
                    new_path = self.resolve_path(original_path, current_package)
                    if new_path != original_path:
                        print(f"转换{attr_name}路径: {original_path} -> {new_path}")
                        element.attrib[attr_name] = new_path
                        converted_count += 1
        
        # 保存转换后的文件
        try:
            # 确保输出目录存在
            self.output_file.parent.mkdir(parents=True, exist_ok=True)
            
            # 写入文件
            tree.write(self.output_file, encoding='utf-8', xml_declaration=True)
            
            # 美化XML格式
            # self.pretty_print_xml(self.output_file)
            
            print(f"转换完成! 共转换了 {converted_count} 个路径")
            print(f"输出文件: {self.output_file}")
            
        except Exception as e:
            print(f"保存文件失败: {e}")
        
    def pretty_print_xml(self, xml_file):
        """美化XML格式"""
        try:
            # 读取文件内容
            with open(xml_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 简单的美化：添加换行和缩进
            content = re.sub(r'>\s*<', '>\n<', content)
            content = re.sub(r'(\S)<', r'\1\n<', content)
            content = re.sub(r'>(\S)', r'>\n\1', content)
            
            # 写入美化后的内容
            with open(xml_file, 'w', encoding='utf-8') as f:
                f.write(content)
                
        except Exception as e:
            print(f"美化XML时出错: {e}")

def main():
    parser = argparse.ArgumentParser(
        description='转换URDF文件中的ROS包路径为相对路径或绝对路径',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 基本用法
  python convert_urdf_paths.py my_env.urdf
  
  # 指定输出文件
  python convert_urdf_paths.py my_env.urdf -o converted/my_env.urdf
  
  # 自定义包设置
  python convert_urdf_paths.py my_env.urdf --package my_env_description false --package realsense2_description true
  
  # 强制所有包使用绝对路径
  python convert_urdf_paths.py my_env.urdf --package "*" true
        """
    )
    
    parser.add_argument('input_file', help='输入的URDF文件路径')
    parser.add_argument('-o', '--output', help='输出文件路径（可选）')
    parser.add_argument('--package', action='append', nargs=2, 
                       metavar=('PACKAGE_NAME', 'ABSOLUTE'), 
                       help='设置包路径类型。PACKAGE_NAME可以是具体包名或"*"表示所有包，ABSOLUTE为true/false')
    
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not Path(args.input_file).exists():
        print(f"错误: 输入文件不存在: {args.input_file}")
        return
    
    # 创建转换器
    converter = URDFPathConverter(args.input_file, args.output)
    
    # 处理包设置
    if args.package:
        for package_name, absolute_str in args.package:
            use_absolute = absolute_str.lower() in ('true', 'yes', '1', 'on')
            converter.set_package_setting(package_name, use_absolute)
            if package_name == "*":
                print(f"设置所有包使用{'绝对' if use_absolute else '相对'}路径")
            else:
                print(f"设置包 '{package_name}' 使用{'绝对' if use_absolute else '相对'}路径")
    
    # 执行转换
    converter.convert_urdf()

if __name__ == "__main__":
    # 检查是否在ROS2环境中
    try:
        import ament_index_python
    except ImportError:
        print("警告: 未找到ament_index_python，将使用备用方法查找包路径")
        print("建议在ROS2环境中运行此脚本以获得最佳效果")
    
    main()