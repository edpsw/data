import re
import xml.etree.ElementTree as ET

def parse_doc(doc_text):
    """解析文档文本，提取关节的质量属性数据"""
    data_dict = {}
    blocks = doc_text.split('坐标系：')

    # print(len(blocks[0]))
    # print(len(blocks[1]))
    # print(blocks[0])
    
    for block in blocks[1:]:
        # 提取关节名称
        joint_name = block.split('\n')[0].strip()
        data = {'mass': None, 'xyz': [None, None, None], 'inertia': [None]*6}
        
        # 提取质量
        mass_match = re.search(r'质量 = ([\d\.]+)', block)
        # print("mass_match",mass_match)
        if mass_match:
            data['mass'] = float(mass_match.group(1))
        
        print("mass_match",data['mass'])
        
        # 提取重心坐标
        x_match = re.search(r'X\s*=\s*([\d\.\-]+)', block)
        y_match = re.search(r'Y\s*=\s*([\d\.\-]+)', block)
        z_match = re.search(r'Z\s*=\s*([\d\.\-]+)', block)
        if x_match and y_match and z_match:
            data['xyz'] = [
                float(x_match.group(1)),
                float(y_match.group(1)),
                float(z_match.group(1))
            ]
        else:
            # 尝试另一种格式匹配
            xyz_match = re.search(r'重心 : ( 米 )\s*\(米\)\s*([\d\.\-]+)\s*Y=([\d\.\-]+)\s*Z=([\d\.\-]+)', block)
            if xyz_match:
                data['xyz'] = [
                    float(xyz_match.group(1)),
                    float(xyz_match.group(2)),
                    float(xyz_match.group(3))
                ]
        inertia_section  = extract_inertia_tensor(block)
        data['inertia'] = inertia_section
        data_dict[joint_name] = data
        # print(f"Parsed: {joint_name} - Mass: {data['mass']}, XYZ: {data['xyz']}, Inertia: {data['inertia']}")
    
    return data_dict



def extract_inertia_tensor(text):
    """
    从文本中提取惯性张量（转动惯量）数据
    返回包含6个分量的列表 [ixx, ixy, ixz, iyy, iyz, izz]
    """
    # 模糊匹配模式1：考虑等号前后的空格和不同分隔符
    pattern1 = re.compile(
        r'Lxx\s*[=:]\s*([\d\.\-Ee]+)[\s\t]'   # Lxx
        r'.*?Lxy\s*[=:]\s*([\d\.\-Ee]+)[\s\t]' # Lxy
        r'.*?Lxz\s*[=:]\s*([\d\.\-Ee]+)[\s\t]' # Lxz
        r'.*?Lyy\s*[=:]\s*([\d\.\-Ee]+)[\s\t]' # Lyy
        r'.*?Lyz\s*[=:]\s*([\d\.\-Ee]+)[\s\t]' # Lyz
        r'.*?Lzz\s*[=:]\s*([\d\.\-Ee]+)'       # Lzz
    , re.DOTALL)
    
    # 模糊匹配模式2：考虑行分隔和不同命名方式
    pattern2 = re.compile(
        r'Lxx\s*[=:]\s*([\d\.\-Ee]+).*?'   # Lxx
        r'(?:Lxy|Lyx)\s*[=:]\s*([\d\.\-Ee]+).*?' # Lxy 或 Lyx（对称）
        r'(?:Lxz|Lzx)\s*[=:]\s*([\d\.\-Ee]+).*?' # Lxz 或 Lzx（对称）
        r'Lyy\s*[=:]\s*([\d\.\-Ee]+).*?'   # Lyy
        r'(?:Lyz|Lzy)\s*[=:]\s*([\d\.\-Ee]+).*?' # Lyz 或 Lzy（对称）
        r'Lzz\s*[=:]\s*([\d\.\-Ee]+)'      # Lzz
    , re.DOTALL | re.IGNORECASE)
    
    # 模糊匹配模式3：表格形式提取
    pattern3 = re.compile(
        r'\|.*?Lxx\s*=\s*([\d\.\-Ee]+).*?'  # 表格中的Lxx
        r'\|.*?Lxy\s*=\s*([\d\.\-Ee]+).*?' # 表格中的Lxy
        r'\|.*?Lxz\s*=\s*([\d\.\-Ee]+).*?' # 表格中的Lxz
        r'\|.*?Lyy\s*=\s*([\d\.\-Ee]+).*?' # 表格中的Lyy
        r'\|.*?Lyz\s*=\s*([\d\.\-Ee]+).*?' # 表格中的Lyz
        r'\|.*?Lzz\s*=\s*([\d\.\-Ee]+).*?' # 表格中的Lzz
    , re.DOTALL)
    
    # 尝试多种模糊匹配模式
    for pattern in [pattern1, pattern2, pattern3]:
        match = pattern.search(text)
        if match:
            try:
                # 提取并转换6个分量值
                return [float(val) for val in match.groups()]
            except (ValueError, TypeError):
                continue
    
    # 如果所有模式都失败，尝试更宽松的数值提取
    numbers = re.findall(r'[-]?\d+\.\d+[Ee]?[-+]?\d*', text)
    if len(numbers) >= 6:
        try:
            return [float(val) for val in numbers[:6]]
        except (ValueError, TypeError):
            pass
    
    # 未找到有效数据
    return None

def map_joint_to_link(joint_name):
    """将文档中的关节名称映射到URDF中的链接名称"""
    if joint_name == 'pelvis-joint':
        return 'pelvis'
    # elif 'ankle' in joint_name:
    #     # 特殊处理踝关节
    #     return joint_name.replace('-', '_').replace('_joint', '')
    else:
        return joint_name.replace('-', '_').replace('_joint', '_link')

def update_urdf(urdf_path, data_dict):
    """更新URDF文件中的质量属性"""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    for link in root.findall('link'):
        link_name = link.get('name')
        # print(f"Processing link: {link_name}")
        
        # 查找匹配的关节数据
        found = False
        for joint_name, data in data_dict.items():
            mapped_name = map_joint_to_link(joint_name)
            # print("mapped_name",mapped_name)
            if link_name == mapped_name:
                print(f"Matched: {joint_name} -> {mapped_name}")
                found = True
                inertial = link.find('inertial')
                if inertial is None:
                    continue
                
                # 更新质量
                mass = inertial.find('mass')
                if mass is not None and data['mass'] is not None:
                    mass.set('value', str(data['mass']))
                
                # 更新重心
                origin = inertial.find('origin')
                if origin is not None and all(x is not None for x in data['xyz']):
                    xyz = f"{data['xyz'][0]:.7f} {data['xyz'][1]:.7f} {data['xyz'][2]:.7f}"
                    origin.set('xyz', xyz)
                
                # 更新惯性张量
                inertia_elem = inertial.find('inertia')
                if inertia_elem is not None and all(x is not None for x in data['inertia']):
                    inertia_elem.set('ixx', f"{data['inertia'][0]:.7f}")
                    inertia_elem.set('ixy', f"{data['inertia'][1]:.7f}")
                    inertia_elem.set('ixz', f"{data['inertia'][2]:.7f}")
                    inertia_elem.set('iyy', f"{data['inertia'][3]:.7f}")
                    inertia_elem.set('iyz', f"{data['inertia'][4]:.7f}")
                    inertia_elem.set('izz', f"{data['inertia'][5]:.7f}")
        
        if not found:
            print(f"No match found for link: {link_name}")
    
    # 保存修改后的URDF
    save_path = urdf_path.replace('_raw.urdf', '.urdf')
    tree.write(save_path, encoding='utf-8', xml_declaration=True)
    print(f"Updated URDF saved to: {save_path}")
    return save_path

# 主程序
if __name__ == "__main__":
    # 读取文档内容
    doc_path = '/home/z/code/unitree_rl_gym/resources/robots/q2/mass.txt'
    with open(doc_path, 'r', encoding='utf-8') as f:
        doc_text = f.read()
    # print(doc_text)
    
    # 解析文档数据
    print("Starting document parsing...")
    data_dict = parse_doc(doc_text)
    print(f"Parsed {len(data_dict)} joints")
    # print(data_dict)
    
    # 更新URDF文件
    urdf_path = '/home/z/code/unitree_rl_gym/resources/robots/q1/q1_raw.urdf'
    print(f"Updating URDF: {urdf_path}")
    updated_urdf_path = update_urdf(urdf_path, data_dict)
    
    print("URDF文件更新完成！")