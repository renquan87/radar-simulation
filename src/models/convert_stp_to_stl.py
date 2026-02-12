import sys
import FreeCAD
import Part
import Mesh

def convert_stp_to_stl(input_file, output_file):
    try:
        # 打开STP文件
        doc = FreeCAD.openDocument(input_file)
        
        # 导出为STL
        Mesh.export([obj for obj in doc.Objects if hasattr(obj, 'Shape')], output_file)
        
        # 关闭文档
        FreeCAD.closeDocument(doc.Name)
        
        print(f"成功转换: {input_file} -> {output_file}")
        return True
    except Exception as e:
        print(f"转换失败: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python convert_stp_to_stl.py <输入文件.stp> <输出文件.stl>")
        sys.exit(1)
    
    convert_stp_to_stl(sys.argv[1], sys.argv[2])
