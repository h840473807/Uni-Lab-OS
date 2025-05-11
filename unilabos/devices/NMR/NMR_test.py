import os
from datetime import datetime

def generate_experiment_file(              # 必填参数：输出路径
    user: str = "admin",             # 默认值参数
    holder: int = 1,                 # 默认值参数
    name: str = "Samplename1",       # 默认值参数
    expno: int = 1,                  # 默认值参数
    solvent: str = "D2O",            # 默认值参数
    experiment: str = "PROTON16",    # 默认值参数
    title: str = "test_1H", 
    save_root: str = None,                              # 默认动态生成
):
    
    # 生成文件内容
    content = f"""USER {user}
HOLDER {holder}
NAME {name}
EXPNO {expno}
SOLVENT {solvent}
EXPERIMENT {experiment}
TITLE {title}
END
"""
    # 生成文件名（当前日期）
    filename = datetime.now().strftime("%Y%m%d%H%M") + ".txt"
    
    # 创建输出目录（如果不存在）
    os.makedirs(save_root, exist_ok=True)
    
    # 写入文件
    filepath = os.path.join(save_root, filename)
    with open(filepath, "w") as f:
        f.write(content)
    
    return filepath

# 使用示例

generate_experiment_file( 
    user="chem_user", 
    holder=1,
    name="MySample", 
    expno=2, 
    solvent="CDCl3",
    experiment = "PROTON16",
    save_root=r"D:\Uni-lab\NMR",
)

