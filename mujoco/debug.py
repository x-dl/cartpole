# debug_mujoco.py

print("--- 开始 MuJoCo 环境诊断 ---")

try:
    import mujoco
    print("成功导入 'mujoco' 库。")
    
    # 1. 打印 mujoco 库的来源路径
    # 这是最关键的一步，它会告诉我们 Python 从哪个文件加载的 mujoco
    print(f"mujoco 库的文件路径: {mujoco.__file__}")
    
    # 2. 列出 mujoco 模块的所有可用属性
    # 我们要看看 'viewer' 是否在这个列表里
    print("\n'mujoco' 模块包含的属性:")
    attributes = dir(mujoco)
    print(attributes)
    
    if 'viewer' in attributes:
        print("\n诊断结果: 'viewer' 属性存在！理论上渲染应该工作。")
    else:
        print("\n诊断结果: 'viewer' 属性不存在！这是导致错误的核心原因。")
        
    # 3. 尝试显式导入 viewer 子模块
    print("\n尝试显式导入 'mujoco.viewer'...")
    try:
        import mujoco.viewer
        print("成功！可以显式导入 'mujoco.viewer'。")
    except ImportError as e:
        print(f"失败！无法显式导入 'mujoco.viewer'。错误信息: {e}")
        print("这证实了 mujoco 的 viewer 相关组件没有被正确安装或找到。")
        
except ImportError as e:
    print(f"导入 'mujoco' 失败！请检查是否已安装。错误: {e}")

print("\n--- 诊断结束 ---")