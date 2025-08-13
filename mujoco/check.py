# check_render.py
import mujoco
import mujoco.viewer
import time

xml = """
<mujoco>
  <worldbody>
    <light pos="0 0 1"/>
    <geom type="box" size=".1 .1 .1" rgba="1 0 0 1"/>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print("即将启动渲染器...")
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("渲染器已启动，将运行5秒...")
        start = time.time()
        while viewer.is_running() and time.time() - start < 5:
            viewer.sync()
        print("5秒结束。")
except Exception as e:
    print(f"启动渲染器时发生错误: {e}")

print("脚本执行完毕。")