# setup.py
from setuptools import setup, find_packages

setup(
    name='pendulum_control',
    version='1.0.0',
    author='[Your Name]', # 在这里写上你的名字
    author_email='[Your Email]', # 你的邮箱
    description='A library to control a first-order linear inverted pendulum.',
    long_description=open('README.md', encoding='utf-8').read(),
    long_description_content_type='text/markdown',
    url='[Optional: URL to your project, e.g., a GitHub repo]',
    packages=find_packages(),
    install_requires=[
        'pyserial>=3.5', # 你的代码依赖 pyserial
        # 如果有其他依赖，也在这里添加
    ],
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'License :: OSI Approved :: MIT License', # 建议选择一个开源许可证
        'Operating System :: OS Independent',
        'Topic :: Scientific/Engineering',
    ],
    python_requires='>=3.7',
)