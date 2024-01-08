# RVSim: RISCV 模拟器

## 简介

RVSim 是一个从零开始搭建的 RISCV 模拟器，完全用 C 语言编写。本项目旨在提供一个轻量级、高效的 RISCV 指令集模拟器，适合学习和教学目的。

项目 2023/11/26正式开始的。

## 特点

- **全C语言实现**: 直接使用 C 语言编写，易于理解和修改。
- **轻量级设计**: 精简且高效的代码，适合学习和实验。
- **开放源代码**: 项目代码完全开源，欢迎社区贡献。

## 如何使用

目前只能在Windows上用Visual Studio编译，Linux 和 MacOS的支持很快会来。

1. 克隆仓库：

    Visual Studio：克隆仓库 https://github.com/telos27/rvsim
    
    命令行：git clone https://github.com/telos27/rvsim

2. 编译项目：

    Visual Studio：生成解决方案

4. 运行模拟器：

    Visual Studio: 加上命令行参数：tests\linux32-nommu.bin tests\64mb.dtb
                    然后调试
   
    命令行：.\rvsim ..\..\tests\linux32-nommu.bin ..\..\tests\64mb.dtb    

## 文件说明

| 文件类型     | 文件名            |
| ------------ | ----------------- |
| 主文件       | rv-emulator.c，mmu.c, clint.c |
| Linux 内核   | tests\linux32-nommu.bin |
| DTB 文件     | tests\64mb.dtb    |
| 汇编文件     | t1.asm            |
| 二进制编码   | asm.o             |
| ppt文件      | RISCV模拟器.pptx |



## 贡献

欢迎任何形式的贡献，包括但不限于bug修复、功能增强、文档改善等。请提交 Pull Request 或者 Issue。

## 支持

如果您喜欢这个项目，请给我们一个 Star ⭐️，也欢迎 Follow 项目获取最新更新。

## 许可

本项目采用 [GNU v3 许可证](LICENSE.txt)。

---

感谢您对 RVSim 项目的关注！
