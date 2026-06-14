# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 当前状态

此仓库为空，仅初始化了 git。尚无源代码。

## 项目方向

根据环境推测，这是一个 Rust 项目（全局配置了 `rtk cargo` 工具链）。`cargo init` 后需要熟悉的命令：

- **构建:** `rtk cargo build`
- **检查:** `rtk cargo check`
- **测试:** `rtk cargo test`
- **运行:** `rtk cargo run`
- **Lint:** `rtk cargo clippy`
- **格式化:** `rtk cargo fmt`

## 提交前检查

- [ ] 无硬编码密钥
- [ ] 所有错误被显式处理
- [ ] 不突变数据 —— 始终返回新对象
- [ ] 函数 <50 行，文件 <800 行，嵌套 ≤4 层
- [ ] 测试通过，覆盖率 ≥80%
