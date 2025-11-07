# Git 代码状态保存使用指南

## 概述

使用 Git 来保存和恢复代码状态，替代 Cursor 的对话历史恢复功能。

## 基本工作流程

### 1. 保存当前代码状态（提交）

在完成一个重要功能或解决一个问题后，保存当前代码状态：

```bash
# 查看当前修改的文件
git status

# 添加所有修改的文件
git add .

# 或者只添加特定文件
git add src/robot_bringup_cpp/config/nav2_params.yaml

# 提交更改，使用描述性的提交信息
git commit -m "功能描述: 具体做了什么修改"
```

**提交信息建议格式：**
- `"修复导航参数: 调整yaw_goal_tolerance精度"`
- `"添加新功能: 实现IMU角度监控"`
- `"优化配置: 更新SLAM参数"`

### 2. 查看提交历史

```bash
# 查看简洁的提交历史
git log --oneline

# 查看详细的提交历史
git log

# 查看最近5次提交
git log --oneline -5
```

### 3. 恢复到之前的代码状态

#### 方法一：查看特定提交的代码（不修改当前代码）

```bash
# 查看某个提交的详细信息
git show <提交哈希>

# 查看某个提交中特定文件的内容
git show <提交哈希>:<文件路径>
```

#### 方法二：恢复到特定提交（会修改当前代码）

```bash
# 恢复到某个提交（保留工作区的修改）
git checkout <提交哈希>

# 恢复到某个提交（丢弃所有未提交的修改）
git checkout <提交哈希> -- .

# 创建新分支来测试旧版本（推荐）
git checkout -b test-old-version <提交哈希>
```

#### 方法三：完全重置到某个提交（危险操作）

```bash
# 警告：这会丢失所有未提交的修改！
git reset --hard <提交哈希>
```

### 4. 回到最新状态

```bash
# 回到最新的提交
git checkout master




# 或者
git checkout main
```

## 实用技巧

### 1. 在每次重要对话后提交

建议在每次与 AI 完成一个重要任务后提交：

```bash
git add .
git commit -m "AI对话: 描述完成的任务"
```

### 2. 使用标签标记重要节点

```bash
# 创建标签
git tag -a v1.0 -m "重要版本标记"

# 查看所有标签
git tag

# 恢复到标签版本
git checkout v1.0
```

### 3. 查看文件修改历史

```bash
# 查看某个文件的修改历史
git log --follow <文件路径>

# 查看某个文件的具体修改内容
git log -p <文件路径>
```

### 4. 比较不同版本

```bash
# 比较当前代码和某个提交
git diff <提交哈希>

# 比较两个提交之间的差异
git diff <提交哈希1> <提交哈希2>

# 比较某个文件在不同版本间的差异
git diff <提交哈希1> <提交哈希2> -- <文件路径>
```

## 快速参考命令

```bash
# 状态查看
git status                    # 查看当前状态
git log --oneline            # 查看提交历史

# 保存状态
git add .                    # 添加所有修改
git commit -m "描述"         # 提交更改

# 恢复状态
git checkout <提交哈希>       # 恢复到某个提交
git checkout master          # 回到最新版本

# 查看差异
git diff                     # 查看未提交的修改
git show <提交哈希>          # 查看某个提交的详情
```

## 注意事项

1. **定期提交**：建议在每次完成一个重要任务后提交
2. **提交信息要清晰**：使用描述性的提交信息，方便以后查找
3. **谨慎使用 reset --hard**：这会永久删除未提交的修改
4. **使用分支**：在测试旧版本时，建议创建新分支而不是直接切换

## 示例场景

### 场景1：想回到修改导航参数之前的状态

```bash
# 1. 查看提交历史，找到修改导航参数之前的提交
git log --oneline

# 2. 假设提交哈希是 abc1234，创建新分支来查看
git checkout -b review-nav-params abc1234

# 3. 查看文件内容
cat src/robot_bringup_cpp/config/nav2_params.yaml

# 4. 回到最新版本
git checkout master
git branch -d review-nav-params  # 删除临时分支
```

### 场景2：保存当前工作，但还没完成

```bash
# 使用临时提交保存当前状态
git add .
git commit -m "WIP: 进行中的工作"

# 继续工作...

# 完成后再提交
git add .
git commit -m "完成: 具体功能描述"
```

