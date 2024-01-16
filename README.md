# RoboFusionLib WPIE机甲大师赛通用库

## 架构说明

### 库配置`cfg`

### 应用层`application`（开发中）

RM常用功能模块，比如：底盘（行为控制，功率控制）、云台（行为控制，发射控制）等。

### 通用算法层`algorithm`（开发中）

常用数据处理、数据结构、控制器等算法（纯软件、不与硬件相关），移植不会报错。

### 硬件抽象层`device`（开发中）

提供固定的接口供用户、应用层调用（用户只应调用该层提供的接口），本身是纯软件，但是是基于硬件驱动层的。

### 硬件驱动层`bsp`（开发中）

为硬件抽象层提供外设的物理实现，目前直接移植会报错。

### 芯片驱动层`driver`（开发中）

为硬件抽象层提供芯片的物理实现，目前直接移植会报错。

### 操作系统层（未来可期）

## 开发规范

### 通用

#### 文件名

文件名使用小写下划线法命名。

#### 文档末尾编译保护

文档的最末尾需是换行，即文档最后一行为空行。

#### 头文件重引用防护

```c
#ifndef _FILE_NAME_H__
#define _FILE_NAME_H__

//code

#endif /* _FILE_NAME_H__ */
```

#### 头文件调用

- 在需要的地方调用需要的头文件，不需要的地方不得调用不相关头文件。
- 调用的头文件有分类，各分类具有排序优先级，如下。
  - 标准库文件；
  - 该文件的头文件；
  - 底层库；
  - 操作系统层；
  - 芯片驱动层；
  - 硬件驱动层；
  - 硬件抽象层；
  - 通用算法层；
  - 应用层；
- 头文件调用时不同分类间需空行，各分类需按优先级排序。

#### 变量

1. 结构体&枚举类型标签命名，使用大驼峰法，例如`RflExample`；
2. 结构体&枚举数据类型命名，使用小写下划线法，结构体要加后缀`_s`，枚举要加后缀`_e`，例如`rfl_example_s`、`rfl_example_e`；

#### 函数

1. 命名：使用以rfl开头的小驼峰法，例如`rflExampleFunction`；
2. 声明：必须使用`extern`关键字作为前缀，例如：

```c
extern void rflExampleFunction(void);
```

## 使用说明

### 应用层`application`

#### 底盘模块`app_chassis`

### 通用算法层`algorithm`

### 硬件抽象层`device`

#### 电机`dev_motor`

### 硬件驱动层`bsp`

### 芯片驱动层`driver`

#### CAN总线`drv_can`
