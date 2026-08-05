#!/usr/bin/env python3
"""
HappyMac — 模型训练 + 验证 + 量化

三步流程：
  1. Random Forest 快速验证（特征有效性）
  2. MLP 训练（TensorFlow/Keras）
  3. INT8 量化 → C 头文件（可直接 #include 到 ESP32-C3）

用法：
  python train.py                    # 训练模型 A 和 B
  python train.py --model a          # 只训练模型 A
  python train.py --model b          # 只训练模型 B
  python train.py --rf-only          # 只跑 RF 验证（不训练 MLP）
  python train.py --export           # 导出 C 头文件

输出：
  models/model_a.tflite              # 模型 A 量化权重
  models/model_b.tflite
  models/model_a.h                   # C 头文件（可直接 #include）
  models/model_b.h
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from sklearn.preprocessing import StandardScaler

# ─── 配置 ───
from config import *


# ============================================================
#  加载数据
# ============================================================

def load_data(model: str):
    """加载预处理后的特征和标签"""
    X_path = MODEL_DIR / f"X_{model}.npy"
    y_path = MODEL_DIR / f"y_{model}.npy"
    if not X_path.exists() or not y_path.exists():
        print(f"❌ 找不到 {X_path} / {y_path}")
        print("   请先运行: python preprocess.py")
        sys.exit(1)

    X = np.load(X_path)
    y = np.load(y_path)
    return X, y


def split_data(X, y):
    """分层划分训练/验证/测试"""
    # 先分测试集
    X_temp, X_test, y_temp, y_test = train_test_split(
        X, y, test_size=TEST_SPLIT, stratify=y, random_state=42)

    # 从剩余中分验证集
    val_ratio = VAL_SPLIT / (1.0 - TEST_SPLIT)
    X_train, X_val, y_train, y_val = train_test_split(
        X_temp, y_temp, test_size=val_ratio, stratify=y_temp, random_state=42)

    return X_train, X_val, X_test, y_train, y_val, y_test


# ============================================================
#  Random Forest 快速验证
# ============================================================

def rf_validate(model: str):
    """用 Random Forest 快速验证特征有效性"""
    print(f"\n{'='*60}")
    print(f"  Random Forest 验证 — 模型 {model.upper()}")
    print(f"{'='*60}")

    X, y = load_data(model)
    X_train, X_val, X_test, y_train, y_val, y_test = split_data(X, y)

    # 标准化
    scaler = StandardScaler()
    X_train_s = scaler.fit_transform(X_train)
    X_test_s  = scaler.transform(X_test)

    rf = RandomForestClassifier(
        n_estimators=RF_N_ESTIMATORS,
        max_depth=RF_MAX_DEPTH,
        random_state=42, n_jobs=-1)
    rf.fit(X_train_s, y_train)

    # 测试集准确率
    train_acc = rf.score(X_train_s, y_train)
    test_acc  = rf.score(X_test_s, y_test)

    # 详细报告
    if model == "a":
        names = LABEL_A_CLASSES
    else:
        names = LABEL_B_CLASSES
    # 确保 names 长度匹配
    names = names[:len(np.unique(y))]

    y_pred = rf.predict(X_test_s)
    print(f"\n  训练准确率: {train_acc:.3f}")
    print(f"  测试准确率: {test_acc:.3f}")
    print(f"  过拟合度:   {train_acc - test_acc:.3f} {'⚠️  偏高' if train_acc - test_acc > 0.15 else '✅'}")

    print(f"\n  分类报告:")
    report = classification_report(y_test, y_pred, target_names=names,
                                    zero_division=0)
    print("  " + report.replace("\n", "\n  "))

    # 特征重要性
    if hasattr(X, 'shape') and X.shape[1] <= len(FEATURE_NAMES):
        importances = rf.feature_importances_
        top_n = min(10, len(importances))
        top_idx = np.argsort(importances)[::-1][:top_n]
        print(f"\n  Top {top_n} 特征:")
        feat_names = FEATURE_NAMES[:X.shape[1]]
        for i, idx in enumerate(top_idx):
            bar = "█" * int(importances[idx] * 50 / importances[top_idx[0]])
            print(f"  {i+1:2d}. {feat_names[idx]:20s} {importances[idx]:.4f} {bar}")

    # 混淆矩阵摘要
    cm = confusion_matrix(y_test, y_pred)
    print(f"\n  混淆矩阵（行=真，列=预测）：")
    header = "     " + "".join(f"{n[:4]:>5}" for n in names)
    print(f"  {header}")
    for i, row in enumerate(cm):
        line = "".join(f"{v:5d}" for v in row)
        print(f"  {names[i][:4]:>4}{line}")

    # 结论
    if test_acc > 0.85:
        print(f"\n  ✅ 特征有效！准确率 {test_acc:.1%} > 85%，可以蒸馏到 MLP")
        return True, scaler, rf
    elif test_acc > 0.70:
        print(f"\n  ⚠️  准确率 {test_acc:.1%}，特征勉强够用，建议增加数据后再蒸馏")
        return True, scaler, rf
    else:
        print(f"\n  ❌ 准确率 {test_acc:.1%} < 70%，特征不足以训练 MLP")
        print(f"     请检查: (1) 数据量是否够 (2) 标签分布是否均匀 (3) 雷达信号是否有明显模式")
        return False, None, None


# ============================================================
#  MLP 训练 + 量化
# ============================================================

def augment(X, y):
    """数据增强：加噪声 + 偏移"""
    X_aug = X.copy()
    noise = np.random.normal(0, AUGMENT_NOISE_STD * np.std(X, axis=0), X.shape)
    shift = np.random.uniform(-AUGMENT_SHIFT_MAX, AUGMENT_SHIFT_MAX, (X.shape[0], 1))
    X_aug += noise
    X_aug[:, 0] += shift[:, 0]  # x_mean 加随机偏移模拟位置变化
    return np.concatenate([X, X_aug]), np.concatenate([y, y])


def train_mlp(model: str, scaler):
    """训练 MLP 并量化"""
    import tensorflow as tf

    X, y = load_data(model)
    X_train, X_val, X_test, y_train, y_val, y_test = split_data(X, y)

    # 数据增强
    X_train_aug, y_train_aug = augment(X_train, y_train)

    # 标准化
    X_train_s = scaler.fit_transform(X_train_aug)
    X_val_s   = scaler.transform(X_val)
    X_test_s  = scaler.transform(X_test)

    n_features = X.shape[1]
    n_classes  = len(np.unique(y))

    print(f"\n{'='*60}")
    print(f"  MLP 训练 — 模型 {model.upper()}")
    print(f"{'='*60}")
    print(f"  特征数: {n_features}")
    print(f"  类别数: {n_classes}")
    print(f"  训练样本: {len(X_train_s)}（增强后）")
    print(f"  验证样本: {len(X_val_s)}")
    print(f"  测试样本: {len(X_test_s)}")

    # ── 构建 MLP ──
    tf_model = tf.keras.Sequential([
        tf.keras.layers.Dense(MLP_HIDDEN_1, activation='relu',
                               input_shape=(n_features,),
                               kernel_regularizer=tf.keras.regularizers.l2(0.001)),
        tf.keras.layers.Dropout(MLP_DROPOUT),
        tf.keras.layers.Dense(MLP_HIDDEN_2, activation='relu',
                               kernel_regularizer=tf.keras.regularizers.l2(0.001)),
        tf.keras.layers.Dense(n_classes, activation='softmax'),
    ])

    tf_model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE),
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy'])

    tf_model.summary()
    total_params = tf_model.count_params()
    print(f"  总参数量: {total_params} ({total_params * 4 / 1024:.1f} KB float32)")

    # ── 训练 ──
    early_stop = tf.keras.callbacks.EarlyStopping(
        monitor='val_loss', patience=20, restore_best_weights=True)
    reduce_lr = tf.keras.callbacks.ReduceLROnPlateau(
        monitor='val_loss', factor=0.5, patience=5, min_lr=1e-5)

    history = tf_model.fit(
        X_train_s, y_train_aug,
        validation_data=(X_val_s, y_val),
        epochs=TRAIN_EPOCHS,
        batch_size=BATCH_SIZE,
        callbacks=[early_stop, reduce_lr],
        verbose=1)

    # ── 测试 ──
    test_loss, test_acc = tf_model.evaluate(X_test_s, y_test, verbose=0)
    print(f"\n  测试准确率: {test_acc:.4f}")

    # ── 保存 float32 模型 ──
    keras_path = MODEL_DIR / f"model_{model}.h5"
    tf_model.save(keras_path)
    print(f"  已保存: {keras_path}")

    # ── INT8 量化 ──
    print(f"\n{'='*60}")
    print(f"  INT8 量化")
    print(f"{'='*60}")

    converter = tf.lite.TFLiteConverter.from_keras_model(tf_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    # 代表性数据集（用于 INT8 校准）
    def representative_dataset():
        n_samples = min(QUANT_SAMPLES, len(X_train_s))
        for i in range(n_samples):
            yield [X_train_s[i:i+1].astype(np.float32)]

    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type  = tf.int8
    converter.inference_output_type = tf.int8

    try:
        tflite_quant = converter.convert()
        quant_path = MODEL_DIR / f"model_{model}.tflite"
        with open(quant_path, "wb") as f:
            f.write(tflite_quant)
        quant_size = len(tflite_quant)
        print(f"  量化模型: {quant_path} ({quant_size} 字节)")

        # ── 验证量化精度 ──
        interpreter = tf.lite.Interpreter(model_content=tflite_quant)
        interpreter.allocate_tensors()
        input_details  = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # 量化参数
        input_scale, input_zero = input_details[0]['quantization']
        output_scale, output_zero = output_details[0]['quantization']

        correct = 0
        for i in range(min(200, len(X_test_s))):
            # 量化输入
            x_q = (X_test_s[i:i+1] / input_scale + input_zero).astype(np.int8)
            interpreter.set_tensor(input_details[0]['index'], x_q)
            interpreter.invoke()
            y_q = interpreter.get_tensor(output_details[0]['index'])
            # 反量化
            y_f = (y_q.astype(np.float32) - output_zero) * output_scale
            if np.argmax(y_f) == y_test[i]:
                correct += 1

        quant_acc = correct / min(200, len(X_test_s))
        print(f"  量化准确率: {quant_acc:.4f} (drop = {test_acc - quant_acc:.4f})")

        if test_acc - quant_acc > 0.05:
            print(f"  ⚠️  量化损失 >5%，建议用 QAT（量化感知训练）")
        else:
            print(f"  ✅ 量化损失可接受")

    except Exception as e:
        print(f"  ⚠️  INT8 量化失败: {e}")
        print(f"  降级到 float16 量化 ...")
        converter.target_spec.supported_types = [tf.float16]
        tflite_f16 = converter.convert()
        quant_path = MODEL_DIR / f"model_{model}.tflite"
        with open(quant_path, "wb") as f:
            f.write(tflite_f16)
        print(f"  float16 模型: {quant_path} ({len(tflite_f16)} 字节)")

    # ── 保存 scaler 参数（C3 上做标准化用）──
    scaler_params = {
        "mean": scaler.mean_.tolist(),
        "scale": scaler.scale_.tolist(),
    }
    scaler_path = MODEL_DIR / f"scaler_{model}.json"
    with open(scaler_path, "w") as f:
        json.dump(scaler_params, f, indent=2)
    print(f"  Scaler: {scaler_path}")

    return tflite_quant if 'tflite_quant' in dir() else None


# ============================================================
#  导出 C 头文件
# ============================================================

def export_header(model: str):
    """将 .tflite 转为 C 头文件，可直接 #include 到 ESP32"""
    tflite_path = MODEL_DIR / f"model_{model}.tflite"
    if not tflite_path.exists():
        print(f"❌ 找不到 {tflite_path}")
        return

    with open(tflite_path, "rb") as f:
        data = f.read()

    h_path = MODEL_DIR / f"model_{model}.h"
    with open(h_path, "w") as f:
        f.write(f"// HappyMac — Model {model.upper()}\n")
        f.write(f"// Auto-generated by train.py\n")
        f.write(f"// Size: {len(data)} bytes\n\n")
        f.write(f"#ifndef HAPPYMAC_MODEL_{model.upper()}_H\n")
        f.write(f"#define HAPPYMAC_MODEL_{model.upper()}_H\n\n")
        f.write(f"const unsigned char model_{model}[] = {{\n  ")
        for i, b in enumerate(data):
            f.write(f"0x{b:02x}, ")
            if (i + 1) % 12 == 0:
                f.write("\n  ")
        f.write(f"\n}};\n\n")
        f.write(f"const unsigned int model_{model}_len = {len(data)};\n\n")
        f.write(f"#endif  // HAPPYMAC_MODEL_{model.upper()}_H\n")

    print(f"  ✅ C 头文件: {h_path} ({len(data)} 字节)")
    print(f"     用法: #include \"model_{model}.h\"")
    print(f"     变量: model_{model}[]  ({len(data)} 字节)")
    print(f"     长度: model_{model}_len")


# ============================================================
#  入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="HappyMac 模型训练")
    parser.add_argument("--model", type=str, default="all",
                        choices=["a", "b", "all"],
                        help="训练哪个模型")
    parser.add_argument("--rf-only", action="store_true",
                        help="只跑 RF 验证，不训练 MLP")
    parser.add_argument("--export", action="store_true",
                        help="只导出 C 头文件（需先训练过）")
    args = parser.parse_args()

    models = ["a", "b"] if args.model == "all" else [args.model]

    for m in models:
        # 1. RF 验证
        ok, scaler, rf = rf_validate(m)
        if not ok:
            print(f"\n⚠️  模型 {m.upper()} RF 验证未通过，跳过 MLP 训练\n")
            continue

        if args.rf_only:
            continue

        # 2. MLP 训练 + 量化
        tflite = train_mlp(m, scaler)

        # 3. 导出 C 头文件
        export_header(m)

    print(f"\n{'='*60}")
    print(f"  完毕")
    print(f"{'='*60}")
    for m in models:
        h_path = MODEL_DIR / f"model_{m}.h"
        if h_path.exists():
            print(f"  → {h_path}")

    print(f"\n  下一步:")
    print(f"    1. 将 model_*.h 复制到 C3 项目目录")
    print(f"    2. 在 new_radar.ino 中 #include \"model_a.h\"")
    print(f"    3. 使用 TensorFlow Lite Micro 加载并推理")


if __name__ == "__main__":
    main()
