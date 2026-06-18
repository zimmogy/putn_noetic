import numpy as np
from lesta.core.datasets.pcd_dataset import PCDDataset
from utils.param import yaml

if __name__ == "__main__":
    # 1. 加载配置和数据集
    cfg = yaml.load("pylesta/configs/lesta.yaml")
    DATASET_CFG = cfg['DATASET']
    file_path = DATASET_CFG['training_data']

    print(f"Loading dataset from: {file_path}...")
    dataset = PCDDataset(file_path, DATASET_CFG)

    # 2. 定位 sparsity 特征的索引
    target_feature = 'sparsity'
    
    if target_feature not in dataset.feature_fields:
        print(f"Error: '{target_feature}' not found in feature fields!")
        print(f"Available fields: {dataset.feature_fields}")
        exit()
        
    sparsity_idx = dataset.feature_fields.index(target_feature)
    
    # 获取所有的 sparsity 数据 (形状: [N, ])
    sparsity_data = dataset.feature_vectors[:, sparsity_idx]
    total_samples = len(sparsity_data)

    # 3. 基础统计学分布
    print("\n" + "="*40)
    print(f"📊 稀疏度 (Sparsity) 全局数值分布 📊")
    print("="*40)
    print(f"总网格样本数: {total_samples}")
    print(f"Min (最小值): {np.min(sparsity_data):.6f}")
    print(f"Max (最大值): {np.max(sparsity_data):.6f}")
    print(f"Mean (均值):  {np.mean(sparsity_data):.6f}")
    print(f"Std (标准差): {np.std(sparsity_data):.6f}")
    
    print("\n--- 分位数 (Percentiles) ---")
    print(f"25% 分位数: {np.percentile(sparsity_data, 25):.6f}")
    print(f"50% 分位数 (中位数): {np.percentile(sparsity_data, 50):.6f}")
    print(f"75% 分位数: {np.percentile(sparsity_data, 75):.6f}")
    print(f"90% 分位数: {np.percentile(sparsity_data, 90):.6f}")
    print(f"99% 分位数: {np.percentile(sparsity_data, 99):.6f}")

    # 4. 针对 C++ 端 PUTN 逻辑的区间占比分析
    zero_count = np.sum(sparsity_data == 0.0)
    one_count = np.sum(sparsity_data == 1.0)
    in_between_count = total_samples - zero_count - one_count
    
    print("\n--- 物理语义区间占比 (基于 PUTN 规则) ---")
    print(f"✅ 安全/离散噪声 (Sparsity = 0.0): \t{zero_count} 样本 \t占比: {(zero_count/total_samples)*100:.2f}%")
    print(f"⚠️ 集中性暗坑 (0.0 < Sparsity < 1.0):\t{in_between_count} 样本 \t占比: {(in_between_count/total_samples)*100:.2f}%")
    print(f"❌ 绝对空洞/悬崖 (Sparsity = 1.0): \t{one_count} 样本 \t占比: {(one_count/total_samples)*100:.2f}%")

    # 5. 生成直方图 (可选，如果是在带界面的 Linux/Mac 下运行)
    try:
        import matplotlib.pyplot as plt
        plt.hist(sparsity_data, bins=50, color='blue', alpha=0.7, edgecolor='black')
        plt.title('Distribution of Sparsity in PCD Dataset')
        plt.xlabel('Sparsity Value')
        plt.ylabel('Frequency')
        plt.yscale('log') # 使用对数坐标轴，因为 0.0 的数量通常会呈压倒性优势
        plt.grid(axis='y', alpha=0.75)
        plt.savefig('sparsity_distribution.png')
        print("\n[!] 分布直方图已保存至当前目录下的 'sparsity_distribution.png'")
    except ImportError:
        print("\n[!] 提示: 安装 matplotlib 可生成可视化直方图 (pip install matplotlib)")