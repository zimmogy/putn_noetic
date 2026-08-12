"""
Modified by: Haoran Wang
Revision date: 2026-08-12
"""

import numpy as np
from lesta.core.datasets.pcd_dataset import PCDDataset
from utils.param import yaml

if __name__ == "__main__":
    cfg = yaml.load("pylesta/configs/lesta.yaml")
    DATASET_CFG = cfg['DATASET']
    file_path = DATASET_CFG['training_data']

    print(f"Loading dataset from: {file_path}...")
    dataset = PCDDataset(file_path, DATASET_CFG)

    target_feature = 'sparsity'
    
    if target_feature not in dataset.feature_fields:
        print(f"Error: '{target_feature}' not found in feature fields!")
        print(f"Available fields: {dataset.feature_fields}")
        exit()
        
    sparsity_idx = dataset.feature_fields.index(target_feature)
    
    sparsity_data = dataset.feature_vectors[:, sparsity_idx]
    total_samples = len(sparsity_data)

    print("\n" + "="*40)
    print("Sparsity distribution")
    print("="*40)
    print(f"Total samples: {total_samples}")
    print(f"Min: {np.min(sparsity_data):.6f}")
    print(f"Max: {np.max(sparsity_data):.6f}")
    print(f"Mean: {np.mean(sparsity_data):.6f}")
    print(f"Std: {np.std(sparsity_data):.6f}")
    
    print("\n--- Percentiles ---")
    print(f"25%: {np.percentile(sparsity_data, 25):.6f}")
    print(f"50%: {np.percentile(sparsity_data, 50):.6f}")
    print(f"75%: {np.percentile(sparsity_data, 75):.6f}")
    print(f"90%: {np.percentile(sparsity_data, 90):.6f}")
    print(f"99%: {np.percentile(sparsity_data, 99):.6f}")

    zero_count = np.sum(sparsity_data == 0.0)
    one_count = np.sum(sparsity_data == 1.0)
    in_between_count = total_samples - zero_count - one_count
    
    print("\n--- Sparsity intervals ---")
    print(f"Sparsity = 0.0: {zero_count} samples, {(zero_count/total_samples)*100:.2f}%")
    print(f"0.0 < sparsity < 1.0: {in_between_count} samples, {(in_between_count/total_samples)*100:.2f}%")
    print(f"Sparsity = 1.0: {one_count} samples, {(one_count/total_samples)*100:.2f}%")

    try:
        import matplotlib.pyplot as plt
        plt.hist(sparsity_data, bins=50, color='blue', alpha=0.7, edgecolor='black')
        plt.title('Distribution of Sparsity in PCD Dataset')
        plt.xlabel('Sparsity Value')
        plt.ylabel('Frequency')
        plt.yscale('log')
        plt.grid(axis='y', alpha=0.75)
        plt.savefig('sparsity_distribution.png')
        print("\nSaved histogram to 'sparsity_distribution.png'")
    except ImportError:
        print("\nInstall matplotlib to generate a histogram: pip install matplotlib")
