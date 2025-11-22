#!/usr/bin/env python3
"""
Visualization script for GA optimization results
Shows target function vs NOP output with error metrics
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def plot_results(csv_file='results.csv'):
    """
    Plot target function vs NOP output with error visualization
    """
    
    # Проверяем наличие файла
    if not Path(csv_file).exists():
        print(f"Error: {csv_file} not found!")
        print("Please run the C++ program first: ./simple_function")
        return
    
    # Читаем данные
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return
    
    print("Data loaded successfully!")
    print(f"Shape: {df.shape}")
    print(f"Columns: {list(df.columns)}")
    
    # Базовая статистика
    rmse = np.sqrt(np.mean(df['error']**2))
    mae = np.mean(np.abs(df['error']))
    max_error = np.max(np.abs(df['error']))
    
    print(f"\n=== METRICS ===")
    print(f"RMSE: {rmse:.6f}")
    print(f"MAE:  {mae:.6f}")
    print(f"Max Error: {max_error:.6f}")
    
    # Создаём фигуру с двумя подграфиками
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # ===== ГРАФИК 1: Функции =====
    ax1.plot(df['x'], df['target'], 'b-', linewidth=2.5, label='Target Function: sin(x) + 2.5*cos(x)', alpha=0.8)
    ax1.plot(df['x'], df['nop_output'], 'r--', linewidth=2.0, label='NOP Output', alpha=0.8)
    ax1.fill_between(df['x'], df['target'], df['nop_output'], alpha=0.2, color='gray', label='Difference')
    
    ax1.set_xlabel('x', fontsize=12, fontweight='bold')
    ax1.set_ylabel('y', fontsize=12, fontweight='bold')
    ax1.set_title('Target Function vs NOP Output', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='best', fontsize=11)
    
    # Добавляем текст с метриками на первый график
    metrics_text = f'RMSE: {rmse:.6f}\nMAE: {mae:.6f}\nMax Error: {max_error:.6f}'
    ax1.text(0.02, 0.98, metrics_text, transform=ax1.transAxes,
             fontsize=11, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # ===== ГРАФИК 2: Ошибка =====
    colors = ['red' if e > 0 else 'green' for e in df['error']]
    ax2.bar(df['x'], df['error'], color=colors, alpha=0.6, width=0.15, label='Error: NOP - Target')
    ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.8)
    
    ax2.set_xlabel('x', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Error', fontsize=12, fontweight='bold')
    ax2.set_title('Prediction Error (Red: Overestimate, Green: Underestimate)', 
                  fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.legend(loc='best', fontsize=11)
    
    plt.tight_layout()
    
    # Сохраняем и показываем
    output_file = 'results_plot.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved to {output_file}")
    
    plt.show()

def plot_error_distribution(csv_file='results.csv'):
    """
    Additional plot showing error distribution
    """
    
    if not Path(csv_file).exists():
        return
    
    df = pd.read_csv(csv_file)
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    
    # Гистограмма ошибок
    ax1.hist(df['error'], bins=30, color='skyblue', edgecolor='black', alpha=0.7)
    ax1.set_xlabel('Error Value', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Frequency', fontsize=11, fontweight='bold')
    ax1.set_title('Error Distribution', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # Абсолютная ошибка по x
    ax2.plot(df['x'], np.abs(df['error']), 'o-', color='orange', markersize=3, linewidth=1)
    ax2.set_xlabel('x', fontsize=11, fontweight='bold')
    ax2.set_ylabel('|Error|', fontsize=11, fontweight='bold')
    ax2.set_title('Absolute Error vs x', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Кумулятивная ошибка
    cumulative_error = np.cumsum(np.abs(df['error']))
    ax3.plot(df['x'], cumulative_error, linewidth=2, color='purple')
    ax3.fill_between(df['x'], 0, cumulative_error, alpha=0.3, color='purple')
    ax3.set_xlabel('x', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Cumulative |Error|', fontsize=11, fontweight='bold')
    ax3.set_title('Cumulative Error', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    
    # Box plot
    ax4.boxplot([df['target'], df['nop_output']], labels=['Target', 'NOP Output'])
    ax4.set_ylabel('Values', fontsize=11, fontweight='bold')
    ax4.set_title('Distribution Comparison', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    
    output_file = 'error_analysis.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Error analysis saved to {output_file}")
    
    plt.show()

if __name__ == '__main__':
    print("=== GA Optimization Results Visualization ===\n")
    
    # Основной график
    plot_results('results.csv')
    
    # Дополнительный анализ ошибок
    print("\n" + "="*50)
    print("Generating error analysis...")
    plot_error_distribution('results.csv')
