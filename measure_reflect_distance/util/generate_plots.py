#!/usr/bin/env python3
import os
import glob
import re
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def parse_filename(filename):
    """
    ファイル名から条件を抽出する
    例: result_0deg_0.7m.csv -> angle=0, dist=0.7
    """
    basename = os.path.basename(filename)
    # 正規表現で "result_(\d+)deg_(\d+\.?\d*)m.csv" のパターンを探す
    match = re.search(r'result_(\d+)deg_([\d\.]+)m\.csv', basename)
    if match:
        return int(match.group(1)), float(match.group(2))
    return None, None

def main():
    parser = argparse.ArgumentParser(description='CSVを集計してグラフ化するスクリプト')
    parser.add_argument('--input_dir', type=str, required=True, help='CSVファイルが入っているディレクトリ')
    parser.add_argument('--output_dir', type=str, default=None, help='グラフの保存先（省略時はinput_dirと同じ）')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_dir = args.output_dir if args.output_dir else input_dir
    
    # CSVファイルのリスト取得
    csv_files = glob.glob(os.path.join(input_dir, "result_*.csv"))
    if not csv_files:
        print(f"エラー: {input_dir} に 'result_*.csv' が見つかりません。")
        return

    summary_data = []

    print(f"{len(csv_files)} 個のファイルを処理します...")

    for f in csv_files:
        angle, dist = parse_filename(f)
        if angle is None:
            print(f"スキップ (命名規則外): {os.path.basename(f)}")
            continue
        
        # CSV読み込み
        try:
            df = pd.read_csv(f)
            if len(df) == 0:
                print(f"スキップ (データなし): {os.path.basename(f)}")
                continue

            # 集計 (平均と標準偏差)
            dist_err_mean = df['dist_error_mm'].mean()
            dist_err_std = df['dist_error_mm'].std()
            angle_err_mean = df['angle_error_deg'].mean()
            angle_err_std = df['angle_error_deg'].std()

            summary_data.append({
                'Angle': angle,
                'Distance': dist,
                'DistErr_Mean': dist_err_mean,
                'DistErr_Std': dist_err_std,
                'AngleErr_Mean': angle_err_mean,
                'AngleErr_Std': angle_err_std,
                'Count': len(df)
            })
            print(f"読み込み完了: {os.path.basename(f)} (N={len(df)})")
        
        except Exception as e:
            print(f"エラー読み込み失敗: {f}, {e}")

    if not summary_data:
        print("有効なデータがありませんでした。")
        return

    # DataFrame化してソート
    df_summary = pd.DataFrame(summary_data)
    df_summary = df_summary.sort_values(by=['Angle', 'Distance'])
    
    # 集計結果をCSV保存
    summary_csv = os.path.join(output_dir, 'summary_stats.csv')
    df_summary.to_csv(summary_csv, index=False)
    print(f"\n集計結果を保存しました: {summary_csv}")

    # --- グラフ作成設定 ---
    plt.rcParams['font.family'] = 'sans-serif' # 日本語フォントがあれば指定、なければデフォルト
    plt.rcParams['font.size'] = 12
    
    # ユニークな角度ごとにプロット
    angles = sorted(df_summary['Angle'].unique())
    colors = ['blue', 'red', 'green', 'orange', 'purple'] # 角度ごとの色

    # 1. 距離誤差グラフ
    plt.figure(figsize=(8, 6))
    for i, angle in enumerate(angles):
        subset = df_summary[df_summary['Angle'] == angle]
        color = colors[i % len(colors)]
        plt.errorbar(subset['Distance'], subset['DistErr_Mean'], 
                     yerr=subset['DistErr_Std'], fmt='-o', 
                     label=f'{angle} deg', color=color, capsize=5)

    plt.title('Accuracy of Mirror Plane Distance Estimation')
    plt.xlabel('Distance to Mirror [m]')
    plt.ylabel('Distance Error [mm]')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    # plt.ylim(0, 50) # 必要ならY軸の範囲を固定
    
    save_path_dist = os.path.join(output_dir, 'graph_dist_error.png')
    plt.savefig(save_path_dist, dpi=300)
    print(f"グラフ保存: {save_path_dist}")

    # 2. 角度誤差グラフ
    plt.figure(figsize=(8, 6))
    for i, angle in enumerate(angles):
        subset = df_summary[df_summary['Angle'] == angle]
        color = colors[i % len(colors)]
        plt.errorbar(subset['Distance'], subset['AngleErr_Mean'], 
                     yerr=subset['AngleErr_Std'], fmt='-s', 
                     label=f'{angle} deg', color=color, capsize=5)

    plt.title('Accuracy of Mirror Normal Estimation')
    plt.xlabel('Distance to Mirror [m]')
    plt.ylabel('Normal Angle Error [deg]')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    # plt.ylim(0, 10) # 必要ならY軸の範囲を固定

    save_path_angle = os.path.join(output_dir, 'graph_angle_error.png')
    plt.savefig(save_path_angle, dpi=300)
    print(f"グラフ保存: {save_path_angle}")

if __name__ == '__main__':
    main()