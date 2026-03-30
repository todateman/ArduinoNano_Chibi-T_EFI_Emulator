#!/usr/bin/env python3
"""
RPM スイープテスト - オシロスコープ測定用
1000～6000 rpm を順番に設定し、各周波数を測定する準備用スクリプト
"""

import serial
import time

# 期待される周波数（360 PPR）
EXPECTED_FREQS = {
    1000: {"A": 6000, "Z": 16.67, "G": 8.33},
    2000: {"A": 12000, "Z": 33.33, "G": 16.67},
    3000: {"A": 18000, "Z": 50, "G": 25},
    4000: {"A": 24000, "Z": 66.67, "G": 33.33},
    5000: {"A": 30000, "Z": 83.33, "G": 41.67},
    6000: {"A": 36000, "Z": 100, "G": 50},
}

def run_test():
    """RPMスイープテスト実行"""
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
    time.sleep(2)  # Arduinoブート待機
    
    print("="*70)
    print("RPM スイープテスト - オシロスコープ出力確認")
    print("="*70)
    print("\nオシロスコープ接続情報:")
    print("  CH1: D9 (Z相/インデックス)")
    print("  CH2: D10 (G相/ゲート信号)")
    print("  CH3: D8 (A相/メイン出力)")
    print("\n各RPMで以下の周波数が出力されます:")
    print("-"*70)
    
    for rpm in [1000, 2000, 3000, 4000, 5000, 6000]:
        # バッファクリア
        while ser.in_waiting:
            ser.read(256)
        time.sleep(0.3)
        
        # RPM設定
        cmd = f'RPM:{rpm}\n'.encode()
        ser.write(cmd)
        ser.flush()
        time.sleep(0.8)
        
        # 応答読み取り
        response = b''
        while ser.in_waiting:
            chunk = ser.read(256)
            if not chunk:
                break
            response += chunk
        
        # 結果処理
        lines = response.decode(errors='replace').strip().split('\n')
        ok_line = [l for l in lines if 'OK RPM' in l]
        status_line = [l for l in lines if 'KILL SW' in l]
        
        print(f"\nRPM: {rpm}")
        print("-"*70)
        
        if ok_line:
            print(f"  設定: ✓ {ok_line[-1].strip()}")
        
        if status_line:
            status = status_line[-1].strip()
            # ステータスから角度を抽出（デバッグ用）
            if 'Angle:' in status:
                angle_part = status.split('Angle:')[1].split(',')[0].strip()
                print(f"  動作: ✓ {status}")
        
        # 期待周波数を表示
        print(f"\n  期待周波数:")
        print(f"    CH3 (A相):  {EXPECTED_FREQS[rpm]['A']:>8.1f} Hz  (360 PPR, 基本周波数)")
        print(f"    CH1 (Z相):  {EXPECTED_FREQS[rpm]['Z']:>8.2f} Hz  (1 pulse/turn, インデックス)")
        print(f"    CH2 (G相):  {EXPECTED_FREQS[rpm]['G']:>8.2f} Hz  (1 pulse/2 rotations, ゲート)")
        
        time.sleep(1)
    
    print("\n" + "="*70)
    print("テスト完了。オシロスコープで各周波数が期待値と一致することを確認してください。")
    print("="*70)
    
    ser.close()

if __name__ == '__main__':
    try:
        run_test()
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
