import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

def magic_formula(S, C, D, BCD, E):
    """Pacejka魔法公式核心计算"""
    B = BCD / (C * D)
    return D * np.sin(C * np.arctan(B * S - E * (B * S - np.arctan(B * S))))

def load_and_fit(fx_path, fy_path):
    # 读取并存储原始数据
    fx_df = pd.read_csv(fx_path, header=0)
    fz_fx = fx_df.columns[1:6].values.astype(float)
    slip_ratios = fx_df.iloc[:, 0].values 
    fx_data = {fz: fx_df.iloc[:, i+1].values for i, fz in enumerate(fz_fx)}

    fy_df = pd.read_csv(fy_path, header=0)
    fz_fy = fy_df.columns[1:6].values.astype(float)
    slip_angles = fy_df.iloc[:, 0].values*3.1415/180
    fy_data = {fz: fy_df.iloc[:, i+1].values for i, fz in enumerate(fz_fy)}

    parameters = {}

    # 拟合Fx参数
    for i, fz in enumerate(fz_fx):
        fx_vals = fx_data[fz]
        valid = ~np.isnan(fx_vals)
        S = slip_ratios[valid]
        Fx = fx_vals[valid]/fz

        try:
            D_init = np.max(np.abs(Fx)) * np.sign(Fx[np.argmax(np.abs(Fx))])
            slope = np.polyfit(S[:5], Fx[:5], 1)[0] if len(S)>=5 else 1.0
            p0 = [1.65, D_init, slope, 0]
            
            popt, _ = curve_fit(magic_formula, S, Fx, p0=p0, maxfev=10000)
            parameters[fz] = {
                'C_k': popt[0],
                'D_k': popt[1],
                'BCD_k': popt[2],
                'E_k': popt[3]
            }
        except Exception as e:
            print(f"Fx拟合失败 @ Fz={fz}: {str(e)}")

    # 拟合Fy参数
    for i, fz in enumerate(fz_fy):
        fy_vals = fy_data[fz]
        valid = ~np.isnan(fy_vals)
        alpha = slip_angles[valid]
        Fy = fy_vals[valid]/fz

        try:
            D_init = np.max(np.abs(Fy)) * np.sign(Fy[np.argmax(np.abs(Fy))])
            slope = np.polyfit(alpha[:5], Fy[:5], 1)[0] if len(alpha)>=5 else 1.0
            p0 = [1.3, D_init, slope, 0]
            
            popt, _ = curve_fit(magic_formula, alpha, Fy, p0=p0, maxfev=10000)
            params = parameters.get(fz, {})
            params.update({
                'C_alpha': popt[0],
                'D_alpha': popt[1],
                'BCD_alpha': popt[2],
                'E_alpha': popt[3]
            })
            parameters[fz] = params
        except Exception as e:
            print(f"Fy拟合失败 @ Fz={fz}: {str(e)}")

    return parameters, (slip_ratios, fx_data, slip_angles, fy_data)

def plot_comparison(parameters, raw_data):
    slip_ratios, fx_data, slip_angles, fy_data = raw_data
    
    # 绘制纵向力对比
    plt.figure(figsize=(15, 10))
    for idx, fz in enumerate(sorted(parameters.keys()), 1):
        params = parameters.get(fz, {})
        if 'C_k' not in params:
            continue
            
        valid = ~np.isnan(fx_data[fz])
        S = slip_ratios[valid]
        Fx_actual = fx_data[fz][valid]
        
        C = params['C_k']
        D = params['D_k']
        BCD = params['BCD_k']
        E = params['E_k']
        Fx_fit = magic_formula(S, C, D, BCD, E)*fz
        
        plt.subplot(2, 3, idx)
        plt.plot(S*100, Fx_actual, 'o', markersize=4, label='Real')
        plt.plot(S*100, Fx_fit, 'r-', linewidth=2, label='Fit')
        plt.xlabel('Slip Ratio (%)')
        plt.ylabel('Longitudinal Force (N)')
        plt.title(f'Fz = {fz} N')
        plt.legend()
    plt.tight_layout()
    plt.suptitle('Longitudinal Force Comparison', y=1.02)
    plt.show()

    # 绘制侧向力对比
    plt.figure(figsize=(15, 10))
    for idx, fz in enumerate(sorted(parameters.keys()), 1):
        params = parameters.get(fz, {})
        if 'C_alpha' not in params:
            continue
            
        valid = ~np.isnan(fy_data[fz])
        alpha = slip_angles[valid]
        Fy_actual = fy_data[fz][valid]
        
        C = params['C_alpha']
        D = params['D_alpha']
        BCD = params['BCD_alpha']
        E = params['E_alpha']
        Fy_fit = magic_formula(alpha, C, D, BCD, E)*fz
        
        plt.subplot(2, 3, idx)
        plt.plot(alpha, Fy_actual, 'o', markersize=4, label='Real')
        plt.plot(alpha, Fy_fit, 'r-', linewidth=2, label='Fit')
        plt.xlabel('Slip Angle (deg)')
        plt.ylabel('Lateral Force (N)')
        plt.title(f'Fz = {fz} N')
        plt.legend()
    plt.tight_layout()
    plt.suptitle('Lateral Force Comparison', y=1.02)
    plt.show()

if __name__ == "__main__":
    fx_file = "./Tire_Fx_slip.csv"
    fy_file = "./Tire_Fy_alpha.csv"
    
    params, raw_data = load_and_fit(fx_file, fy_file)
    
    # 打印所有参数
    print("\n所有载荷拟合参数:")
    for fz in sorted(params.keys()):
        print(f"\n垂向载荷 {fz} N:")
        for key, value in params[fz].items():
            print(f"{key}: {value:.4f}")
    
    # 绘制对比图
    plot_comparison(params, raw_data)
