import pandas as pd
import matplotlib.pyplot as plt

def plot_csv_scores(csv_file, host_id, target_id):
    df = pd.read_csv(csv_file)
    plt.figure(figsize=(10, 6))
    plt.plot(df['time_step'], df['trust_sample'], label='Trust Sample', linestyle='-', color='b')
    plt.plot(df['time_step'], df['gamma_cross'], label='Gamma Cross', linestyle='--', color='r')
    plt.plot(df['time_step'], df['gamma_local'], label='Gamma Local', linestyle='-.', color='g')
    plt.plot(df['time_step'], df['v_score'], label='V Score', linestyle=':', color='c')
    plt.plot(df['time_step'], df['d_score'], label='D Score', linestyle='-', color='m')
    plt.plot(df['time_step'], df['a_score'], label='A Score', linestyle='--', color='y')
    plt.plot(df['time_step'], df['final_score'], label='Final Score', linestyle='-', color='k', linewidth=1.5)
    plt.xlabel('Time Step')
    plt.ylabel('Value')
    plt.title(f'Car {host_id} -> Trust and Scores for Car {target_id}')
    plt.legend(loc='best')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(f"trust_plot_car_{host_id}_target_{target_id}.png", dpi=300)
    plt.close()

plot_csv_scores('/home/qcar2_scripts/trust_scores_car_1_target_0.csv', 1, 0)
plot_csv_scores('/home/qcar2_scripts/trust_scores_car_2_target_1.csv', 2, 1)