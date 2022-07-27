import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import matplotlib.patches as mpatches

# Load CSV data
csv_arr = ['2022-06-06-14-18-57.csv',	'2022-06-06-14-23-54.csv',	'2022-06-06-14-30-52.csv', '2022-06-06-14-21-09.csv', '2022-06-06-14-25-20.csv', '2022-06-06-14-22-28.csv',	'2022-06-06-14-27-26.csv']

# Plotting of the coordinates of the positioning data
for elem in csv_arr:
    df = pd.read_csv(elem)  

    cm = plt.cm.get_cmap('RdYlBu')
    factor = 1.3
    figure_name = plt.figure(figsize=(5*factor, 8*factor))
    sc = plt.scatter(df[' position_x'], df[' position_y'], c=df['timestamp'], vmin=df['timestamp'].min(), vmax=df['timestamp'].max(), s=35, cmap=cm)
    plt.xlim([0,4530])
    plt.ylim([0,7530])
    left, bottom, width, height = (1250, 2500, 2000, 2000)
    rect=mpatches.Rectangle((left,bottom),width,height, 
                            fill=False,
                            color="purple",
                        linewidth=2)
    plt.gca().add_patch(rect)
    plt.grid(b=True, which='major', color='#666666', linestyle='-')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    plt.xlabel("X-Coordinates (mm)")
    plt.ylabel("Y-Coordinates (mm)")
    # plt.show()
    filename = elem[:-4] + ".png"
    plt.savefig(filename, dpi=400)

################################################################################################################################

df_plot = pd.read_csv("error_09_y.csv")

# Plotting of the error values in X
cm = plt.cm.get_cmap('RdYlBu')
factor = 2
figure_name = plt.figure(figsize=(6*factor, 4*factor))
sc = plt.scatter(df_plot['timestamp'], df_plot['difference_y'], c=df_plot['timestamp'], vmin=df_plot['timestamp'].min(), vmax=df_plot['timestamp'].max(), cmap=cm, zorder=2)
ln = plt.plot(df_plot['timestamp'], df_plot['difference_y'], linewidth=5, color="#D3D3D3", zorder=1)

plt.grid(b=True, which='major', color='#666666', linestyle='-')
plt.minorticks_on()
plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
plt.ylabel("Error (mm)")
plt.xlabel("Timestamp (ms)")
plt.title("DWM1001 Tag Y-coordinate Error Measurement")
# plt.show()
plt.savefig("error_09_y.png",dpi=300)

# Plotting of the error values in Y
cm = plt.cm.get_cmap('RdYlBu')
factor = 2
figure_name = plt.figure(figsize=(6*factor, 4*factor))
sc = plt.scatter(df_plot['timestamp'], df_plot['difference_x'], c=df_plot['timestamp'], vmin=df_plot['timestamp'].min(), vmax=df_plot['timestamp'].max(), cmap=cm, zorder=2)
ln = plt.plot(df_plot['timestamp'], df_plot['difference_x'], linewidth=5, color="#D3D3D3", zorder=1)

plt.grid(b=True, which='major', color='#666666', linestyle='-')
plt.minorticks_on()
plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
plt.ylabel("Error (mm)")
plt.xlabel("Timestamp (ms)")
plt.title("DWM1001 Tag X-coordinate Error Measurement")
# plt.show()
plt.savefig("error_57_x.png",dpi=300)