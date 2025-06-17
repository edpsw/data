import pandas as pd
import numpy as np

# 读取CSV文件（假设无表头）
df = pd.read_csv('./g1/dance1_subject2.csv', header=None)
print(df.shape)
print(df.head())

start_col = 7  
end_col = -1  
selected_rows = df.iloc[:,start_col:]  # 切片包含end_row
print(selected_rows.shape)
# 转换为NumPy数组并保存
np.save('./npy/g1/dance1_subject2.npy', selected_rows.to_numpy())