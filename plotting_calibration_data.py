#%%
import numpy as np
with open('calibration_data.txt', 'r') as f:
    data = [eval(i) for i in f.readlines()]

grid = np.ones((len(list(range(-70, 75, 5))),
                len(list(range(-40, 40, 5))),
                2))

new_data = dict()
for i in data: 
    new_data[i['angles']] = i['pixels']
    grid[*(np.array(i['angles'])/5).astype(int)] = np.array(i['pixels']) if i['pixels'] is not None else np.array([np.nan, np.nan])

# %%
