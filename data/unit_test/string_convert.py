import numpy as np
import h5py


with h5py.File("TNR_database.h5", 'r') as reader:
    print(np.array(reader['/current_force/angle_unit']))
    print(np.array(reader['/current_force/speed_unit']))
    print(np.array(reader['/current_force/convention']))
    print(np.array(reader['/current_force/frame']))


f1 = h5py.File('TNR_database.h5', 'r+')

del f1['current_force/angle_unit']
dset = f1.create_dataset('current_force/angle_unit', data="DEG")

# del f1['current_force/speed_unit']
# dset = f1.create_dataset('current_force/speed_unit', data="KNOT")
#
# del f1['current_force/convention']
# dset = f1.create_dataset('current_force/convention', data="GOTO")
#
# del f1['current_force/frame']
# dset = f1.create_dataset('current_force/frame', data="NED")
#
# del f1['wind_force/angle_unit']
# dset = f1.create_dataset('wind_force/angle_unit', data="DEG")
#
# del f1['wind_force/speed_unit']
# dset = f1.create_dataset('wind_force/speed_unit', data="MS")
#
# del f1['wind_force/convention']
# dset = f1.create_dataset('wind_force/convention', data="GOTO")
#
# del f1['wind_force/frame']
# dset = f1.create_dataset('wind_force/frame', data="NED")
f1.close()


with h5py.File("TNR_database.h5", 'r') as reader:
    print(np.array(reader['/current_force/angle_unit']))
    print(np.array(reader['/current_force/speed_unit']))
    print(np.array(reader['/current_force/convention']))
    print(np.array(reader['/current_force/frame']))
