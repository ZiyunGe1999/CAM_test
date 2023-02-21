import csv
import os
import sys
import shutil
# import matplotlib.pyplot as plt

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print('Usage: python3 timeframe_alignment.py <PATH/TO/CSV/FILE> <PATH/TO/IMAGE/FOLDER> <NEW/CSV/FILENAME> <NEW/IMAGE/FOLDER>')
        exit(0)
    path_to_csv_file = sys.argv[1]
    path_to_image_folder = sys.argv[2]
    csv_filename_new = sys.argv[3]
    image_folder_new = sys.argv[4]
    
    # timestamps = []
    # Fzs = []
    # pre_timestamp = 0
    touch_timestamp = 0
    found_touch = False
    with open(path_to_csv_file, newline='') as csvfile, open(csv_filename_new, 'w', newline='') as csvfile_new:
        reader = csv.DictReader(csvfile)
        writer  = csv.DictWriter(csvfile_new, fieldnames=reader.fieldnames)
        writer.writeheader()
        for row in reader:
            # print(row)
            timestamp = float(row['timestamp'])
            fz = float(row['Fz'])
            if found_touch:
                writer.writerow(row)
                # timestamps.append(timestamp)
                # Fzs.append(fz)
            elif not found_touch and fz < -10:
                touch_timestamp = timestamp
                print(f'found touched timestamp: {touch_timestamp}')
                found_touch = True
            # assert(float(row['timestamp']) > pre_timestamp)
            # timestamps.append(float(row['timestamp']))
            # Fzs.append(float(row['Fz']))
            # pre_timestamp = float(row['timestamp'])
    
    for filename in os.listdir(path_to_image_folder):
        timestamp = float(filename.split('.')[0])
        if timestamp >= touch_timestamp:
            shutil.copy(f'{path_to_image_folder}/{filename}', f'{image_folder_new}/{filename}')
    
    # plt.plot(timestamps, Fzs)
    # plt.savefig('test.jpg')
