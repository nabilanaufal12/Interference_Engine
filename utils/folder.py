import os
import cv2
import pandas
from datetime import datetime

def create_folder_in_public(public_dir='public/captures'):
    # Jika direktori 'public' belum ada, buat terlebih dahulu
    if not os.path.exists(public_dir):
        os.makedirs(public_dir)

    # Ambil semua item yang ada di direktori 'public' dan filter hanya folder
    existing_folders = [name for name in os.listdir(public_dir) if os.path.isdir(os.path.join(public_dir, name))]
    
    # Tentukan nomor folder baru berdasarkan jumlah folder yang ada + 1
    new_folder_number = len(existing_folders) + 1
    new_folder_name = str(new_folder_number)
    
    # Buat folder baru dengan nama yang sudah ditentukan
    new_folder_path = os.path.join(public_dir, new_folder_name)
    os.makedirs(new_folder_path)

    return new_folder_path

# Fungsi untuk menyimpan gambar di folder yang sudah dibuat
def save_image_to_folder(y,image, folder_path, image_id, label):
    # Path lengkap untuk menyimpan gambar
    image_filename = f"{image_id}.jpg"
    image_path = os.path.join(folder_path, image_filename)

    # Simpan gambar dengan format JPG menggunakan OpenCV
    cv2.putText(image,label, (20,y), 0, .7, (255,255,255) , thickness=2, lineType=cv2.LINE_AA)
    cv2.imwrite(image_path, image)

    print(f"Gambar berhasil disimpan di: {image_path}")
    
def get_image_path(folder_id, image_id):
    folder_path = os.path.join( str(folder_id))
    image_path = os.path.join(folder_path, f"{image_id}.jpg")

    # Cek apakah file gambar ada
    if os.path.exists(image_path) and os.path.isfile(image_path):
        return image_path
    else:
        return None

datas = []
times = datetime.now().strftime('%H:%M:%S.%f')[:-3]
timess = datetime.now().strftime('%d-%m-%Y %H-%M-%S-%f')[:-3]

def save_to_excel(data):
    global timess
    df  = pandas.DataFrame(data, columns=["DAY", "DATE", "TIME", "HDG", "SOG", "COG", "Lat Dir", "Lat", "Lon Dir", "Lon"])
    df.index += 1
    df.index.name = "No"
    df.to_excel(timess+".xlsx", sheet_name="Sheet 1", index=True)
    print("Berhasil mengambil data")
    return


# datas.append([times,2,3,4,5,6,7,8,9,0])
# datas.append([times,2,3,4,5,6,7,8,9,0])
# datas.append([times,2,3,4,5,6,7,8,9,0])
# datas.append([times,2,3,4,5,6,7,8,9,0])

# save_to_excel(datas)