import os
import shutil

def rename_and_copy_images(source_folder, destination_folder):
    # 创建目标文件夹如果不存在
    if not os.path.exists(destination_folder):
        os.makedirs(destination_folder)
    
    # 获取源文件夹中的所有文件
    files = os.listdir(source_folder)
    
    # 过滤出图片文件
    image_files = [f for f in files if f.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp'))]
    
    # 对图片进行排序（可选）
    image_files.sort()
    
    # 重命名并复制图片到目标文件夹
    for i, filename in enumerate(image_files):
        new_filename = f"img{i:03d}.jpg"  # 格式化新文件名，例如：img001.jpg
        source_file_path = os.path.join(source_folder, filename)
        destination_file_path = os.path.join(destination_folder, new_filename)
        shutil.copy(source_file_path, destination_file_path)
        print(f"Copied and renamed {filename} to {new_filename}")

# 使用示例
source_folder = 'home/sunrise/scservo_ws/collect_images'
destination_folder = 'home/sunrise/scservo_ws/collect_images/renamed'
rename_and_copy_images(source_folder, destination_folder)
