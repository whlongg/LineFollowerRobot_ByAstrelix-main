import os
from PIL import Image, ImageDraw, ImageFont
import random

# Đường dẫn tới thư mục chứa ảnh
BASE_DIR = os.path.join(os.path.dirname(__file__), 'Graph Image')

# Định nghĩa các màu khung (nếu hết sẽ random thêm)
FRAME_COLORS = [
    (255, 0, 0),    # Đỏ
    (0, 255, 0),    # Xanh lá
    (0, 0, 255),    # Xanh dương
    (255, 165, 0),  # Cam
    (128, 0, 128),  # Tím
    (0, 255, 255),  # Cyan
    (255, 192, 203) # Hồng
]

# Lấy danh sách ảnh trong các thư mục con
all_images = []  # [(path, subfolder)]
subfolders = []
for root, dirs, files in os.walk(BASE_DIR):
    if root == BASE_DIR:
        continue  # Bỏ qua thư mục gốc
    rel_folder = os.path.relpath(root, BASE_DIR)
    if rel_folder not in subfolders:
        subfolders.append(rel_folder)
    for f in files:
        if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            all_images.append((os.path.join(root, f), rel_folder))

if not all_images:
    print('Không tìm thấy ảnh trong thư mục Graph Image!')
    exit(1)

# Đặt kích thước chuẩn cho mỗi ảnh (resize về cùng kích thước)
IMG_WIDTH, IMG_HEIGHT = 300, 200

# Gom nhóm ảnh theo thư mục con
from collections import defaultdict
images_by_folder = defaultdict(list)
for img_path, folder in all_images:
    images_by_folder[folder].append(img_path)

# Số cột ghép ảnh (tự động tính gần bằng căn bậc 2 tổng số ảnh)
import math
total_images = len(all_images)
cols = math.ceil(math.sqrt(total_images))
rows = math.ceil(total_images / cols)

# Tạo ảnh nền lớn
canvas = Image.new('RGB', (IMG_WIDTH * cols, IMG_HEIGHT * rows), (255, 255, 255))
draw = ImageDraw.Draw(canvas)

# Tải font để ghi chú (nếu có)
try:
    font = ImageFont.truetype("arial.ttf", 20)
except:
    font = ImageFont.load_default()

# Ghép ảnh lên canvas
img_idx = 0
folder_color_map = {}
for i, folder in enumerate(images_by_folder):
    color = FRAME_COLORS[i] if i < len(FRAME_COLORS) else tuple(random.randint(0,255) for _ in range(3))
    folder_color_map[folder] = color
    for img_path in images_by_folder[folder]:
        row, col = divmod(img_idx, cols)
        with Image.open(img_path) as im:
            im = im.convert('RGB')
            im = im.resize((IMG_WIDTH, IMG_HEIGHT))
            canvas.paste(im, (col*IMG_WIDTH, row*IMG_HEIGHT))
            # Vẽ khung màu quanh ảnh
            x0, y0 = col*IMG_WIDTH, row*IMG_HEIGHT
            x1, y1 = x0+IMG_WIDTH-1, y0+IMG_HEIGHT-1
            draw.rectangle([x0, y0, x1, y1], outline=color, width=6)
            # Ghi tên thư mục con lên góc ảnh
            draw.text((x0+8, y0+8), folder, fill=color, font=font)
        img_idx += 1

# Lưu ảnh kết quả
output_path = os.path.join(BASE_DIR, 'merged_result.png')
canvas.save(output_path)
print(f'Đã ghép xong! Ảnh kết quả lưu tại: {output_path}')
