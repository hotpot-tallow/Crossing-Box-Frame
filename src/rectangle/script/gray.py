from PIL import Image
import matplotlib.pyplot as plt

# 1. 读图
color_img = Image.open('/home/ceadman111/图片/摄像头/1.jpg').convert('RGB')
gray_img  = color_img.convert('L')

# 2. 取灰度值和原彩色
gray_value = gray_img.getpixel((610, 460))
r, g, b = color_img.getpixel((610, 460))
print(f'灰度值(0,0): {gray_value}')
print(f'原彩色(0,0): R={r}, G={g}, B={b}')

# 3. 创建纯色图
solid_img = Image.new('RGB', color_img.size, (r, g, b))

# 4. 显示（无 GUI 也能用）
plt.imshow(solid_img)
plt.axis('off')     # 不显示坐标轴
plt.show()