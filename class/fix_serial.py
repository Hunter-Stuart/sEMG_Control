import re

# 读取文件
with open(r'c:\Users\23600\Downloads\class\class\try.html', 'r', encoding='utf-8') as f:
    content = f.read()

# 定义要替换的旧代码块（从行1864到1879）
old_code = '''          // 将每行数据写入环形缓冲区
          for (const line of lines) {
            if (line && isSerialConnected) {
              writeToRingBuffer(line);
            }
          }

          // 批量处理缓冲区中的数据，避免一次处理过多导致UI阻塞
          let processedCount = 0;
          while (getRingBufferCount() > 0 && processedCount < MAX_BATCH_PROCESS) {
            const dataLine = readFromRingBuffer();
            if (dataLine) {
              processData(dataLine);
              processedCount++;
            }
          }'''

# 新代码块
new_code = '''          // 直接处理每行数据，processData函数内部会写入环形缓冲区
          for (const line of lines) {
            if (line && isSerialConnected) {
              processData(line);
            }
          }'''

# 执行替换
if old_code in content:
    content = content.replace(old_code, new_code)
    print("成功找到并替换代码块")
else:
    print("未找到匹配的代码块，尝试查找类似内容...")
    # 尝试查找包含关键部分的代码
    if "writeToRingBuffer(line)" in content and "getRingBufferCount()" in content:
        print("找到相关代码，但格式可能不完全匹配")
    else:
        print("完全未找到相关代码")

# 写回文件
with open(r'c:\Users\23600\Downloads\class\class\try.html', 'w', encoding='utf-8') as f:
    f.write(content)

print("文件修改完成")
