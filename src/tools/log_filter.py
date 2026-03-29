import sys

def filter_log(input_file, output_file=None, keyword="PCTimeSync time"):
    """
    从日志文件中筛选包含特定关键词的行
    
    参数:
        input_file (str): 输入日志文件路径
        output_file (str, optional): 输出文件路径，默认为None(打印到控制台)
        keyword (str, optional): 要筛选的关键词，默认为"[df]:"
    """
    try:
        with open(input_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            
        filtered_lines = [line for line in lines if keyword in line]
        
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.writelines(filtered_lines)
            print(f"筛选结果已保存至: {output_file}")
        else:
            for line in filtered_lines:
                print(line.strip())
                
    except FileNotFoundError:
        print(f"错误: 文件 '{input_file}' 不存在")
    except Exception as e:
        print(f"发生未知错误: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("使用方法: python log_filter.py <input_log_file> [output_file]")
        sys.exit(1)
        
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    filter_log(input_file, output_file)    
