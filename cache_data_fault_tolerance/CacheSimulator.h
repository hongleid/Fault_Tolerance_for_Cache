#ifndef CACHE_SIMULATOR
#define CACHE_SIMULATOR

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
using namespace std;

// 将某一位数取反
# define reversebit(x,y)  x^=(1<<y)


typedef unsigned char _u8;
typedef unsigned long long _u64;

/* 定义Cache标志位 */
const unsigned char CACHE_FLAG_VALID = 0x01;    // 有效位，标记Cache line是否保存着有效数据
const unsigned char CACHE_FLAG_DIRTY = 0x02;    // 脏位，标记要回写到内存中的Cache line
const unsigned char CACHE_FLAG_MASK = 0xFF;     // 在写入Cache line时进行初始化flag用到

/* 设置多级Cache，此处为最多多少级Cache */
const int MAX_LEVEL = 3;

/* 设置read/write标识 */
const char OPERATION_READ = 'l';
const char OPERATION_WRITE = 's';

/* 替换算法 */
enum cache_swap_style {
    CACHE_SWAP_FIFO,    // 先进先出替换
    CACHE_SWAP_LRU,     // 最近最久未使用替换
    CACHE_SWAP_RAND,    // 随机替换
    CACHE_SWAP_MAX      // 不是某种替换算法，表示替换方法的数量
};

/* 创建Cache line类 */
class Cache_Line {
public:
    _u64 tag;
    // 计数，FIFO里记录最一开始的访问时间，LRU里记录上一次访问的时间
    // count用来记录访问的时间，采用union数据结构，后面在更新count的时候
    // 只需要针对不同的替换算法，进行特殊赋值即可，而且不会导致一直使用count出现的思路混乱
    union {
        _u64 count;
        _u64 lru_count;
        _u64 fifo_count;
    };
    _u8 flag;           // 标记位存储，有效位、脏位
    _u64 buf;           // 存储在内存里的数据，即一个Cache行的数据，使用时分配空间
    _u64* addr;         // 存储在下一级存储器（即内存）中数据的地址，脏行时不能找到数据，干净行时说明内存中存在有Cache中的数据
                        // 需要在这块空间添加校验位，这块空间的大小取决于parity check和SEC-DED，没有添加到Cache类里去
    _u8 *extra_cache;   // 校验位存储位置（Cache空间）
    _u8 *extra_mem;     // 校验位存储位置（内存空间）
    // 假设每字32位，即4个字节
    // 一个Cache行如果32字节，那么一个Cache行就有8个子字块
    // 奇偶校验针对每个字，SEC-DED针对Cache行
};

/* 创建Cache_Sim类 */
class CacheSim : public MainWindow{
public:
    _u64 cache_size[MAX_LEVEL];          // Cache的总大小，单位byte
    _u64 cache_line_size[MAX_LEVEL];     // 一个Cache line的大小，单位byte
    _u64 cache_line_num[MAX_LEVEL];      // Cache一共有多少个line
    _u64 cache_word_size[MAX_LEVEL];     // Cache字大小
    _u64 cache_word_num[MAX_LEVEL];      // 一个Cache行有几个字
    _u64 cache_mapping_ways[MAX_LEVEL];  // 几路组相联
    _u64 cache_set_size[MAX_LEVEL];      // 整个Cache有多少组
    _u64 cache_set_shifts[MAX_LEVEL];    // 在内存地址划分中占的位数，log2(cache_set_size)
    _u64 cache_line_shifts[MAX_LEVEL];   // 在内存地址划分中占的位数，log2(cache_line_num)
    Cache_Line *caches[MAX_LEVEL];       // 真正的Cache地址列，指针数组
    _u64 *cache_buf[MAX_LEVEL];          // cache缓冲区，用来存储Cache行数据
    _u64 tick_count;                     // 指令计数器
    int write_style[MAX_LEVEL];          // 写操作
    int swap_style[MAX_LEVEL];           // 缓存替换算法
    _u64 cache_r_count[MAX_LEVEL], cache_w_count[MAX_LEVEL];          // 读写Cache的计数
    _u64 cache_w_memory_count;           // 实际写内存的计数，Cache—>Memory
    _u64 cache_r_memory_count;           // 实际读内存的计数，Memory—>L2
    _u64 cache_hit_count[MAX_LEVEL], cache_miss_count[MAX_LEVEL];     // Cache hit和miss的计数
    _u64 cache_free_num[MAX_LEVEL];      // 空闲Cache line的index记录，在寻找时，返回空闲line的index
    _u64 clean_count;                    // 干净行数目
    _u64 dirty_count;                    // 脏行数目（干净行和脏行针对L2 Cache）
    // L1 : L2 : Mem = 1 : 10 : 100，存取周期，单位时钟周期
    _u64 period;
    _u64 time_period;
    _u64 emergency_period;

    _u64 dirty_interval_time;

    _u64 clean_line_error_count;
    _u64 dirty_line_error_count;
    _u64 dirty_line_error_correct_count;

    _u64 all_dirty_error;

    _u64 m_data;                        // 全局变量，从该值的地址取值不至于出问题，从局部变量取值有问题

    bool START_MLREPS_FLAG;

    // add para
    int PARITY_CHECK;
    int SECDED;
    int EARLY_WRITE_BACK;
    int EMERGENCY_WRITE_BACK;
    int MLREPS;
    int ERROR_TYPE;
    int INJECT_TIMES;

    // 故障注入成功次数（Cache line无效则无法注入）
    _u64 inject_count;
    // 误纠正次数（错误纠正，包括hamming/sec-ded/mlreps）
    _u64 fail_correct;
    // 未检测到的错误次数
    _u64 error_but_not_detect_for_clean;
    _u64 error_but_not_detect_for_dirty;

    CacheSim();
    // ~CacheSim();

    /* 初始化 */
    void init(_u64 a_cache_size[],_u64 a_cache_line_size[], _u64 a_mapping_ways[], int l1_replace,
              int l2_replace, int a_parity, int a_secded, int a_mlreps, int a_early, int a_emergency,
              int a_error_type, int a_inject_times, Ui::MainWindow *ui);

    /* 检查是否命中 */
    int check_cache_hit(_u64 set_base, _u64 addr, int level, Ui::MainWindow *ui);

    /* 获取Cache当前set中空余的line */
    int get_cache_free_line(_u64 set_base, int level, int style, Ui::MainWindow *ui);

    /* 找到合适的line之后，将数据写入L1 Cache line中 */
    void cpu_mem_write_to_l1(_u64 index, _u64 addr, Ui::MainWindow *ui);

    /* 找到合适的line之后，将数据写入L2 Cache line中，这里需要添加编码操作 */
    void mem_write_to_l2(_u64 index, _u64 addr, Ui::MainWindow *ui);    // 干净数据编码
    void l1_cpu_write_to_l2(_u64 index, _u64 addr, Ui::MainWindow *ui); // 脏数据编码

    /* 将数据从L2写回内存，脏数据被替换，这里需要添加解码操作 */
    void l2_write_to_mem(_u64 l2_index, Ui::MainWindow *ui);

    /* 从l2读取数据到cpu，这里需要添加解码操作 */
    void l2_write_to_cpu(_u64 l2_index, Ui::MainWindow *ui);
    
    /* 从l2读取数据到l1，需要添加解码操作，以及对l1的填充 */
    void l2_write_to_l1(_u64 l1_index, _u64 l2_index, _u64 addr, Ui::MainWindow *ui);

    /* 对一个指令进行分析 */
    void do_cache_op(_u64 addr, char oper_style, Ui::MainWindow *ui);

    /* 读入trace文件 */
    void load_trace(const char * filename, Ui::MainWindow *ui);

    void re_init(int bits);

    /* 添加干净行校验 */
    int parity_check(_u64 m);
    int* update_parity_check(_u64 m);

    void parity_check_encode_for_cache(_u64 l2_index, _u64 data);
    void parity_check_decode_for_cache(_u64 l2_index, _u64 data, Ui::MainWindow *ui);

    /* 添加脏行校验 */
    _u64 cal(_u64 sz);
    _u64 antiCal(_u64 sz);
    string hamming_encode(string s);
    int hamming_decode(string d, string &s);
    void hamming_encode_for_cache(_u64 index, _u64 data);
    void hamming_decode_for_cache(_u64 index, _u64 data, Ui::MainWindow *ui);

    /* 修改hamming为SEC-DED */
    string secded_encode(string s);
    int secded_decode(string d, string &s);
    void secded_encode_for_cache(_u64 index, _u64 data);
    void secded_decode_for_cache(_u64 index, _u64 data, Ui::MainWindow *ui);

    /* MLREPS编码 */
    string mlreps_encode(string s);
    int mlreps_decode(string _d, string& s, Ui::MainWindow *ui);
    void mlreps_encode_for_cache(_u64 index, _u64 data);
    void mlreps_decode_for_cache(_u64 index, _u64 data, Ui::MainWindow *ui);

    /* 脏行校验辅助函数 */
    string dec_to_bin(_u64 m);
    _u64 bin_to_dec(string bin);

    /* 早期回写 */
    void early_write_back(_u64 time, Ui::MainWindow *ui);

    /* 紧急回写，遇到脏数据发生错误时 */
    // 为了避免一段时间频繁的脏数据发生错误，需设置紧急回写的缓冲周期
    // 缓冲周期：即在一段时间内不再使用紧急回写机制，避免大量的写内存操作
    // 此时，应该将period重新设置为0
    void emergency_write_back(Ui::MainWindow *ui);

    /* 添加故障注入 */
    int* rand_0(int min, int max, int n);
    int* rand_1(int min, int max, int n);
    int* cache_error_inject(_u64 min, _u64 max, _u64 n, int error_type);
};

#endif
