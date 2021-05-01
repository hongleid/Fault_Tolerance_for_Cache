#include "CacheSimulator.h"
#include <cstdlib>
#include <cstring>
#include <math.h>
#include <cstdio>
#include <time.h>
#include <climits>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <chrono>
using namespace std;

CacheSim::CacheSim() {}

/* 
 * @arg a_cache_size[] 多级Cache的大小设置
 * @arg a_cache_line_size[] 多级Cache的line size大小
 * @arg a_mapping_ways[] 组相连的链接方式
*/
void CacheSim::init(_u64 a_cache_size[3], _u64 a_cache_line_size[3], _u64 a_mapping_ways[3], int l1_replace,
                int l2_replace, int a_parity, int a_secded, int a_mlreps, int a_early, int a_emergency,
                int a_error_type, int a_inject_times, Ui::MainWindow *ui) {
    // 如果输入配置不符合要求，现在为2级Cache
    if (a_cache_line_size[0] < 0 || a_cache_line_size[1] < 0
       || a_mapping_ways[0] < 1 || a_mapping_ways[1] < 1) {
        ui->textBrowser->insertPlainText("输入配置不符合要求！\n");
        return;
    }

    //////////////////////////////////////////////////////////////////////////////////
    // line_size[] = {32, 32}, ways[] = {4, 4}, cache_size[] = {0x1000, 0x8000}
    // cache_size[0] = 0x1000 = 4kb, cache_size[1] = 0x8000 = 32kb
    // cache_line_size[0] = 32byte, cache_line_size[1] = 32byte
    // cache_line_num[0] = 128line, cache_line_num[1] = 1024line
    // cache_line_shifts[0] = 5, cache_line_shifts[1] = 5
    // cache_mapping_ways[0] = 4, cache_mapping_ways[1] = 4
    // cache_set_size[0] = 32, cache_set_size[1] = 256   有多少个组（set）
    // cache_set_shifts[0] = 5, cache_set_shifts[1] = 8
    // cache_free_num[0] = 128, cache_free_num[1] = 128
    //////////////////////////////////////////////////////////////////////////////////

    PARITY_CHECK = a_parity;
    SECDED = a_secded;
    MLREPS = a_mlreps;
    EARLY_WRITE_BACK = a_early;
    EMERGENCY_WRITE_BACK = a_emergency;
    ERROR_TYPE = a_error_type;
    INJECT_TIMES = a_inject_times;

    cache_size[0] = a_cache_size[0] * 1024;
    cache_size[1] = a_cache_size[1] * 1024;
    cache_line_size[0] = a_cache_line_size[0];
    cache_line_size[1] = a_cache_line_size[1];

    // 总的line数 = Cache总大小/每个line的大小（一般64byte，模拟的时候可配置）
    cache_line_num[0] = (_u64)cache_size[0] / a_cache_line_size[0];
    cache_line_num[1] = (_u64)cache_size[1] / a_cache_line_size[1];
    cache_line_shifts[0] = (_u64)log2(a_cache_line_size[0]);
    cache_line_shifts[1] = (_u64)log2(a_cache_line_size[1]);

    // 初始化一个行的每个字大小（字节）
    cache_word_size[0] = 4;
    cache_word_size[1] = 4;

    cache_word_num[0] = cache_line_size[0] / cache_word_size[0];
    cache_word_num[1] = cache_line_size[1] / cache_word_size[1];


    // 几路组相联
    cache_mapping_ways[0] = a_mapping_ways[0];
    cache_mapping_ways[1] = a_mapping_ways[1];

    // 总共有多少set
    cache_set_size[0] = cache_line_num[0] / cache_mapping_ways[0];
    cache_set_size[1] = cache_line_num[1] / cache_mapping_ways[1];

    // 其二进制占用位数，同其他shifts
    cache_set_shifts[0] = (_u64)log2(cache_set_size[0]);
    cache_set_shifts[1] = (_u64)log2(cache_set_size[1]);

    // 空闲块（line），初始所有的块都为空
    cache_free_num[0] = cache_line_num[0];
    cache_free_num[1] = cache_line_num[1];

    cache_r_count[0] = 0;
    cache_r_count[1] = 0;
    cache_w_count[0] = 0;
    cache_w_count[1] = 0;

    cache_w_memory_count = 0;
    cache_r_memory_count = 0;

    clean_count = 0;
    dirty_count = 0;

    period = 0;
    time_period = 0;
    emergency_period = 0;

    dirty_interval_time = 0;

    // 初始化MLREPS编码为关闭状态
    // 当遇到脏行出现错误，此时紧急回写机制触发，脏行变为干净行，启动针对脏行的MLREPS编码
    // MLREPS有运行周期，在下一次早期回写时，MLREPS的标志位变为false，重新换回SEC-DED编码
    START_MLREPS_FLAG = false;

    // 只内存写L2用这个值就可以，其他的用局部变量没问题
    m_data = 0xFFFF;    // 表示每次写入Cache的值

    clean_line_error_count = 0;
    dirty_line_error_count = 0;
    dirty_line_error_correct_count = 0;

    inject_count = 0;
    fail_correct = 0;
    error_but_not_detect_for_dirty = 0;
    error_but_not_detect_for_clean = 0;
    all_dirty_error = 0;

    // 指令数，主要用来在替换策略的时候提供比较的key，在命中或者miss的时候，相应line会更新自己的count为当时的tick_count
    tick_count = 0;

    // 为每一行分配空间
    int check_bits = 0;
    for (int i = 0; i < 2; ++i) {
        caches[i] = (Cache_Line*)malloc(sizeof(Cache_Line) * cache_line_num[i]);
        memset(caches[i], 0, sizeof(Cache_Line) * cache_line_num[i]);
        cache_buf[i] = (_u64 *)malloc(cache_size[i]);
        memset(cache_buf[i], 0, cache_size[i]);
        if (i == 1) {
            // Cache字大小=Cache行大小/几路组相连
            // parity是一个Cache字两个校验位
            // SEC-DED需要根据信息码的位数确定，即根据Cache行大小的位数确定
            // int small_block_num = cache_mapping_ways[1];
            int word_size_bits = (cache_line_size[1] / cache_mapping_ways[1]) * 8;
            // word_size_bits最好不要超过16个字节，要是超过16个字节会出问题
            if (word_size_bits >= 5 && word_size_bits <= 11) check_bits = 4;
            else if (word_size_bits >= 12 && word_size_bits <= 26) check_bits = 5;
            else if (word_size_bits >= 27 && word_size_bits <= 57) check_bits = 6;
            else if (word_size_bits >= 58 && word_size_bits <= 120) check_bits = 7;
            else check_bits = 8;
            check_bits = check_bits * cache_mapping_ways[1];
        }
    }
    // 为每一个Cache行分配校验位空间
    for (_u64 i = 0; i < cache_line_num[1]; ++i) {
        caches[1][i].extra_cache = (_u8*)malloc(sizeof(_u8) * check_bits);
        memset(caches[1][i].extra_cache, 0, sizeof(_u8) * check_bits);
        caches[1][i].extra_mem = (_u8*)malloc(sizeof(_u8) * 40);
        memset(caches[1][i].extra_mem, 0, sizeof(_u8) * 40);
    }
    // Replacement strategy
    if (l1_replace == 0) {
        swap_style[0] = CACHE_SWAP_LRU;
    } else if (l1_replace == 1) {
        swap_style[0] = CACHE_SWAP_FIFO;
    } else if (l1_replace == 2) {
        swap_style[0] = CACHE_SWAP_RAND;
    }

    if (l2_replace == 0) {
        swap_style[1] = CACHE_SWAP_LRU;
    } else if (l2_replace == 1) {
        swap_style[1] = CACHE_SWAP_FIFO;
    } else if (l2_replace == 2) {
        swap_style[1] = CACHE_SWAP_RAND;
    }

    re_init(check_bits);
    srand((unsigned)time(NULL));    // CACHE_SWAP_RAND替换相关
}

/*
 * 顶部的初始化放在最一开始，如果中途需要对tick_count进行清零和caches的清空，执行此函数
 * 主要因为tick_count的自增可能会超过unsigned long long
 * 而且一旦tick_count清零，Caches里的count数据也就出现了错误
*/
void CacheSim::re_init(int bits) {
    tick_count = 0;
    for (_u64 i = 0; i < cache_line_num[1]; ++i) {
        memset(caches[1][i].extra_cache, 0, sizeof(_u8) * bits);
        memset(caches[1][i].extra_mem, 0, sizeof(_u8) * 40);
    }
    memset(cache_hit_count, 0, sizeof(cache_hit_count));
    memset(cache_miss_count, 0, sizeof(cache_miss_count));
    cache_free_num[0] = cache_line_num[0];
    cache_free_num[1] = cache_line_num[1];
    // init函数已经使用malloc函数给caches分配了空间
    memset(caches[0], 0, sizeof(Cache_Line) * cache_line_num[0]);
    memset(caches[1], 0, sizeof(Cache_Line) * cache_line_num[1]);
    memset(cache_buf[0], 0, cache_size[0]);
    memset(cache_buf[1], 0, cache_size[1]);
}

/* 析构函数完成“清理善后”工作 */
//CacheSim::~CacheSim() {
//    for (_u64 i = 0; i < cache_line_num[1]; ++i) {
//        free(caches[1][i].extra_cache);
//    }
//    free(cache_buf[0]);
//    free(cache_buf[1]);
//    free(caches[0]);
//    free(caches[1]);
//}

/* 从文件读取trace，对结果统计 */
void CacheSim::load_trace(const char *filename, Ui::MainWindow *ui) {
    char buf[128];
    // 添加自己的input路径
    FILE *fin;
    // 记录的是trace中指令的读写，由于Cache机制，和真正的读写次数当然不一样
    // 主要是如果设置的写回法，则写会等在Cache中，直到被替换
    _u64 rcount = 0, wcount = 0, sum = 0;
    fin = fopen(filename, "r");

    if (!fin) {
        ui->textBrowser->insertPlainText("load file failed！\n");
        return;
    }

    //  故障注入初始化，确定故障的位置
    int n = INJECT_TIMES, min = 1, max = 300000;
    int* error_struct = (int*)malloc(sizeof(int) * 20 * n);
	memset(error_struct, 0, sizeof(int) * 20 * n);
    if (ERROR_TYPE > 0) {
        error_struct = cache_error_inject(min, max, n, ERROR_TYPE);
    }

    int k = 0;
    // trace文件逐行读入
    while (fgets(buf, sizeof(buf), fin)) {
        _u8 style = 0;
        _u64 addr = 0;
        sscanf(buf, "%c %llx", &style, &addr);

        // 记录总的读写次数
        sum++;

        // 每次到达故障时间，启用故障注入
        if (ERROR_TYPE > 0) {
            // 到达注入时间
            if ((int)sum == error_struct[k]) {
                k++;
                if (error_struct[k] == 1) {
                    k++;
                    if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                        inject_count++;
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                    }
                    k += 2;
                }
                if (error_struct[k] == 2) {
                    k++;
                    if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                        inject_count++;
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                    }
                    k += 3;
                }
                if (error_struct[k] == 3) {
                    k++;
                    if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                        inject_count++;
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                    }
                    k += 4;
                }
                if (error_struct[k] == 4) {
                    k++;
                    if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                        inject_count++;
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 4]);
                    }
                    k += 5;
                }
                if (error_struct[k] == 5) {
                    k++;
                    if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                        inject_count++;
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 4]);
                        reversebit(caches[1][error_struct[k]].buf, error_struct[k + 5]);
                    }
                    k += 6;
                }
                if (error_struct[k] == 12) {
                    k++;
                    if (error_struct[k] == 1) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        }
                        k += 2;
                    }
                    if (error_struct[k] == 2) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        }
                        k += 3;
                    }
                }
                if (error_struct[k] == 13) {
                    k++;
                    if (error_struct[k] == 1) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        }
                        k += 2;
                    }
                    if (error_struct[k] == 2) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        }
                        k += 3;
                    }
                    if (error_struct[k] == 3) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                        }
                        k += 4;
                    }
                }
                if (error_struct[k] == 14) {
                    k++;
                    if (error_struct[k] == 1) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        }
                        k += 2;
                    }
                    if (error_struct[k] == 2) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        }
                        k += 3;
                    }
                    if (error_struct[k] == 3) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                        }
                        k += 4;
                    }
                    if (error_struct[k] == 4) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 4]);
                        }
                        k += 5;
                    }
                }
                if (error_struct[k] == 15) {
                    k++;
                    if (error_struct[k] == 1) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                        }
                        k += 2;
                    }
                    if (error_struct[k] == 2) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                        }
                        k += 3;
                    }
                    if (error_struct[k] == 3) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                        }
                        k += 4;
                    }
                    if (error_struct[k] == 4) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 4]);
                        }
                        k += 5;
                    }
                    if (error_struct[k] == 5) {
                        k++;
                        if (caches[1][error_struct[k]].flag & CACHE_FLAG_VALID) {
                            inject_count++;
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 1]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 2]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 3]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 4]);
                            reversebit(caches[1][error_struct[k]].buf, error_struct[k + 5]);
                        }
                        k += 6;
                    }
                }
            }
        }
        // 对读入的一行操作
        do_cache_op(addr, style, ui);
        switch (style) {
            case 'l' :
                rcount++;
                break;
            case 's' :
                wcount++;
                break;
        }
    }

    string text;
    text = "all r/w/sum: " + to_string(rcount) + "/" + to_string(wcount) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "read rate: " + to_string(100.0 * rcount / tick_count) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "write rate: " + to_string(100.0 * wcount / tick_count) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L1 miss/hit: " + to_string(cache_miss_count[0]) + "/" + to_string(cache_hit_count[0]) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L1 hit rate: " + to_string(100.0 * cache_hit_count[0] / (cache_hit_count[0] + cache_miss_count[0])) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L1 miss rate: " + to_string(100.0 * cache_miss_count[0] / (cache_miss_count[0] + cache_hit_count[0])) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 miss/hit: " + to_string(cache_miss_count[1]) + "/" + to_string(cache_hit_count[1]) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 hit rate: " + to_string(100.0 * cache_hit_count[1] / (cache_hit_count[1] + cache_miss_count[1])) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 miss rate: " + to_string(100.0 * cache_miss_count[1] / (cache_miss_count[1] + cache_hit_count[1])) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "clean line num: " + to_string(clean_count) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "dirty line num: " + to_string(dirty_count) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "clean rate: " + to_string(100.0 * clean_count / (clean_count + dirty_count)) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "dirty rate: " + to_string(100.0 * dirty_count / (clean_count + dirty_count)) + "%" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    // 读写通信，两级Cache的话应该是读写L1/L2的次数
    text = "L1 read num: " + to_string(cache_r_count[0]) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L1 write num: " + to_string(cache_w_count[0]) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 read num: " + to_string(cache_r_count[1]) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 write num: " + to_string(cache_w_count[1]) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L1 read communication: " + to_string(((cache_r_count[0] * cache_line_size[0]) >> 10)) + "KB" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L1 write communication: " + to_string(((cache_w_count[0] * cache_line_size[0]) >> 10)) + "KB" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 read communication: " + to_string(((cache_r_count[1] * cache_line_size[1]) >> 10)) + "KB" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "L2 write communication: " + to_string(((cache_w_count[1] * cache_line_size[1]) >> 10)) + "KB" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "Memory read num: " + to_string(cache_r_memory_count) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "Memory write num: " + to_string(cache_w_memory_count) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "Memory read communication: " + to_string(((cache_r_memory_count * cache_line_size[1]) >> 10)) + "KB" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "Memory write communication: " + to_string(((cache_w_memory_count * cache_line_size[1]) >> 10)) + "KB" + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));

    text = "all clean correct: " + to_string(clean_line_error_count) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "clean error: " + to_string(error_but_not_detect_for_clean) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "all dirty correct: " + to_string(dirty_line_error_correct_count) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));
    text = "dirty error: " + to_string((all_dirty_error - dirty_line_error_correct_count)) + "\n";
    ui->textBrowser->insertPlainText(QString::fromStdString(text));

    ui->textBrowser->insertPlainText("================================\n");
    ui->textBrowser->insertPlainText("\n");

    fclose(fin);
}

/*
 * @arg set_base[] set基址
 * @arg addr[] trace文件中内存地址
 * @arg level[] 哪一级Cache
 * 注：循环查找当前set的所有way（line），通过tag匹配，查看当前地址是否在Cache中
*/
int CacheSim::check_cache_hit(_u64 set_base, _u64 addr, int level, Ui::MainWindow *ui) {
    _u64 i;
    for (i = 0; i < cache_mapping_ways[level]; ++i) {
        if ((caches[level][set_base + i].flag & CACHE_FLAG_VALID) &&
            (caches[level][set_base + i].tag == ((addr >> (cache_set_shifts[level] + cache_line_shifts[level]))))) {
            // tag检查是否命中前可以加校验
            return set_base + i;
        }
    }
    // -1表示这个addr中的数据并没有加载到cache上
    return -1;
}

/*
 * @arg set_base[] set基址
 * @arg level[] 哪一级Cache
 * 注：获取当前set中可用的line，如果没有，就找到要被替换的块
*/
int CacheSim::get_cache_free_line(_u64 set_base, int level, int style, Ui::MainWindow *ui) {
    _u64 i, min_count, j;
    int free_index;
    // 从当前cache set里找可用的空闲line，可用：脏数据/空闲数据（无效数据）
    // cache_free_num是统计的整个cache的可用块
    for (i = 0; i < cache_mapping_ways[level]; ++i) {
        if (!(caches[level][set_base + i].flag & CACHE_FLAG_VALID)) {
            // 既然能走到这里，就说明cache行有无效的，即初始的Cache line未占满，所以肯定有空闲位置
            // 此时，如果内存写数据到l2，则干净行的数量增加
            if (cache_free_num[level] > 0) {
                cache_free_num[level]--;
                return set_base + i;
            }
        }
    }
    // 没有可用line，则执行替换算法
    free_index = -1;
    if (swap_style[level] == CACHE_SWAP_RAND) {
        free_index = rand() % cache_mapping_ways[level];
    } else {
        // FIFO或者LRU替换算法
        min_count = ULLONG_MAX;
        for (j = 0; j < cache_mapping_ways[level]; ++j) {
            if (caches[level][set_base + j].count < min_count) {
                min_count = caches[level][set_base + j].count;
                free_index = j;
            }
        }
    }
    if (free_index >= 0) {
        free_index += set_base;
        // 如果原有Cache line是脏数据，写回内存,只有L2可以来到这里
        // l1读未命中l2读未命中，内存向l2写数据，但是l2为脏行，需要将其替换，此时新的l2为干净行
        // l2写未命中，cpu/l1向l2写数据，但是l2为脏行，需要将其替换，此时新的l2为脏行
        if (caches[level][free_index].flag & CACHE_FLAG_DIRTY) {
            cache_r_count[1]++;
            cache_w_memory_count++;
            period = period + 10 + 100;
            // 添加：错误间隔周期
            dirty_interval_time = dirty_interval_time + 10 + 100;
            l2_write_to_mem((_u64)free_index, ui);
        } else {
            // 原先l2为干净行，直接将干净行替换，然后把该行变为脏行
        }
        return free_index;
    } else {
        cout << "error!" << endl;
    }
}

/* 找到合适的line之后，将数据写入L1 Cache line中 */
void CacheSim::cpu_mem_write_to_l1(_u64 index, _u64 addr, Ui::MainWindow *ui) {
    Cache_Line *line = caches[0] + index;
    line->buf = 0xFFFF;
    line->tag = addr >> (cache_set_shifts[0] + cache_line_shifts[0]);
    line->flag = (_u8) ~CACHE_FLAG_MASK;
    line->flag |= CACHE_FLAG_VALID;
    line->count = tick_count;
}

/* 找到合适的line之后，将数据写入L1 Cache line中 */
void CacheSim::l2_write_to_l1(_u64 l1_index, _u64 l2_index, _u64 addr, Ui::MainWindow *ui) {
    // 需要先对l2解码，然后再做后续的操作
    _u64 data = caches[1][l2_index].buf;
    if (caches[1][l2_index].flag & CACHE_FLAG_DIRTY) {
        // 对脏行解码
        if (!START_MLREPS_FLAG) {
            if (SECDED == 1) hamming_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
            if (SECDED == 2) secded_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        } else {
            if (MLREPS == 1) mlreps_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        }
    }
    if (!(caches[1][l2_index].flag & CACHE_FLAG_DIRTY)) {
        // 对干净行解码
        if (PARITY_CHECK == 1 || PARITY_CHECK == 2) {
            parity_check_decode_for_cache(l2_index, data, ui);
        }
    }
    Cache_Line *line = caches[0] + l1_index;
    line->buf = caches[1][l2_index].buf;
    line->tag = addr >> (cache_set_shifts[0] + cache_line_shifts[0]);
    line->flag = (_u8) ~CACHE_FLAG_MASK;
    line->flag |= CACHE_FLAG_VALID;
    line->count = tick_count;
}

/* l2写数据到cpu */
void CacheSim::l2_write_to_cpu(_u64 l2_index, Ui::MainWindow *ui) {
    _u64 data = caches[1][l2_index].buf;
    if (caches[1][l2_index].flag & CACHE_FLAG_DIRTY) {
        // 对脏行解码
        if (!START_MLREPS_FLAG) {
            if (SECDED == 1) hamming_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
            if (SECDED == 2) secded_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        } else {
            if (MLREPS == 1) mlreps_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        }   
    }
    if (!(caches[1][l2_index].flag & CACHE_FLAG_DIRTY)) {
        // 对干净行解码
        if (PARITY_CHECK == 1 || PARITY_CHECK == 2) {
            parity_check_decode_for_cache(l2_index, data, ui);
            // parity_check_decode_for_cache(l2_index, 62463);
        }
    }
}

/* l2写数据到mem，需要脏数据先解码 */
void CacheSim::l2_write_to_mem(_u64 l2_index, Ui::MainWindow *ui) {
    if (!START_MLREPS_FLAG) {
        if (SECDED == 1) hamming_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        if (SECDED == 2) secded_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        // if (SECDED == 1) secded_decode_for_cache(l2_index, 64511);
        // if (SECDED == 2) secded_decode_for_cache(l2_index, 64511);
    } else {
        if (MLREPS == 1) mlreps_decode_for_cache(l2_index, caches[1][l2_index].buf, ui);
        // if (MLREPS == 1) mlreps_decode_for_cache(l2_index, 63231);
    }
    // test 65503 = 1111111111011111    62463 = 1111001111111111
    _u64 data = caches[1][l2_index].buf;
}

/* 找到合适的line之后，将数据写入L2 Cache line中 */
void CacheSim::l1_cpu_write_to_l2(_u64 index, _u64 addr, Ui::MainWindow *ui) {
    Cache_Line *line = caches[1] + index;
    _u64 data = 0xFFFF;
    line->buf = data;
    line->tag = addr >> (cache_set_shifts[1] + cache_line_shifts[1]);
    line->flag = (_u8) ~CACHE_FLAG_MASK;
    line->flag |= CACHE_FLAG_VALID;
    line->count = tick_count;
    // l1/cpu写数据到l2，l2只能添加SEC-DED的校验位，因为此时l2是脏的
    if (!START_MLREPS_FLAG) {
        if (SECDED == 1) hamming_encode_for_cache(index, data);
        if (SECDED == 2) secded_encode_for_cache(index, data);
    } else {
        if (MLREPS == 1) mlreps_encode_for_cache(index, data);
    }
}

/* 
 * 实现普通的偶校验
 * 奇校验：二进制数据中检验“1”的个数是否为奇数，若为奇数，则校验位为0；若为偶数，则校验位为1
 * 偶校验：二进制数据中检验“1”的个数是否为偶数，若为偶数，则校验位为0；若为奇数，则检验位为1
 * 设计细粒度每字奇偶校验，输入一个整数，输出为一位校验位
 * 返回值为1表示生成的校验位为1，返回值为1表示生成的校验位为0
*/
int CacheSim::parity_check(_u64 m) {
    bool parity = false;
    while (m) {
        parity = !parity;
        m = m & (m - 1);
    }
    return parity == true? 1 : 0;
}

/* 
 * 实现间隔为1的偶校验
 * 返回值为指向数组的指针
 * 第一个元素为奇数位置添加的校验位，第二个元素为偶数位置添加的校验位
*/
int* CacheSim::update_parity_check(_u64 m) {
    int *parity = (int*)malloc(sizeof(int) * 2);
    int odd1 = 0, odd2 = 0;
    int i = 64;     // 现数据字64位
    while (i > 0) {
        i = i - 2;
        int temp1 = 0, temp2 = 0;
        temp1 = m >> i;
        temp2 = m >> i;
        if ((temp1 & 1) == 1) odd1++;
        if ((temp2 & 2) == 2) odd2++;
    }
    parity[0] = (odd1 % 2 == 1)? 1 : 0;
    parity[1] = (odd2 % 2 == 1)? 1 : 0;
    return parity;
}

/* parity check decode for Cache */
void CacheSim::parity_check_decode_for_cache(_u64 l2_index, _u64 data, Ui::MainWindow *ui) {
    if (PARITY_CHECK == 1) {
        for (_u64 i = 0; i < cache_word_num[1]; ++i) {
            _u64 temp = data & (_u64)(pow(2, cache_word_size[1] * 8) - 1);
            if (caches[1][l2_index].extra_cache[i] != (_u8)parity_check(temp)) {
                clean_line_error_count++;
                ui->textBrowser->insertPlainText("奇偶校验检测到错误！\n");
                // 之后需要从下一级存储器恢复
                caches[1][l2_index].buf = *((_u64 *)(void *)(caches[1][l2_index].addr));
                ui->textBrowser->insertPlainText("已经从下一级存储器恢复！\n");
                ui->textBrowser->insertPlainText("正确数据为：");
                ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][l2_index].buf)));
                ui->textBrowser->insertPlainText("\n");
                // 检测到错误就恢复并立即返回
                return;
            }
            data = data >> cache_word_size[1] * 8;
        }
        if (caches[1][l2_index].buf != 0xFFFF) {
            ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][l2_index].buf)));
            ui->textBrowser->insertPlainText("\n");
            error_but_not_detect_for_clean++;
            ui->textBrowser->insertPlainText("parity未检测到错误！\n");
            // 为了之后故障注入的正确，这里修改为正确数据
            // 也需要一个重新编码
            parity_check_encode_for_cache(l2_index, 0xFFFF);
            caches[1][l2_index].buf = 0xFFFF;
        }
    }
    if (PARITY_CHECK == 2) {
        for (_u64 i = 0; i < 2 * cache_word_num[1]; i += 2) {
            _u64 temp = data & (_u64)(pow(2, cache_word_size[1] * 8) - 1);
            int *a = update_parity_check(temp);
            if (caches[1][l2_index].extra_cache[i] != a[0] ||
                caches[1][l2_index].extra_cache[i + 1] != a[1]) {
                clean_line_error_count++;
                ui->textBrowser->insertPlainText("奇偶校验检测到错误！\n");
                caches[1][l2_index].buf = *((_u64 *)(void *)(caches[1][l2_index].addr));
                ui->textBrowser->insertPlainText("已经从下一级存储器恢复！\n");
                ui->textBrowser->insertPlainText("正确数据为：");
                ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][l2_index].buf)));
                ui->textBrowser->insertPlainText("\n");
                return;
            }
            data = data >> cache_word_size[1] * 8;
        }
        if (caches[1][l2_index].buf != 0xFFFF) {
            ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][l2_index].buf)));
            ui->textBrowser->insertPlainText("\n");
            error_but_not_detect_for_clean++;
            ui->textBrowser->insertPlainText("parity未检测到错误！\n");
            // 为了之后故障注入的正确，这里修改为正确数据
            parity_check_encode_for_cache(l2_index, 0xFFFF);
            caches[1][l2_index].buf = 0xFFFF;
        }
    }
}

/* parity check encode for Cache */
void CacheSim::parity_check_encode_for_cache(_u64 l2_index, _u64 data) {
    // 需要添加parity check bits
    if (PARITY_CHECK == 1) {
        caches[1][l2_index].extra_cache = (_u8 *)malloc(sizeof(_u8) * cache_word_num[1]);
        memset(caches[1][l2_index].extra_cache, 0, sizeof(_u8) * cache_word_num[1]);
        for (_u64 i = 0; i < cache_word_num[1]; ++i) {
            _u64 temp = data & (_u64)(pow(2, cache_word_size[1] * 8) - 1);
            caches[1][l2_index].extra_cache[i] = (_u8)parity_check(temp);
            data = data >> cache_word_size[1] * 8;
        }
    }
    if (PARITY_CHECK == 2) {
        caches[1][l2_index].extra_cache = (_u8 *)malloc(sizeof(_u8) * cache_word_num[1] * 2);
        memset(caches[1][l2_index].extra_cache, 0, sizeof(_u8) * cache_word_num[1] * 2);
        for (_u64 i = 0; i < 2 * cache_word_num[1]; i += 2) {
            _u64 temp = data & (_u64)(pow(2, cache_word_size[1] * 8) - 1);
            int *a = update_parity_check(temp);
            caches[1][l2_index].extra_cache[i] = a[0];
            caches[1][l2_index].extra_cache[i + 1] = a[1];
            data = data >> cache_word_size[1] * 8;
        }
    }
}

/* 找到合适的line之后，将数据写入L2 Cache line中 */
void CacheSim::mem_write_to_l2(_u64 index, _u64 addr, Ui::MainWindow *ui) {
    Cache_Line *line = caches[1] + index;
    line->tag = addr >> (cache_set_shifts[1] + cache_line_shifts[1]);
    line->flag = (_u8) ~CACHE_FLAG_MASK;
    line->flag |= CACHE_FLAG_VALID;
    line->count = tick_count;
    line->buf = m_data;
    line->addr = (_u64*)&m_data;
    parity_check_encode_for_cache(index, m_data);
}

/*
 * @arg addr trace文件中的地址
 * @arg oper_style trace文件中的读写标识
*/
void CacheSim::do_cache_op(_u64 addr, char oper_style, Ui::MainWindow *ui) {
    // 映射到哪个set，当前set的首地址
    _u64 set_l1, set_l2, set_base_l1, set_base_l2;
    long long hit_index_l1, hit_index_l2, free_index_l1, free_index_l2;
    tick_count++;
    // 根据对地址的区域划分，来获取当前地址映射到Cache的哪一个set中
    set_l2 = (addr >> cache_line_shifts[1]) % cache_set_size[1];
    set_base_l2 = set_l2 * cache_mapping_ways[1];
    hit_index_l2 = check_cache_hit(set_base_l2, addr, 1, ui);
    set_l1 = (addr >> cache_line_shifts[0]) % cache_set_size[0];
    set_base_l1 = set_l1 * cache_mapping_ways[0];
    hit_index_l1 = check_cache_hit(set_base_l1, addr, 0, ui);

    // 1、l1读命中（不管l2是否读命中，l1命中次数+1）
    if (hit_index_l1 >= 0 && oper_style == OPERATION_READ) {
        // 操作：从l1读数据到cpu
        // l1命中次数+1
        cache_hit_count[0]++;
        // 从l1读数据到cpu，则l1的读取次数+1
        cache_r_count[0]++;
        // 假设从l1读数据到cpu花费了一个时钟周期
        period = period + 1;
        dirty_interval_time = dirty_interval_time + 1;
        // 只有在LRU的时候才更新时间戳，第一次设置时间戳是在被放入数据的时候，所以符合FIFO
        if (CACHE_SWAP_LRU == swap_style[0]) {
            caches[0][hit_index_l1].lru_count = tick_count;
        }
    }

    // 2、l1读未命中，l2读命中
    if (hit_index_l1 < 0 && hit_index_l2 >= 0 && oper_style == OPERATION_READ) {
        // 操作：同时从l2读取数据到l1和cpu
        cache_miss_count[0]++;
        cache_hit_count[1]++;
        cache_r_count[1]++;
        cache_w_count[0]++;
        period = period + 10 + 1;
        dirty_interval_time = dirty_interval_time + 10 + 1;
        free_index_l1 = get_cache_free_line(set_base_l1, 0, OPERATION_READ, ui);
        l2_write_to_l1((_u64)free_index_l1, (_u64)hit_index_l2, addr, ui);
        l2_write_to_cpu((_u64)hit_index_l2, ui);
        // 只要命中了，就修改最近的访问时间
        if (CACHE_SWAP_LRU == swap_style[1]) {
            caches[1][hit_index_l2].lru_count = tick_count;
        }
    }

    // 3、l1读未命中，l2读未命中
    if (hit_index_l1 < 0 && hit_index_l2 < 0 && oper_style == OPERATION_READ) {
        // 操作：同时从内存读取数据到l2/l1和cpu
        cache_miss_count[0]++;
        cache_miss_count[1]++;
        cache_w_count[0]++;
        cache_w_count[1]++;
        cache_r_memory_count++;
        period = period + 100 + 10;
        dirty_interval_time = dirty_interval_time + 100 + 10;
        // 从内存读取数据到l1/cpu
        free_index_l1 = get_cache_free_line(set_base_l1, 0, OPERATION_READ, ui);
        cpu_mem_write_to_l1((_u64)free_index_l1, addr, ui);
        // 从内存读取数据到l2
        free_index_l2 = get_cache_free_line(set_base_l2, 1, OPERATION_READ, ui);
        // 此时多了一条干净行
        clean_count++;
        mem_write_to_l2((_u64)free_index_l2, addr, ui);
        caches[1][free_index_l2].flag &= ~CACHE_FLAG_DIRTY;
    }

    // 4、l1写命中，l2写命中
    if (hit_index_l1 >= 0 && hit_index_l2 >= 0 && oper_style == OPERATION_WRITE) {
        // 操作：设置l2为脏行（如果之前是干净行的话）
        cache_hit_count[0]++;
        cache_hit_count[1]++;
        if (!(caches[1][hit_index_l2].flag & CACHE_FLAG_DIRTY)) {
            dirty_count++;
            caches[1][hit_index_l2].flag |= CACHE_FLAG_DIRTY;
            // 脏行需要编码
            if (!START_MLREPS_FLAG) {
                if (SECDED == 1) hamming_encode_for_cache(hit_index_l2, caches[1][hit_index_l2].buf);
                if (SECDED == 2) secded_encode_for_cache(hit_index_l2, caches[1][hit_index_l2].buf);
            } else {
                if (MLREPS == 1) mlreps_encode_for_cache(hit_index_l2, caches[1][hit_index_l2].buf);
            } 
        }
        if (CACHE_SWAP_LRU == swap_style[0]) {
            caches[0][hit_index_l1].lru_count = tick_count;
        }
        if (CACHE_SWAP_LRU == swap_style[1]) {
            caches[1][hit_index_l2].lru_count = tick_count;
        }
    }

    // 5、l1写命中，l2写未命中
    if (hit_index_l1 >= 0 && hit_index_l2 < 0 && oper_style == OPERATION_WRITE) {
        // 操作：将数据从l1写入l2，并且设置l2为脏行
        cache_hit_count[0]++;
        cache_miss_count[1]++;
        cache_r_count[0]++;
        cache_w_count[1]++;
        period = period + 1 + 10;
        dirty_interval_time = dirty_interval_time + 1 + 10;
        free_index_l2 = get_cache_free_line(set_base_l2, 1, OPERATION_WRITE, ui);
        // 需要脏行编码
        l1_cpu_write_to_l2((_u64)free_index_l2, addr, ui);
        if (!(caches[1][free_index_l2].flag & CACHE_FLAG_DIRTY)) {
            dirty_count++;
            caches[1][free_index_l2].flag |= CACHE_FLAG_DIRTY;
        }
        if (CACHE_SWAP_LRU == swap_style[0]) {
            caches[0][hit_index_l1].lru_count = tick_count;
        }
    }

    // 6、l1写未命中，l2写命中
    if (hit_index_l1 < 0 && hit_index_l2 >= 0 && oper_style == OPERATION_WRITE) {
        // 操作：设置l2脏位为1
        cache_miss_count[0]++;
        cache_hit_count[1]++;
        if (!(caches[1][hit_index_l2].flag & CACHE_FLAG_DIRTY)) {
            dirty_count++;
            caches[1][hit_index_l2].flag |= CACHE_FLAG_DIRTY;
            if (!START_MLREPS_FLAG) {
                if (SECDED == 1) hamming_encode_for_cache(hit_index_l2, caches[1][hit_index_l2].buf);
                if (SECDED == 2) secded_encode_for_cache(hit_index_l2, caches[1][hit_index_l2].buf);                
            } else {
                if (MLREPS == 1) mlreps_encode_for_cache(hit_index_l2, caches[1][hit_index_l2].buf);
            }
        }
        if (CACHE_SWAP_LRU == swap_style[1]) {
            caches[1][hit_index_l2].lru_count = tick_count;
        }
    }

    // 7、l1写未命中，l2写未命中
    if (hit_index_l1 < 0 && hit_index_l2 < 0 && oper_style == OPERATION_WRITE) {
        // 操作：cpu同时将数据写入l1和l2，且设置l2的脏位为1
        cache_miss_count[0]++;
        cache_miss_count[1]++;
        cache_w_count[0]++;
        cache_w_count[1]++;
        period = period + 10;
        dirty_interval_time = dirty_interval_time + 10;
        free_index_l1 = get_cache_free_line(set_base_l1, 0, OPERATION_WRITE, ui);
        cpu_mem_write_to_l1((_u64)free_index_l1, addr, ui);
        free_index_l2 = get_cache_free_line(set_base_l2, 1, OPERATION_WRITE, ui);
        // 需要脏行编码
        l1_cpu_write_to_l2((_u64)free_index_l2, addr, ui);
        if (!(caches[1][free_index_l2].flag & CACHE_FLAG_DIRTY)) {
            dirty_count++;
            caches[1][free_index_l2].flag |= CACHE_FLAG_DIRTY;
        }
    }

    // 早期回写
    if (EARLY_WRITE_BACK == 1) {
        early_write_back(50, ui);
    }
}

/* 脏行的编码/解码操作 */
_u64 CacheSim::cal(_u64 sz) {
	_u64 k = 0;
	_u64 cur = 1;
	while (cur - 1 < sz + k) {
		cur <<= 1;
		k++;
	}
	return k;
}

string CacheSim::hamming_encode(string s) {
	string d;
    d.clear();
    _u64 k = cal(s.size());
    d.resize(s.size() + k);
    for (_u64 i = 0, j = 0, p = 0; i!= d.size(); i++) {
        if ((i + 1) == pow(2,p) && p < k) {
            d[i] = '0';
            p++;
        } else if (s[j] == '0' || s[j] == '1') {
			d[i] = s[j++];
		} else {
			cout << "error!" << endl;
		}
    }
    for (_u64 i = 0; i != k; i++) {
        _u64 count = 0, index = 1 << i;
		for (_u64 j = index - 1; j < d.size(); j += index) {
			for (_u64 k = 0; k != index && j < d.size(); k++, j++) {
				count ^= d[j] - '0';
			}
		}
        d[index - 1] = '0' + count;
    }
    return d;
}

_u64 CacheSim::antiCal(_u64 sz) {
	_u64 k = 0;
	_u64 cur = 1;
	while (cur < sz) {
		cur <<= 1;
		k++;
	}
	return k;
}

int CacheSim::hamming_decode(string d, string &s) {
    s.clear();
	_u64 k = antiCal(d.size());
    s.resize(d.size() - k);
    _u64 sum = 0;
    for (_u64 p = 0; p != k; p++) {
        int pAnti = 0;
		_u64 index = 1 << p;
        for (_u64 i = index - 1; i < d.size(); i += index) {
			for (_u64 j = 0; j < index && i < d.size(); i++, j++) {
				pAnti ^= d[i] - '0';
			}
        }
        sum += pAnti << p;
    }
	// 现在假定Cache行大小最少为8字节，数据位为64位，那么校验位为7位
	if (sum > 71) {
		// cout << "汉明码出现不能纠正的错误!" << endl;
		return -1;
	}
	if (sum != 0) {
		d[sum - 1] = (1 - (int)(d[sum - 1] - '0')) + '0';
	}
    for (_u64 i = 0, p = 0, j = 0; i != d.size(); i++) {
		if ((i + 1) == (1 << p) && p < k) {
			p++;
		}
		else {
			s[j++] = d[i];
		}
    }
    return sum;
}

string CacheSim::dec_to_bin(_u64 m) {
	char* bin = (char*)malloc(sizeof(char) * 65);
	memset(bin, 0, sizeof(char) * 65);
	for (int i = 63; i >= 0; i--) {
		int temp = ((m >> i) & 1);
		bin[i] = (char)(temp + '0');
	}
	bin[64] = '\0';
	return bin;
}

_u64 CacheSim::bin_to_dec(string bin) {
	_u64 size = bin.size();
	_u64 parseBinary = 0;
	for (_u64 i = 0; i < size; ++i) {
		if (bin[i] == '1') {
			parseBinary += pow(2.0, size - i - 1);
		}
	}
	return parseBinary;
}

void CacheSim::hamming_encode_for_cache(_u64 index, _u64 data) {
    // SEC-DED针对Cache line，但是每次写入是64位，并且设置Cache line最小为8字节，所以这里为64位数据编码
    // 64位数据编码需要7个校验位
    caches[1][index].extra_cache = (_u8 *)malloc(sizeof(_u8) * 8);
    memset(caches[1][index].extra_cache, 0, sizeof(_u8) * 8);
    caches[1][index].extra_cache[7] = '\0';
    string str = dec_to_bin(data);
    string encode_str = hamming_encode(str);
    for (int i = 0; i < 7; ++i) {
		caches[1][index].extra_cache[i] = encode_str[pow(2, i) - 1];
	}
}

void CacheSim::hamming_decode_for_cache(_u64 index, _u64 data, Ui::MainWindow *ui) {
    string decode_str = "";
    string str = dec_to_bin(data);
    decode_str = decode_str + (char)caches[1][index].extra_cache[0] + (char)caches[1][index].extra_cache[1] +
        str.substr(0,1) + (char)caches[1][index].extra_cache[2] + str.substr(1,3) + (char)caches[1][index].extra_cache[3] +
        str.substr(4,7) + (char)caches[1][index].extra_cache[4] + str.substr(11,15) + (char)caches[1][index].extra_cache[5] +
        str.substr(26,21) + (char)caches[1][index].extra_cache[6] + str.substr(47,18);
    string real_data;
    int ret = hamming_decode(decode_str, real_data);;
    if (ret < 0) {
        // dirty_line_error_count++;
        all_dirty_error++;
        ui->textBrowser->insertPlainText("汉明码出现不能纠正的错误!\n");
        // 出现不能纠错，为了后续注入，这里再修改为正确的
        hamming_encode_for_cache(index, 0xFFFF);
        caches[1][index].buf = 0xFFFF;

        // 启用紧急回写机制，后续加MLREPS校验
        if (EMERGENCY_WRITE_BACK == 1) {
            emergency_write_back(ui);
        }
    }
    if (ret == 0) {
        if (caches[1][index].buf != 0xFFFF) {
            ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][index].buf)));
            ui->textBrowser->insertPlainText("\n");
            // error_but_not_detect_for_dirty++;
            all_dirty_error++;
            ui->textBrowser->insertPlainText("未检测到错误！\n");
            // 出现不能检测纠错，为了后续注入，这里再修改为正确的
            hamming_encode_for_cache(index, 0xFFFF);
            caches[1][index].buf = 0xFFFF;
        }
    }
    if (ret > 0) {
        // dirty_line_error_count++;
        all_dirty_error++;
        ui->textBrowser->insertPlainText("汉明码检测到错误！\n");
        reverse(real_data.begin(), real_data.end());
        caches[1][index].buf = bin_to_dec(real_data);
        ui->textBrowser->insertPlainText("错误已修复！\n");
        ui->textBrowser->insertPlainText("正确数据为：");
        ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][index].buf)));
        ui->textBrowser->insertPlainText("\n");

        if (caches[1][index].buf != 0xFFFF) {
            // fail_correct++;
            // 出现不能检测纠错，为了后续注入，这里再修改为正确的
            hamming_encode_for_cache(index, 0xFFFF);
            caches[1][index].buf = 0xFFFF;
        } else {
            dirty_line_error_correct_count++;
        }

        // 启用紧急回写机制，后续加MLREPS校验
        if (EMERGENCY_WRITE_BACK == 1) {
            emergency_write_back(ui);
        }
    }
}

string CacheSim::secded_encode(string s) {
	string d;
	d.clear();
	_u64 k = cal(s.size());
	d.resize(s.size() + k);
	// 按照校验位个数填充0
	for (_u64 i = 0, j = 0, p = 0; i != d.size(); i++) {
		if ((i + 1) == pow(2, p) && p < k) {
			d[i] = '0';
			p++;
		} else if (s[j] == '0' || s[j] == '1') {
			d[i] = s[j++];
		} else {
			cout << "error!" << endl;
		}
	}
	// 要记录每个位被访问了多少次
	int* n = (int*)malloc(sizeof(int) * d.size());
	memset(n, 0, sizeof(int) * d.size());
	for (_u64 i = 0; i != k; i++) {
		_u64 count = 0, index = 1 << i;
		for (_u64 j = index - 1; j < d.size(); j += index) {
			for (_u64 k = 0; k != index && j < d.size(); k++, j++) {
				count ^= d[j] - '0';
				n[j]++;
			}
		}
		d[index - 1] = '0' + count;
	}
	// 补充一个校验位
	_u64 count = 0;
	for (_u64 i = 0; i < d.size(); ++i) {
		if (n[i] == 2) {
			count ^= d[i] - '0';
			n[i]++;
		}
	}
	d += ('0' + count);
	return d;
}

int CacheSim::secded_decode(string _d, string &s) {
	s.clear();
	string d = _d.substr(0, _d.size() - 1);
	_u64 k = antiCal(d.size());
	s.resize(d.size() - k);
	_u64* parity = (_u64*)malloc(sizeof(_u64) * (k + 1));
	memset(parity, 0, sizeof(int) * (k + 1));
	int* n = (int*)malloc(sizeof(int) * d.size());
	memset(n, 0, sizeof(int) * d.size());

	_u64 sum = 0;
	for (_u64 p = 0; p != k; p++) {
		_u64 pAnti = 0;
		_u64 index = 1 << p;
		for (_u64 i = index - 1; i < d.size(); i += index) {
			for (_u64 j = 0; j < index && i < d.size(); i++, j++) {
				pAnti ^= d[i] - '0';
				n[i]++;
			}
		}
		parity[p] = pAnti;
		sum += pAnti << p;
	}
	
	_u64 count = 0;
	for (_u64 i = 0; i < d.size(); ++i) {
		if (n[i] == 2) {
			count ^= d[i] - '0';
			n[i]++;
		}
	}

	parity[k] = count ^ (_d[_d.size() - 1] - '0');
	
	int not_zero = 0;
	for (int i = 0; i < k + 1; ++i) {
		if (parity[i] != 0) {
			++not_zero;
		}
	}
	if (not_zero == 0) {
        s = "";
		return 0;
	}
	else if (not_zero >= 3 && not_zero <= 6) {
        if (sum > d.size()) {
			return -1;
		}
		if (sum != 0) {
			d[sum - 1] = (1 - (int)(d[sum - 1] - '0')) + '0';
		}
		for (_u64 i = 0, p = 0, j = 0; i != d.size(); i++) {
			if ((i + 1) == (1 << p) && p < k) {
				p++;
			}
			else {
				s[j++] = d[i];
			}
		}
		return 1;
	}
	else {
		// cout << "SEC-DED出现不能纠正的错误!" << endl;
		return -1;
	}
    // return 0;
}

void CacheSim::secded_encode_for_cache(_u64 index, _u64 data) {
    // SEC-DED针对Cache line，但是每次写入是64位，并且设置Cache line最小为8字节，所以这里为64位数据编码
    // 64位数据编码需要8个校验位，hamming需要7个校验位
    caches[1][index].extra_cache = (_u8 *)malloc(sizeof(_u8) * 9);
    memset(caches[1][index].extra_cache, 0, sizeof(_u8) * 9);
    caches[1][index].extra_cache[8] = '\0';
    string str = dec_to_bin(data);
    string encode_str = secded_encode(str);
    for (int i = 0; i < 7; ++i) {
		caches[1][index].extra_cache[i] = encode_str[pow(2, i) - 1];
	}
    caches[1][index].extra_cache[7] = encode_str[encode_str.size() - 1];
}

void CacheSim::secded_decode_for_cache(_u64 index, _u64 data, Ui::MainWindow *ui) {
    string decode_str = "";
    string str = dec_to_bin(data);
    decode_str = decode_str + (char)caches[1][index].extra_cache[0] + (char)caches[1][index].extra_cache[1] +
        str.substr(0,1) + (char)caches[1][index].extra_cache[2] + str.substr(1,3) + (char)caches[1][index].extra_cache[3] +
        str.substr(4,7) + (char)caches[1][index].extra_cache[4] + str.substr(11,15) + (char)caches[1][index].extra_cache[5] +
        str.substr(26,21) + (char)caches[1][index].extra_cache[6] + str.substr(47,18) + (char)caches[1][index].extra_cache[7];
    
    string real_data;
    int ret = secded_decode(decode_str, real_data);

    if (ret < 0) {
        // dirty_line_error_count++;
        all_dirty_error++;
        ui->textBrowser->insertPlainText("SEC-DED出现不能纠正的错误!\n");
        // 出现不能纠错，为了后续注入，这里再修改为正确的
        secded_encode_for_cache(index, 0xFFFF);
        caches[1][index].buf = 0xFFFF;

        // 启用紧急回写机制，后续加MLREPS校验
        if (EMERGENCY_WRITE_BACK == 1) {
            emergency_write_back(ui);
        }
    }
    if (ret == 0) {
        // cout << "SEC-DED未检测到错误！" << endl;
        if (caches[1][index].buf != 0xFFFF) {
            ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][index].buf)));
            ui->textBrowser->insertPlainText("\n");
            // error_but_not_detect_for_dirty++;
            all_dirty_error++;
            ui->textBrowser->insertPlainText("未检测到错误！\n");
            // 出现不能检测纠错，为了后续注入，这里再修改为正确的
            secded_encode_for_cache(index, 0xFFFF);
            caches[1][index].buf = 0xFFFF;
        }
    }
    if (ret == 1) {
        // dirty_line_error_count++;
        all_dirty_error++;
        ui->textBrowser->insertPlainText("SEC-DED检测到错误！\n");
        reverse(real_data.begin(), real_data.end());
        caches[1][index].buf = bin_to_dec(real_data);

        ui->textBrowser->insertPlainText("错误已修复！\n");
        ui->textBrowser->insertPlainText("正确数据为：");
        ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][index].buf)));
        ui->textBrowser->insertPlainText("\n");

        if (caches[1][index].buf != 0xFFFF) {
            // fail_correct++;
            // 出现不能检测纠错，为了后续注入，这里再修改为正确的
            secded_encode_for_cache(index, 0xFFFF);
            caches[1][index].buf = 0xFFFF;
        } else {
            dirty_line_error_correct_count++;
        }

        // 启用紧急回写机制，后续加MLREPS校验
        if (EMERGENCY_WRITE_BACK == 1) {
            emergency_write_back(ui);
        }
    }
}

string CacheSim::mlreps_encode(string s) {
	string d;
	char* hp = (char*)malloc(sizeof(char) * 17);
	memset(hp, 0, sizeof(char) * 17);
	char* lp = (char*)malloc(sizeof(char) * 49);
	memset(lp, 0, sizeof(char) * 49);
	char* vp = (char*)malloc(sizeof(char) * 25);
	memset(vp, 0, sizeof(char) * 25);
	hp[16] = '\0';
	lp[49] = '\0';
	vp[25] = '\0';

	int count = 0;
	int k = 0;
	for (int i = 0; i < 16; ++i) {
		count = (s[i + 0] - '0') ^ (s[i + 16] - '0') ^ (s[i + 32] - '0') ^ (s[i + 48] - '0');
		hp[i] = count + '0';
		count = (s[i + 0] - '0') ^ (s[i + 48] - '0');
		lp[k++] = count + '0';
		count = (s[i + 16] - '0') ^ (s[i + 48] - '0');
		lp[k++] = count + '0';
		count = (s[i + 32] - '0') ^ (s[i + 48] - '0');
		lp[k++] = count + '0';
	}

	k = 0;
	for (int i = 0; i < 24; i += 3) {
		count = (lp[i] - '0') ^ (lp[i + 24] - '0');
		vp[k++] = count + '0';
		count = (lp[i + 1] - '0') ^ (lp[i + 1 + 24] - '0');
		vp[k++] = count + '0';
		count = (lp[i + 2] - '0') ^ (lp[i + 2 + 24] - '0');
		vp[k++] = count + '0';
	}
	d = s + (string)hp + (string)vp;
	return d;
}

int CacheSim::mlreps_decode(string _d, string& s, Ui::MainWindow *ui) {
	string d = _d.substr(0, 64);
	string h_parity = _d.substr(64, 16);
	string v_parity = _d.substr(80, 24);

	char* lp = (char*)malloc(sizeof(char) * 49);
	memset(lp, 0, sizeof(char) * 49);
	lp[49] = '\0';

	string h_valid, v_valid;
	s = d;
	
	int count = 0;
	int k = 0;
	int n = 0;
	for (int i = 0; i < 16; ++i) {
		count = (d[i + 0] - '0') ^ (d[i + 16] - '0') ^ (d[i + 32] - '0') ^ (d[i + 48] - '0');
		if (((h_parity[i] - '0') ^ count) == 0) {
			h_valid.append(1, '0');
		} else {
			h_valid.append(1, '1');
			n++;
		}
		count = (d[i + 0] - '0') ^ (d[i + 48] - '0');
		lp[k++] = count + '0';
		count = (d[i + 16] - '0') ^ (d[i + 48] - '0');
		lp[k++] = count + '0';
		count = (d[i + 32] - '0') ^ (d[i + 48] - '0');
		lp[k++] = count + '0';
	}

	k = 0;
	for (int i = 0; i < 24; i += 3) {
		count = (lp[i] - '0') ^ (lp[i + 24] - '0');
		if ((count ^ (v_parity[k++] - '0')) == 0) {
			v_valid.append(1, '0');
		} else {
			v_valid.append(1, '1');
			n++;
		}
		count = (lp[i + 1] - '0') ^ (lp[i + 1 + 24] - '0');
		if ((count ^ (v_parity[k++] - '0')) == 0) {
			v_valid.append(1, '0');
		} else {
			v_valid.append(1, '1');
			n++;
		}
		count = (lp[i + 2] - '0') ^ (lp[i + 2 + 24] - '0');
		if ((count ^ (v_parity[k++] - '0')) == 0) {
			v_valid.append(1, '0');
		} else {
			v_valid.append(1, '1');
			n++;
		}
	}

//	if (n == 0) {
//		return 0;
//	}

    int ret = 0;
	k = 0;
	for (int i = 0; i < 16; ++i) {
		if (h_valid[i] == '1') {
			if ((v_valid[k] == '0' && v_valid[k + 1] == '0' && v_valid[k + 2] == '0') ||
				(v_valid[k] == '1' && v_valid[k + 1] == '1' && v_valid[k + 2] == '0') ||
				(v_valid[k] == '1' && v_valid[k + 1] == '0' && v_valid[k + 2] == '1') ||
				(v_valid[k] == '0' && v_valid[k + 1] == '1' && v_valid[k + 2] == '1')) {
                ui->textBrowser->insertPlainText("MLREPS无法确定具体出错的位置！\n");
				return -1;
			}
			if (v_valid[k] == '1' && v_valid[k + 1] == '0' && v_valid[k + 2] == '0') {
                ui->textBrowser->insertPlainText("MLREPS:E");
                ui->textBrowser->insertPlainText(QString::fromStdString(to_string(i + 0)));
                ui->textBrowser->insertPlainText("出现错误！\n");

				if (s[i] == '0') s[i] = '1';
				else s[i] = '0';
                ret = 1;
                ui->textBrowser->insertPlainText("错误已修复！\n");
			}
			if (v_valid[k] == '0' && v_valid[k + 1] == '1' && v_valid[k + 2] == '0') {
                ui->textBrowser->insertPlainText("MLREPS:E");
                ui->textBrowser->insertPlainText(QString::fromStdString(to_string(i + 16)));
                ui->textBrowser->insertPlainText("出现错误！\n");

				if (s[i + 16] == '0') s[i + 16] = '1';
				else s[i + 16] = '0';
                ret = 1;
                ui->textBrowser->insertPlainText("错误已修复！\n");
			}
			if (v_valid[k] == '0' && v_valid[k + 1] == '0' && v_valid[k + 2] == '1') {
                ui->textBrowser->insertPlainText("MLREPS:E");
                ui->textBrowser->insertPlainText(QString::fromStdString(to_string(i + 32)));
                ui->textBrowser->insertPlainText("出现错误！\n");

				if (s[i + 32] == '0') s[i + 32] = '1';
				else s[i + 32] = '0';
                ret = 1;
                ui->textBrowser->insertPlainText("错误已修复！\n");
			}
			if (v_valid[k] == '1' && v_valid[k + 1] == '1' && v_valid[k + 2] == '1') {
                ui->textBrowser->insertPlainText("MLREPS:E");
                ui->textBrowser->insertPlainText(QString::fromStdString(to_string(i + 48)));
                ui->textBrowser->insertPlainText("出现错误！\n");

				if (s[i + 48] == '0') s[i + 48] = '1';
				else s[i + 48] = '0';
                ret = 1;
                ui->textBrowser->insertPlainText("错误已修复！\n");
			}
		}	
		k += 3;
		if (i == 7) {
			k = 0;
		}
	}
	return ret;
}

void CacheSim::mlreps_encode_for_cache(_u64 index, _u64 data) {
    // MLREPS针对Cache line，但是每次写入是64位，并且设置Cache line最小为8字节，所以这里为64位数据编码
    // 64位数据编码需要40个校验位，其中16个水平奇偶校验位，24个垂直奇偶校验位
    caches[1][index].extra_mem = (_u8 *)malloc(sizeof(_u8) * 41);
    memset(caches[1][index].extra_mem, 0, sizeof(_u8) * 41);
    caches[1][index].extra_mem[40] = '\0';
    string str = dec_to_bin(data);
    string encode_str = mlreps_encode(str);
    for (int i = 64; i < encode_str.size(); ++i) {
		caches[1][index].extra_mem[i - 64] = encode_str[i];
	}
}

void CacheSim::mlreps_decode_for_cache(_u64 index, _u64 data, Ui::MainWindow *ui) {
    string decode_str = "";
    string str = dec_to_bin(data);
    string temp;
    for (int i = 0; i < 40; ++i) {
        temp += caches[1][index].extra_mem[i];
    }
    decode_str = str + temp;
    string real_data;
    int ret = mlreps_decode(decode_str, real_data, ui);
    if (ret < 0) {
        // dirty_line_error_count++;
        all_dirty_error++;
        // 出现不能纠错，为了后续注入，这里再修改为正确的
        mlreps_encode_for_cache(index, 0xFFFF);
        caches[1][index].buf = 0xFFFF;
        dirty_interval_time = 0;
    }
    if (ret == 0) {
        // cout << "MLREPS未检测到错误！" << endl;
        if (caches[1][index].buf != 0xFFFF) {
            // error_but_not_detect_for_dirty++;
            all_dirty_error++;
            ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][index].buf)));
            ui->textBrowser->insertPlainText("\n");
            ui->textBrowser->insertPlainText("未检测到错误！\n");
            // 出现不能检测纠错，为了后续注入，这里再修改为正确的
            // 但是相应的编码也应该修改，这里都没更新编码
            mlreps_encode_for_cache(index, 0xFFFF);
            caches[1][index].buf = 0xFFFF;
        }
    }
    if (ret == 1) {
        // 出现错误就重置错误间隔时间
        dirty_interval_time = 0;
        // dirty_line_error_count++;
        all_dirty_error++;
        reverse(real_data.begin(), real_data.end());
        caches[1][index].buf = bin_to_dec(real_data);
        ui->textBrowser->insertPlainText("正确数据为：");
        ui->textBrowser->insertPlainText(QString::fromStdString(to_string(caches[1][index].buf)));
        ui->textBrowser->insertPlainText("\n");

        if (caches[1][index].buf != 0xFFFF) {
            // fail_correct++;
            // 出现不能检测纠错，为了后续注入，这里再修改为正确的
            mlreps_encode_for_cache(index, 0xFFFF);
            caches[1][index].buf = 0xFFFF;
        } else {
            dirty_line_error_correct_count++;
        }
    }
}

/* 早期回写实现，参数为固定的周期 */
void CacheSim::early_write_back(_u64 time, Ui::MainWindow *ui) {
    // 初始设置time为10k时钟周期
    if (period >= time * 1024) {
        // 遍历所有的组，找到脏行，将其变为干净行，并且添加干净行的校验
        for (_u64 i = 0; i < cache_set_size[1] * cache_mapping_ways[1]; i++) {
            if (caches[1][i].flag & CACHE_FLAG_DIRTY) {
                // l2 to mem
                clean_count++;
                _u64 data = caches[1][i].buf;
                // 假设存放到内存的地址为m_data（全局变量）的地址
                // 对脏行解码
                if (!START_MLREPS_FLAG) {
                    if (SECDED == 1) hamming_decode_for_cache(i, caches[1][i].buf, ui);
                    if (SECDED == 2) secded_decode_for_cache(i, caches[1][i].buf, ui);
                } else {
                    if (MLREPS == 1) mlreps_decode_for_cache(i, caches[1][i].buf, ui);
                }
                caches[1][i].flag &= ~CACHE_FLAG_DIRTY;
                caches[1][i].addr = (_u64*)&m_data;
                parity_check_encode_for_cache(i, m_data);
                cache_w_memory_count++;
                period = period + 100 + 10;
                dirty_interval_time = dirty_interval_time + 10 + 100;
            }
        }
        period = 0;
        // MLREPS无出错间隔周期为20k，则之后脏数据转换为SEC-DED编码
        if (MLREPS == 1 && START_MLREPS_FLAG == true && dirty_interval_time >= 100 * 1024) {
            // cout << dirty_interval_time << endl;
            if (START_MLREPS_FLAG == true) {
                START_MLREPS_FLAG = false;
                // 如果再遇到错误，则紧急回写机制也生效
            }
        }
    }
}

/* 紧急回写实现，所有的脏数据都回写，不单单出错数据相近的 */
void CacheSim::emergency_write_back(Ui::MainWindow *ui) {
    // 缓冲周期结束，如果遇到错误可以再次回写
    // if (emergency_period == 0) {
    if (START_MLREPS_FLAG ==false) {
        // 设置紧急回写缓冲为1k时钟周期
        // 修改：紧急回写周期应该与MLREPS编码开启关闭有关
        // 即紧急回写开启->紧急回写关闭->MLREPS开启->MLREPS关闭->紧急回写开启
        // emergency_period = 1 * 1024;
        // 遍历所有的组，找到脏行，将其变为干净行，并且添加干净行的校验
        // 其实脏行有可能是出错的行，所以先把所有的错误纠正
        for (_u64 i = 0; i < cache_set_size[1] * cache_mapping_ways[1]; ++i) {
            if (caches[1][i].flag & CACHE_FLAG_DIRTY) {
                // l2 to mem
                clean_count++;
                _u64 data = caches[1][i].buf;
                // 假设存放到内存的地址为m_data（全局变量）的地址
                // 对脏行解码
                if (!START_MLREPS_FLAG) {
                    if (SECDED == 1) hamming_decode_for_cache(i, caches[1][i].buf, ui);
                    if (SECDED == 2) secded_decode_for_cache(i, caches[1][i].buf, ui);
                } else {
                    if (MLREPS == 1) mlreps_decode_for_cache(i, caches[1][i].buf, ui);
                }
                caches[1][i].flag &= ~CACHE_FLAG_DIRTY;
                caches[1][i].addr = (_u64*)&m_data;
                parity_check_encode_for_cache(i, m_data);
                cache_w_memory_count++;
                period = period + 100 + 10;
                dirty_interval_time = dirty_interval_time + 100 + 10;
            }
        }
        period = 0;
        if (MLREPS == 1) {
            if (START_MLREPS_FLAG == false) {
                START_MLREPS_FLAG = true;
            }
        }
    }
}

int* CacheSim::rand_0(int min, int max, int n) {
	vector<int> temp;
	for (int i = min; i <= max; ++i) {
		temp.push_back(i);
	}
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	shuffle(temp.begin(), temp.end(), default_random_engine(seed));
	int* trigger_time = (int*)malloc(sizeof(int) * n);
	memset(trigger_time, 0, sizeof(int) * n);
	for (int i = 0; i < n; ++i) {
		trigger_time[i] = temp[i];
	}
	sort(trigger_time, trigger_time + n);
	return trigger_time;
}

int* CacheSim::rand_1(int min, int max, int n) {
	vector<int> temp;
	for (int i = min; i <= max; ++i) {
		temp.push_back(i);
	}
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	shuffle(temp.begin(), temp.end(), default_random_engine(seed));
	int* index = (int*)malloc(sizeof(int) * n);
	memset(index, 0, sizeof(int) * n);
	for (int i = 0; i < n; ++i) {
		index[i] = temp[i % 31];
	}
	return index;
}

int* CacheSim::cache_error_inject(_u64 min, _u64 max, _u64 n, int error_type) {
	int* error_struct = (int*)malloc(sizeof(int) * 20 * n);
	memset(error_struct, 0, sizeof(int) * 20 * n);
	int* trigger_time = (int*)malloc(sizeof(int) * n);
	memset(trigger_time, 0, sizeof(int) * n);
	int* line = (int*)malloc(sizeof(int) * n);
	memset(line, 0, sizeof(int) * n);
	int* index = (int*)malloc(sizeof(int) * n);
	memset(index, 0, sizeof(int) * n);

	trigger_time = rand_0(min, max, n);
	line = rand_1(0, 1023, n);
    index = rand_1(2, 61, n);

    int k = 0;
	for (int i = 0; i < n; ++i) {
		error_struct[k++] = trigger_time[i];
		if (error_type == 1) {
			error_struct[k++] = error_type;
			error_struct[k++] = line[i];
			error_struct[k++] = index[i];
		}
        if (error_type == 2) {
            error_struct[k++] = error_type;
			error_struct[k++] = line[i];
			error_struct[k++] = index[i];
            error_struct[k++] = index[i] + 1;
        }
        if (error_type == 3) {
            error_struct[k++] = error_type;
			error_struct[k++] = line[i];
			error_struct[k++] = index[i] - 1;
            error_struct[k++] = index[i];
            error_struct[k++] = index[i] + 1;
        }
        if (error_type == 4) {
			error_struct[k++] = error_type;
			error_struct[k++] = line[i];
			error_struct[k++] = index[i] - 1;
			error_struct[k++] = index[i];
			error_struct[k++] = index[i] + 1;
			error_struct[k++] = index[i] + 2;
		}
        if (error_type == 5) {
            error_struct[k++] = error_type;
            error_struct[k++] = line[i];
            error_struct[k++] = index[i] - 2;
            error_struct[k++] = index[i] - 1;
            error_struct[k++] = index[i];
            error_struct[k++] = index[i] + 1;
            error_struct[k++] = index[i] + 2;
        }
        if (error_type == 12) {
            error_struct[k++] = 12;
			if (line[i] % 2 == 1) {
				error_struct[k++] = 1;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
			}
			if (line[i] % 2 == 0) {
				error_struct[k++] = 2;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
				error_struct[k++] = index[(i + 1) % n];
			}
		}
        if (error_type == 13) {
            error_struct[k++] = 13;
			if (line[i] % 3 == 1) {
				error_struct[k++] = 1;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
			}
			if (line[i] % 3 == 2) {
				error_struct[k++] = 2;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
				error_struct[k++] = index[(i + 1) % n];
			}
			if (line[i] % 3 == 0) {
				error_struct[k++] = 3;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
				error_struct[k++] = index[(i + 1) % n];
				error_struct[k++] = index[(i + 2) % n];
			}
		}
        if (error_type == 14) {
            error_struct[k++] = 14;
			if (line[i] % 4 == 1) {
				error_struct[k++] = 1;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
			}
			if (line[i] % 4 == 2) {
				error_struct[k++] = 2;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
				error_struct[k++] = index[(i + 1) % n];
			}
			if (line[i] % 4 == 3) {
				error_struct[k++] = 3;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
				error_struct[k++] = index[(i + 1) % n];
				error_struct[k++] = index[(i + 2) % n];
			}
			if (line[i] % 4 == 0) {
				error_struct[k++] = 4;
				error_struct[k++] = line[i];
				error_struct[k++] = index[i];
				error_struct[k++] = index[(i + 1) % n];
				error_struct[k++] = index[(i + 2) % n];
				error_struct[k++] = index[(i + 3) % n];
			}
		}
        if (error_type == 15) {
            error_struct[k++] = 15;
            if (line[i] % 5 == 1) {
                error_struct[k++] = 1;
                error_struct[k++] = line[i];
                error_struct[k++] = index[i];
            }
            if (line[i] % 5 == 2) {
                error_struct[k++] = 2;
                error_struct[k++] = line[i];
                error_struct[k++] = index[i];
                error_struct[k++] = index[(i + 1) % n];
            }
            if (line[i] % 5 == 3) {
                error_struct[k++] = 3;
                error_struct[k++] = line[i];
                error_struct[k++] = index[i];
                error_struct[k++] = index[(i + 1) % n];
                error_struct[k++] = index[(i + 2) % n];
            }
            if (line[i] % 5 == 4) {
                error_struct[k++] = 4;
                error_struct[k++] = line[i];
                error_struct[k++] = index[i];
                error_struct[k++] = index[(i + 1) % n];
                error_struct[k++] = index[(i + 2) % n];
                error_struct[k++] = index[(i + 3) % n];
            }
            if (line[i] % 5 == 0) {
                error_struct[k++] = 5;
                error_struct[k++] = line[i];
                error_struct[k++] = index[i];
                error_struct[k++] = index[(i + 1) % n];
                error_struct[k++] = index[(i + 2) % n];
                error_struct[k++] = index[(i + 3) % n];
                error_struct[k++] = index[(i + 4) % n];
            }
        }
	}
	return error_struct;
}
