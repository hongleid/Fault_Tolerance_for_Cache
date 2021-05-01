#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "CacheSimulator.h"
#include <cstring>
#include <iostream>
using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowFlags(windowFlags()& ~Qt::WindowMaximizeButtonHint);
    setFixedSize(this->width(), this->height());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_start_btn_clicked()
{
    char test_case[100] = "";
    string cases = ui->box_file->currentText().toStdString();
    strcpy(test_case, cases.c_str());

    string l1_size = ui->box_l1_size->currentText().toStdString();
    string l2_size = ui->box_l2_size->currentText().toStdString();
    string l1_line_size = ui->box_l1_line->currentText().toStdString();
    string l2_line_size = ui->box_l2_line->currentText().toStdString();
    string l1_ways = ui->box_l1_way->currentText().toStdString();
    string l2_ways = ui->box_l2_way->currentText().toStdString();
    string l1_replacement_strategy = ui->box_replace1->currentText().toStdString();
    string l2_replacement_strategy = ui->box_replace2->currentText().toStdString();

    string parity_str = ui->box_parity->currentText().toStdString();
    string secded_str = ui->box_secded->currentText().toStdString();
    string mlreps_str = ui->box_mlreps->currentText().toStdString();
    string early_write_str = ui->box_early->currentText().toStdString();
    string emergency_write_str = ui->box_emergency->currentText().toStdString();

    string error_type_str = ui->box_fault->currentText().toStdString();
    string inject_time_str = ui->box_inject->currentText().toStdString();

    // get cache config
    CacheSim cache;
    _u64 line_size[] = {0, 0};
    _u64 ways[] = {0, 0};
    _u64 cache_size[] = {0, 0};
    int l1_replace = 0;
    int l2_replace = 0;
    cache_size[0] = atoi(l1_size.c_str());
    cache_size[1] = atoi(l2_size.c_str());
    line_size[0] = atoi(l1_line_size.c_str());
    line_size[1] = atoi(l2_line_size.c_str());
    ways[0] = atoi(l1_ways.c_str());
    ways[1] = atoi(l2_ways.c_str());

    if (l1_replacement_strategy == "LRU") {
        l1_replace = 0;
    } else if (l1_replacement_strategy == "FIFO") {
        l1_replace = 1;
    } else if (l1_replacement_strategy == "RAND") {
        l1_replace = 2;
    }

    if (l2_replacement_strategy == "LRU") {
        l2_replace = 0;
    } else if (l2_replacement_strategy == "FIFO") {
        l2_replace = 1;
    } else if (l2_replacement_strategy == "RAND") {
        l2_replace = 2;
    }

    // get tolerance config
    int parity = 0;
    int secded = 0;
    int mlreps = 0;
    int early_write = 0;
    int emergency_write = 0;

    if (parity_str == "disable") {
        parity = 0;
    } else if (parity_str == "ordinary") {
        parity = 1;
    } else if (parity_str == "cross") {
        parity = 2;
    }

    if (secded_str == "disable") {
        secded = 0;
    } else if (secded_str == "hamming") {
        secded = 1;
    } else if (secded_str == "secded") {
        secded = 2;
    }

    if (mlreps_str == "disable") {
        mlreps = 0;
    } else if (mlreps_str == "enable") {
        mlreps = 1;
    }

    if (early_write_str == "disable") {
        early_write = 0;
    } else if (early_write_str == "enable") {
        early_write = 1;
    }

    if (emergency_write_str == "disable") {
        emergency_write = 0;
    } else if (emergency_write_str == "enable") {
        emergency_write = 1;
    }

    // get error inject config
    int error_type = 0;
    int inject_times = 0;

    if (error_type_str == "disable") {
        error_type = 0;
    } else if (error_type_str == "SBU") {
        error_type = 1;
    } else if (error_type_str == "adjacent 2") {
        error_type = 2;
    } else if (error_type_str == "adjacent 3") {
        error_type = 3;
    } else if (error_type_str == "adjacent 4") {
        error_type = 4;
    } else if (error_type_str == "adjacent 5") {
        error_type = 5;
    } else if (error_type_str == "random 1-2") {
        error_type = 12;
    } else if (error_type_str == "random 1-3") {
        error_type = 13;
    } else if (error_type_str == "random 1-4") {
        error_type = 14;
    } else if (error_type_str == "random 1-5") {
        error_type = 15;
    }

    if (inject_time_str == "10000") {
        inject_times = 10000;
    } else if (inject_time_str == "3000") {
        inject_times = 3000;
    } else if (inject_time_str == "6000") {
        inject_times = 6000;
    } else if (inject_time_str == "9000") {
        inject_times = 9000;
    } else if (inject_time_str == "12000") {
        inject_times = 12000;
    } else if (inject_time_str == "15000") {
        inject_times = 15000;
    }

    // go on
    cache.init(cache_size, line_size, ways, l1_replace, l2_replace, parity, secded,
               mlreps, early_write, emergency_write, error_type, inject_times, ui);

    cache.load_trace(test_case, ui);
}
