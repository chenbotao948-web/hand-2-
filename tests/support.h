#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H
#include "main.h"
volatile unsigned test_checks, test_failed_line;
/* 仿真器直接设置 SP 并进入 main；满足 microlib 未使用入口的链接符号。 */
unsigned __initial_sp;
#define CHECK(condition) do { test_checks++; if (!(condition)) { test_failed_line = __LINE__; return 1; } } while (0)
#endif
