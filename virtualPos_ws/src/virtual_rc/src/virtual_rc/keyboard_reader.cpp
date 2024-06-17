/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-03-29 10:55:34
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-03-29 10:55:39
 */

#include "virtual_rc/keyboard_reader.h"

KeyboardReader::KeyboardReader()
{
    initscr();             // 初始化ncurses
    cbreak();              // 禁用行缓冲，立即发送每个键
    noecho();              // 不在终端上显示按键
    keypad(stdscr, TRUE);  // 特殊键位支持
    nodelay(stdscr, TRUE); // 非阻塞读取
}

KeyboardReader::~KeyboardReader()
{
    endwin(); // 结束ncurses模式
}

void KeyboardReader::update()
{
    int ch = getch();
    while (ch != ERR)
    {
        keyStates[ch] = true;
        ch = getch(); // 继续读取，直到缓冲区为空
    }
}

bool KeyboardReader::keyPressed(int key)
{
    return keyStates.find(key) != keyStates.end() && keyStates[key];
}

void KeyboardReader::reset()
{
    keyStates.clear();
}