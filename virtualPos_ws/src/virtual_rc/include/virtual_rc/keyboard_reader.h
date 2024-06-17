/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-03-29 10:55:34
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-03-29 10:55:39
 */

#ifndef KEYBOARD_READER_H_
#define KEYBOARD_READER_H_

#include <vector>
#include <ros/ros.h>
#include <ncurses.h>
#include <unordered_map>

class KeyboardReader
{
public:
    KeyboardReader();
    KeyboardReader(const KeyboardReader &rhs) = delete;
    KeyboardReader &operator=(const KeyboardReader &rhs) = delete;
    KeyboardReader(KeyboardReader &&rhs) = delete;
    KeyboardReader &operator=(KeyboardReader &&rhs) = delete;
    virtual ~KeyboardReader();

    void update();
    bool keyPressed(int key);
    void reset();

private:
    std::unordered_map<int, bool> keyStates;
};

#endif // KEYBOARD_READER_H_